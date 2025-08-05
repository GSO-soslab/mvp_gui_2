# mvp_gui/nodes/ros_interface_node.py
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import socketio as sio_client_lib
import time
import threading
import numpy as np
import message_filters
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoseStamped
from mvp_msgs.msg import Waypoint, Waypoints, HelmState
from std_msgs.msg import Float32MultiArray, Bool, Int16MultiArray
from tf_transformations import euler_from_quaternion
from mvp_msgs.srv import SetString, SendWaypoints
from std_srvs.srv import SetBool
from rcl_interfaces.srv import GetParameters

class RosInterfaceNode(Node):
    def __init__(self, sio_client):
        super().__init__('ros_interface_node')
        self.sio = sio_client
        self.dynamic_clients_configured = False

        self.get_logger().info('ROS Interface Node started. Setting up communications...')

        # --- Declare and Get ROS Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('topic_ns', '/default_ns/'),
                ('service_ns', '/default_ns/'),
                ('poses_source', 'odometry'),
                ('geo_pose_source', 'geopose'),
                ('survey_geopath_source', 'survey/geopath'),
                ('helm_state_get', 'helm/state'),
                ('controller_state_get', 'controller_state'),
                ('gpio_power_get', 'gpio_power_state'),
                ('roslaunch_state_get', 'roslaunch_state'),
                ('computer_info_source', 'computer_info'),
                ('power_info_source', 'power_info'),
                ('helm_state_set', 'mvp_helm/change_state'),
                ('controller_state_set', 'controller/set'),
                ('pub_waypoints_service', 'mvp_helm/set_waypoints'),
                ('set_power_service', 'gpio_manager/set_power/'),
                ('c2_commander_node', ''),
            ]
        )
        self.topic_ns = self.get_parameter('topic_ns').value
        self.service_ns = self.get_parameter('service_ns').value
        self.c2_commander_node_name = self.get_parameter('c2_commander_node').value

        self.power_clients = {}
        self.launch_clients = {}
        self.launch_keys = []
        self.gpio_device_keys = []

        self.setup_ros_communications()
        self.setup_sio_handlers()

        # --- Asynchronously fetch parameters from C2 Commander node ---
        if self.c2_commander_node_name:
            self.get_logger().info(f"Will attempt to fetch parameters from '{self.c2_commander_node_name}'.")
            self.param_fetch_timer = self.create_timer(1.0, self.try_fetch_and_setup_dynamic_clients)
        else:
            self.get_logger().error("'c2_commander_node' not specified in config. Cannot setup dynamic clients.")

    def get_topic(self, key):
        return self.topic_ns + self.get_parameter(key).value

    def get_service(self, key):
        return self.service_ns + self.get_parameter(key).value

    def setup_ros_communications(self):
        self.callback_group = ReentrantCallbackGroup()
        
        # --- Synchronized Pose Subscribers ---
        odom_topic = self.get_topic('poses_source')
        geopose_topic = self.get_topic('geo_pose_source')
        odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)
        geopose_sub = message_filters.Subscriber(self, GeoPoseStamped, geopose_topic)
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, geopose_sub], queue_size=10, slop=0.2
        )
        self.time_synchronizer.registerCallback(self.synchronized_pose_callback)
        self.get_logger().info(f"Synchronizing pose on odom topic '{odom_topic}' and geopose topic '{geopose_topic}'.")
        
        # --- Other Subscribers ---
        self.create_subscription(Float32MultiArray, self.get_topic('power_info_source'), self.power_info_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Float32MultiArray, self.get_topic('computer_info_source'), self.computer_info_callback, 10, callback_group=self.callback_group)
        self.create_subscription(HelmState, self.get_topic('helm_state_get'), self.helm_state_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Bool, self.get_topic('controller_state_get'), self.controller_state_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Int16MultiArray, self.get_topic('gpio_power_get'), self.power_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Int16MultiArray, self.get_topic('roslaunch_state_get'), self.roslaunch_state_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Waypoints, self.get_topic('survey_geopath_source'), self.survey_geopath_callback, 10, callback_group=self.callback_group)

        # --- Static Service Clients ---
        self.set_helm_state_client = self.create_client(SetString, self.get_service('helm_state_set'), callback_group=self.callback_group)
        self.set_controller_state_client = self.create_client(SetBool, self.get_service('controller_state_set'), callback_group=self.callback_group)
        self.pub_waypoints_client = self.create_client(SendWaypoints, self.get_service('pub_waypoints_service'), callback_group=self.callback_group)

    def try_fetch_and_setup_dynamic_clients(self):
        if self.param_fetch_timer: self.param_fetch_timer.cancel()
        if self.dynamic_clients_configured: return

        self.get_logger().info(f"Attempting to fetch parameters from '{self.c2_commander_node_name}'...")
        param_client = self.create_client(GetParameters, f'{self.c2_commander_node_name}/get_parameters', callback_group=self.callback_group)
        if not param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Parameter service for '{self.c2_commander_node_name}' not available. Retrying in 5s...")
            self.param_fetch_timer = self.create_timer(5.0, self.try_fetch_and_setup_dynamic_clients)
            return

        req = GetParameters.Request(names=['gpio_devices', 'launch_packages', 'launch_files'])
        future = param_client.call_async(req)
        future.add_done_callback(self.process_param_fetch_result)

    def process_param_fetch_result(self, future):
        try:
            result = future.result()
            params = {name: value for name, value in zip(['gpio_devices', 'launch_packages', 'launch_files'], result.values)}
            if not all(p in params and params[p].type != 0 for p in params):
                raise ValueError("Missing required parameters")

            gpio_devices = params['gpio_devices'].string_array_value
            launch_packages = params['launch_packages'].string_array_value
            launch_files = params['launch_files'].string_array_value
            self.get_logger().info(f"Fetched parameters: gpio_devices={list(gpio_devices)}, launch_files={list(launch_files)}")

            base_power_srv = self.get_service('set_power_service')
            self.gpio_device_keys = list(gpio_devices)
            for device_name in self.gpio_device_keys:
                if device_name not in self.power_clients:
                    self.power_clients[device_name] = self.create_client(SetBool, f"{base_power_srv}{device_name}", callback_group=self.callback_group)

            self.launch_keys = [f"{p}/{f}" for p, f in zip(launch_packages, launch_files)]
            for launch_key in self.launch_keys:
                if launch_key not in self.launch_clients:
                    self.launch_clients[launch_key] = self.create_client(SetBool, f"{self.service_ns}roslaunch/{launch_key}", callback_group=self.callback_group)

            self.get_logger().info(f"Dynamically created {len(self.power_clients)} power and {len(self.launch_clients)} launch clients.")
            self.dynamic_clients_configured = True
            if self.sio.connected:
                self.sio.emit('update_launch_keys', {'keys': self.launch_keys})
        except Exception as e:
            self.get_logger().error(f"Failed to process parameter fetch result: {e}. Retrying in 5s...")
            self.param_fetch_timer = self.create_timer(5.0, self.try_fetch_and_setup_dynamic_clients)

    def setup_sio_handlers(self):
        @self.sio.on('ros_action')
        def handle_ros_action(data):
            action = data.get('action')
            self.get_logger().info(f"Received forwarded GUI action '{action}'")
            actions = {
                'change_state': lambda d: self.call_set_helm_state(d.get('value')),
                'controller_state': lambda d: self.call_set_controller_state(d.get('value')),
                'publish_waypoints': lambda d: self.call_publish_waypoints(d.get('waypoints')),
                'set_power': lambda d: self.call_set_power(d.get('name'), d.get('status')),
                'launch_file': lambda d: self.call_launch_file(d.get('key'), d.get('status')),
            }
            if action in actions:
                actions[action](data)

    def survey_geopath_callback(self, msg):
        if not self.sio.connected: return
        path_data = [{'lat': w.ll_wpt.latitude, 'lon': w.ll_wpt.longitude, 'alt': w.ll_wpt.altitude, 'surge': w.u} for w in msg.wpt]
        self.sio.emit('published_path_update', path_data)

    def roslaunch_state_callback(self, msg):
        if not self.sio.connected: return
        statuses = list(msg.data)
        if len(statuses) != len(self.launch_keys):
            self.get_logger().warning(f"Launch status size ({len(statuses)}) != launch files ({len(self.launch_keys)}).")
            return
        self.sio.emit('launch_status_update', {'keys': self.launch_keys, 'statuses': statuses})

    def power_callback(self, msg):
        if not self.sio.connected: return
        self.sio.emit('power_update', {'keys': self.gpio_device_keys,  'statuses': list(msg.data)})

    def power_info_callback(self, msg):
        if not self.sio.connected: return
        self.sio.emit('power_info_update', {'voltage': msg.data[0], 'current': msg.data[1]})

    def computer_info_callback(self, msg):
        if not self.sio.connected: return
        self.sio.emit('computer_info_update', {'mem_usage': msg.data[0], 'cpu_temp': msg.data[1], 'cpu_usage': msg.data[2]})

    def helm_state_callback(self, msg):
        if not self.sio.connected: return
        self.sio.emit('helm_state_update', {'current_state': msg.name, 'transitions': msg.transitions})

    def controller_state_callback(self, msg):
        if not self.sio.connected: return
        self.sio.emit('controller_state_update', {'state': msg.data})

    def synchronized_pose_callback(self, odom_msg, geo_pose_msg):
        if not self.sio.connected: return
        q = geo_pose_msg.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        pose_data = {
            "lat": geo_pose_msg.pose.position.latitude, "lon": geo_pose_msg.pose.position.longitude, "alt": geo_pose_msg.pose.position.altitude,
            "roll": np.rad2deg(euler[0]), "pitch": np.rad2deg(euler[1]), "yaw": np.rad2deg(euler[2]),
            "x": odom_msg.pose.pose.position.x, "y": odom_msg.pose.pose.position.y, "z": odom_msg.pose.pose.position.z,
            "u": odom_msg.twist.twist.linear.x, "v": odom_msg.twist.twist.linear.y, "w": odom_msg.twist.twist.linear.z,
            "p": np.rad2deg(odom_msg.twist.twist.angular.x), "q": np.rad2deg(odom_msg.twist.twist.angular.y), "r": np.rad2deg(odom_msg.twist.twist.angular.z),
        }
        self.sio.emit('vehicle_pose_update', pose_data)

    def _check_service(self, client, timeout_sec=1.0):
        if not client.wait_for_service(timeout_sec=timeout_sec): 
            self.get_logger().error(f"Service '{client.srv_name}' not available.")
            return False
        return True

    def call_launch_file(self, launch_key, status):
        client = self.launch_clients.get(launch_key)
        if client and self._check_service(client):
            self.get_logger().info(f"Calling launch service for '{launch_key}' with status: {status}")
            client.call_async(SetBool.Request(data=bool(status)))

    def call_set_power(self, name, status):
        client = self.power_clients.get(name)
        if client and self._check_service(client):
            client.call_async(SetBool.Request(data=bool(status)))

    def call_set_helm_state(self, state_name):
        if self._check_service(self.set_helm_state_client):
            self.set_helm_state_client.call_async(SetString.Request(data=state_name))

    def call_set_controller_state(self, value):
        if self._check_service(self.set_controller_state_client):
            self.set_controller_state_client.call_async(SetBool.Request(data=bool(value)))

    def call_publish_waypoints(self, waypoints_data):
        if self._check_service(self.pub_waypoints_client):
            req = SendWaypoints.Request(type='waypoint')
            for wp_data in waypoints_data:
                wpt = Waypoint()
                wpt.ll_wpt.latitude = wp_data['lat']
                wpt.ll_wpt.longitude = wp_data['lon']
                wpt.ll_wpt.altitude = wp_data['alt']
                wpt.u = wp_data['surge']
                req.wpt.append(wpt)
            self.pub_waypoints_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    
    # Enable logging for the client to see connection details
    sio = sio_client_lib.Client(logger=True, engineio_logger=True)

    @sio.event
    def connect():
        print("ROS Interface connected successfully to WebSocket server.")

    @sio.event
    def connect_error(data):
        print(f"Connection to WebSocket server failed: {data}")

    @sio.event
    def disconnect():
        print("ROS Interface disconnected from WebSocket server.")

    server_url = 'http://localhost:5001'
    # Loop to attempt connection as long as ROS is running
    while rclpy.ok():
        try:
            print(f"Attempting to connect to {server_url}...")
            sio.connect(server_url)
            break  # If connection is successful, exit the loop
        except sio_client_lib.exceptions.ConnectionError:
            print("Connection failed. Retrying in 3 seconds...")
            time.sleep(3)
        except Exception as e:
            print(f"An unexpected error occurred during connection: {e}")
            time.sleep(3)

    # After the loop, check if we should proceed
    if not rclpy.ok() or not sio.connected:
        print("Could not connect to WebSocket server or ROS was shut down. Shutting down node.")
        if sio.connected:
            sio.disconnect()
        if rclpy.ok():
            rclpy.shutdown()
        return

    # Create and spin the ROS node
    ros_node = RosInterfaceNode(sio)
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # sio.wait() blocks until the client is disconnected.
        sio.wait()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received.")
    finally:
        print("Shutting down ros_interface_node...")
        ros_node.destroy_node()
        executor.shutdown()
        if sio.connected:
            sio.disconnect()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()