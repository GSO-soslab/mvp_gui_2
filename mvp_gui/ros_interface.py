import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import socketio as sio_client_lib # Use the full library name for clarity
import time
import threading
import numpy as np
import yaml
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoseStamped, GeoPath
from mvp_msgs.msg import Waypoint, Waypoints, ControlProcess, HelmState
from std_msgs.msg import Float64, Float32MultiArray, Int16, Bool, Int16MultiArray
from tf_transformations import euler_from_quaternion
from mvp_msgs.srv import SetString, SendWaypoints
from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.srv import GetParameters
import os
import logging
# New import for message synchronization
import message_filters

# logging.basicConfig(level=logging.DEBUG)

class RosInterfaceNode(Node):
    def __init__(self, sio_client):
        super().__init__('mvp_gui_interface_node')
        self.sio = sio_client
        self.dynamic_clients_configured = False # Flag to ensure we configure only once

        # --- Load Main Configuration ---
        config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'config.yaml')
        with open(config_path, 'r') as file:
            self.yaml_config = yaml.safe_load(file)
        
        # --- C2 Configuration will be fetched from ROS parameters ---
        self.c2_commander_node_name = self.yaml_config.get('c2_commander_node')
        
        self.get_logger().info('ROS Interface Node started. Setting up communications...')
        self.power_clients = {}
        self.launch_clients = {}
        self.launch_keys = [] # Store the ordered list of launch keys
        self.gpio_device_keys = []
        self.setup_ros_communications()
        self.setup_sio_handlers()

        # --- Asynchronously fetch parameters from C2 Commander node ---
        if self.c2_commander_node_name:
            self.get_logger().info(f"Will attempt to fetch parameters from '{self.c2_commander_node_name}'.")
            # This timer will try to fetch params, and reschedule itself on failure.
            self.param_fetch_timer = self.create_timer(1.0, self.try_fetch_and_setup_dynamic_clients)
        else:
            self.get_logger().error("'c2_commander_node' not specified in config.yaml. Cannot setup dynamic clients.")

    def setup_ros_communications(self):
        """
        This function sets up subscribers and clients. Dynamic clients for
        power and launch are set up later after fetching parameters.
        """
        self.callback_group = ReentrantCallbackGroup()
        
        self.topic_ns = self.yaml_config.get('topic_ns', '/default_ns/')
        self.service_ns = self.yaml_config.get('service_ns', '/default_ns/')
        
        # --- Synchronized Pose Subscribers ---
        # Using message_filters to synchronize Odometry and GeoPoseStamped messages
        odom_topic = self.topic_ns + self.yaml_config.get('poses_source', 'odometry')
        geopose_topic = self.topic_ns + self.yaml_config.get('geo_pose_source', 'geopose')

        odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)
        geopose_sub = message_filters.Subscriber(self, GeoPoseStamped, geopose_topic)

        # ApproximateTimeSynchronizer will find pairs of messages that are close in time.
        # The 'slop' parameter defines the maximum time difference (in seconds)
        # for messages to be considered a match.
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, geopose_sub],
            queue_size=10,
            slop=0.2
        )
        self.time_synchronizer.registerCallback(self.synchronized_pose_callback)
        self.get_logger().info(f"Synchronizing pose on odom topic '{odom_topic}' and geopose topic '{geopose_topic}'.")
        
        # --- Other Subscribers ---
        self.create_subscription(Float32MultiArray, self.topic_ns + self.yaml_config.get('power_info_source', 'power_info'), self.power_info_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Float32MultiArray, self.topic_ns + self.yaml_config.get('computer_info_source', 'computer_info'), self.computer_info_callback, 10, callback_group=self.callback_group)
        self.create_subscription(HelmState, self.topic_ns + self.yaml_config.get('helm_state_get', 'helm/state'), self.helm_state_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Bool, self.topic_ns + self.yaml_config.get('controller_state_get', 'controller_state'), self.controller_state_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Int16MultiArray, self.topic_ns + self.yaml_config.get('gpio_power_get', 'gpio_power_state'), self.power_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Int16MultiArray, self.topic_ns + self.yaml_config.get('roslaunch_state_get', 'roslaunch_state'), self.roslaunch_state_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Waypoints, self.topic_ns + self.yaml_config.get('survey_geopath_source', 'survey/geopath'), self.survey_geopath_callback, 10, callback_group=self.callback_group)

        # --- Static Service Clients ---
        self.set_helm_state_client = self.create_client(SetString, self.service_ns + self.yaml_config.get('helm_state_set', 'mvp_helm/change_state'), callback_group=self.callback_group)
        self.set_controller_state_client = self.create_client(SetBool, self.service_ns + self.yaml_config.get('controller_state_set', 'controller/set'), callback_group=self.callback_group)
        self.pub_waypoints_client = self.create_client(SendWaypoints, self.service_ns + self.yaml_config.get('pub_waypoints_service', 'mvp_helm/set_waypoints'), callback_group=self.callback_group)
        
    def try_fetch_and_setup_dynamic_clients(self):
        # Cancel the timer so it doesn't fire again while we're processing.
        if self.param_fetch_timer:
            self.param_fetch_timer.cancel()
            self.param_fetch_timer = None
        
        if self.dynamic_clients_configured:
            return

        self.get_logger().info(f"Attempting to fetch parameters from '{self.c2_commander_node_name}'...")
        
        param_client = self.create_client(GetParameters, f'{self.c2_commander_node_name}/get_parameters', callback_group=self.callback_group)
        if not param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Parameter service for '{self.c2_commander_node_name}' not available. Retrying in 5 seconds...")
            self.param_fetch_timer = self.create_timer(5.0, self.try_fetch_and_setup_dynamic_clients)
            return

        req = GetParameters.Request()
        req.names = ['gpio_devices', 'launch_packages', 'launch_files']
        future = param_client.call_async(req)
        future.add_done_callback(self.process_param_fetch_result)

    def process_param_fetch_result(self, future):
        try:
            result = future.result()
            
            # These are the names we requested.
            requested_names = ['gpio_devices', 'launch_packages', 'launch_files']
            
            # CORRECTED: Manually pair the requested names with the returned values.
            params = {name: value for name, value in zip(requested_names, result.values)}
            
            # Now we can check if we got valid values for all requested params.
            if not all(p in params and params[p].type != 0 for p in requested_names):
                self.get_logger().error(f"Could not retrieve all required parameters from '{self.c2_commander_node_name}'. Retrying...")
                raise ValueError("Missing required parameters")

            gpio_devices = params['gpio_devices'].string_array_value
            launch_packages = params['launch_packages'].string_array_value
            launch_files = params['launch_files'].string_array_value
            
            self.get_logger().info(f"Successfully fetched parameters: gpio_devices={list(gpio_devices)}, launch_packages={list(launch_packages)}, launch_files={list(launch_files)}")
            
            # --- Dynamically Create Power Service Clients ---
            base_power_srv = self.service_ns + self.yaml_config.get('set_power_service', 'gpio_manager/set_power/')
            for device_name in gpio_devices:
                self.gpio_device_keys.append(device_name)
                if device_name not in self.power_clients:
                    full_service_name = f"{base_power_srv}{device_name}"
                    self.power_clients[device_name] = self.create_client(SetBool, full_service_name, callback_group=self.callback_group)

            # --- Dynamically Create Launch Service Clients ---
            self.launch_keys = [] # Clear and rebuild
            for package, filename in zip(launch_packages, launch_files):
                launch_key = f"{package}/{filename}"
                self.launch_keys.append(launch_key)
                if launch_key not in self.launch_clients:
                    full_service_name = f"{self.service_ns}roslaunch/{launch_key}"
                    self.launch_clients[launch_key] = self.create_client(SetBool, full_service_name, callback_group=self.callback_group)

            self.get_logger().info(f"Dynamically created {len(self.power_clients)} power clients and {len(self.launch_clients)} launch clients.")
            self.dynamic_clients_configured = True
            
            # Send the list of launch keys to the Flask server to cache.
            if self.sio.connected:
                self.sio.emit('update_launch_keys', {'keys': self.launch_keys})
                self.sio.emit('update_launch_keys', {'keys': self.launch_keys})

        except Exception as e:
            self.get_logger().error(f"Failed to process parameter fetch result: {e}. Retrying in 5 seconds...")
            self.param_fetch_timer = self.create_timer(5.0, self.try_fetch_and_setup_dynamic_clients)

    def setup_sio_handlers(self):
        # This handler listens for commands relayed from the browser
        @self.sio.on('ros_action')
        def handle_ros_action(data):
            action = data.get('action')
            self.get_logger().info(f"Received forwarded GUI action '{action}', data: {data}")
            if action == 'change_state': 
                self.call_set_helm_state(data.get('value'))
            elif action == 'controller_state': 
                self.call_set_controller_state(data.get('value'))
            elif action == 'publish_waypoints': 
                self.call_publish_waypoints(data.get('waypoints'))
            elif action == 'set_power': 
                self.call_set_power(data.get('name'), data.get('status'))
            elif action == 'launch_file': 
                self.call_launch_file(data.get('key'), data.get('status'))

    # --- Methods to emit status updates TO the server ---
    def survey_geopath_callback(self, msg):
        """Callback for the published geopath. Relays data to the browser."""
        if not self.sio.connected:
            return
        path_data = []
        for single_wpt in msg.wpt:
            path_data.append({'lat': single_wpt.ll_wpt.latitude, 'lon': single_wpt.ll_wpt.longitude, 'alt': single_wpt.ll_wpt.altitude, 'surge': single_wpt.u})
        self.sio.emit('published_path_update', path_data)

    def roslaunch_state_callback(self, msg):
        if not self.sio.connected:
            return
        statuses = list(msg.data)
        if len(statuses) != len(self.launch_keys):
            self.get_logger().warning(f"Launch status array size ({len(statuses)}) does not match number of launch files ({len(self.launch_keys)}).")
            return
        self.sio.emit('launch_status_update', {'keys': self.launch_keys, 'statuses': statuses})

    def power_callback(self, msg):
        if not self.sio.connected:
            return
        statuses = list(msg.data)
        power_data = {'keys': self.gpio_device_keys,  'statuses': statuses}
        self.sio.emit('power_update', power_data)

    def power_info_callback(self, msg):
        if not self.sio.connected:
            return
        power_info = {'voltage': msg.data[0], 'current': msg.data[1]}
        self.sio.emit('power_info_update', power_info)

    def computer_info_callback(self, msg):
        if not self.sio.connected:
            return
        computer_info = {'mem_usage': msg.data[0], 'cpu_temp': msg.data[1], 'cpu_usage': msg.data[2]}
        self.sio.emit('computer_info_update', computer_info)

    def helm_state_callback(self, msg):
        if not self.sio.connected:
            return
        self.sio.emit('helm_state_update', {'current_state': msg.name, 'transitions': msg.transitions})

    def controller_state_callback(self, msg):
        if not self.sio.connected:
            return
        self.sio.emit('controller_state_update', {'state': msg.data})
    
    def _convert_rad_to_deg(self, angle):
        return angle * 180.0 / np.pi
    
    def _check_for_service(self, client, timeout_sec=1.0):
        if not client.wait_for_service(timeout_sec=timeout_sec): 
            return self.get_logger().error(f"Launch service '{client.srv_name}' not available.")

    # --- NEW: Callback for synchronized pose messages ---
    def synchronized_pose_callback(self, odom_msg, geo_pose_msg):
        """
        Callback for synchronized Odometry and GeoPoseStamped messages.
        This function is called by the ApproximateTimeSynchronizer only when it
        receives a pair of messages that are close in time. It combines them
        into a single data structure and emits it to the web GUI.
        """
        if not self.sio.connected:
            return
        quad = [
            geo_pose_msg.pose.orientation.x,
            geo_pose_msg.pose.orientation.y,
            geo_pose_msg.pose.orientation.z,
            geo_pose_msg.pose.orientation.w,
        ]
        euler_angles = euler_from_quaternion(quad)
        pose_data = {
            "lat": geo_pose_msg.pose.position.latitude,
            "lon": geo_pose_msg.pose.position.longitude,
            "alt": geo_pose_msg.pose.position.altitude,
            "roll": self._convert_rad_to_deg(euler_angles[0]),
            "pitch": self._convert_rad_to_deg(euler_angles[1]),
            "yaw": self._convert_rad_to_deg(euler_angles[2]),
            "frame_id": odom_msg.header.frame_id,
            "child_frame_id": odom_msg.child_frame_id,
            "x": odom_msg.pose.pose.position.x,
            "y": odom_msg.pose.pose.position.y,
            "z": odom_msg.pose.pose.position.z,
            "u": odom_msg.twist.twist.linear.x,
            "v": odom_msg.twist.twist.linear.y,
            "w": odom_msg.twist.twist.linear.z,
            "p": self._convert_rad_to_deg(odom_msg.twist.twist.angular.x),
            "q": self._convert_rad_to_deg(odom_msg.twist.twist.angular.y),
            "r": self._convert_rad_to_deg(odom_msg.twist.twist.angular.z),
        }
        self.sio.emit('vehicle_pose_update', pose_data)

    # --- Methods to call ROS services ---
    def call_launch_file(self, launch_key, status):
        client = self.launch_clients.get(launch_key)
        if client is None: 
            return self.get_logger().error(f"No launch service client for '{launch_key}'.")
        # if not client.wait_for_service(timeout_sec=1.0): 
        #     return self.get_logger().error(f"Launch service '{client.srv_name}' not available.")
        self._check_for_service(client)
        req = SetBool.Request()
        req.data = bool(status)
        self.get_logger().info(f"Calling launch service for '{launch_key}' with status: {status}")
        client.call_async(req)

    def call_set_power(self, name, status):
        client = self.power_clients.get(name)
        if client is None: return self.get_logger().error(f"No power service client for '{name}'.")
        self._check_for_service(client)
        req = SetBool.Request()
        req.data = bool(status)
        client.call_async(req)

    def call_set_helm_state(self, state_name):
        # if not self.set_helm_state_client.wait_for_service(timeout_sec=1.0): 
        #     return self.get_logger().error(f"Service '{self.set_helm_state_client.srv_name}' not available.")
        self._check_for_service(self.set_helm_state_client)
        self.set_helm_state_client.call_async(SetString.Request(data=state_name))

    def call_set_controller_state(self, value):
        # if not self.set_controller_state_client.wait_for_service(timeout_sec=1.0): return self.get_logger().error(f"Service '{self.set_controller_state_client.srv_name}' not available.")
        self._check_for_service(self.set_controller_state_client)
        self.set_controller_state_client.call_async(SetBool.Request(data=bool(value)))

    def call_publish_waypoints(self, waypoints_data):
        # if not self.pub_waypoints_client.wait_for_service(timeout_sec=1.0):
        #     return self.get_logger().error(f"Service '{self.pub_waypoints_client.srv_name}' not available.")
        self._check_for_service(self.pub_waypoints_client)
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
    sio = sio_client_lib.Client(logger=False, engineio_logger=False)
    @sio.event
    def connect():
        print("ROS Interface connected successfully to WebSocket server.")
    @sio.event
    def connect_error(data):
        print(f"Connection failed: {data}")
    @sio.event
    def disconnect():
        print("ROS Interface disconnected from WebSocket server.")

    # --- Connection Retry Loop ---
    server_url = 'http://localhost:5001'
    connected = False
    while not connected:
        try:
            print(f"Attempting to connect to server at {server_url}...")
            sio.connect(server_url)
            connected = True
        except sio_client_lib.exceptions.ConnectionError as e:
            print(f"Connection failed: {e}. Retrying in 5 seconds...")
            time.sleep(5)
    
    ros_node = RosInterfaceNode(sio)
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    # Run the executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # The sio.wait() call will block and handle socket events.
        # This is more efficient than a `while True: time.sleep(1)` loop.
        sio.wait()
    except KeyboardInterrupt:
        print("Keyboard interrupt received.")
    finally:
        print("Shutting down...")
        ros_node.destroy_node()
        executor.shutdown()
        if sio.connected:
            sio.disconnect()
        rclpy.shutdown()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()