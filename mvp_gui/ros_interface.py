import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import socketio as sio_client_lib # Use the full library name for clarity
import time
import threading
import numpy as np
import yaml
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoseStamped
from mvp_msgs.msg import Power, Waypoint, ControlProcess, HelmState
from std_msgs.msg import Float64, Float32MultiArray, Int16, Bool, Int16MultiArray
from tf_transformations import euler_from_quaternion
from mvp_msgs.srv import SetString, SendWaypoints
from std_srvs.srv import SetBool, Trigger
import os

class RosInterfaceNode(Node):
    def __init__(self, sio_client):
        super().__init__('mvp_gui_interface_node')
        self.sio = sio_client

        # --- Load Main Configuration ---
        config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'config.yaml')
        with open(config_path, 'r') as file:
            self.yaml_config = yaml.safe_load(file)
        # --- Load Secondary C2 Configuration for specific device/launch names ---
        self.c2_config = {}
        c2_config_path_str = self.yaml_config.get('c2_config_yaml')
        if c2_config_path_str:
            c2_config_path = os.path.expanduser(c2_config_path_str)
            try:
                with open(c2_config_path, 'r') as file:
                    self.c2_config = yaml.safe_load(file)
                self.get_logger().info(f"Successfully loaded C2 config from: {c2_config_path}")
            except FileNotFoundError:
                self.get_logger().error(f"C2 config file not found at: {c2_config_path}")
        
        self.get_logger().info('ROS Interface Node started. Setting up communications...')
        self.power_clients = {}
        self.launch_clients = {}
        self.launch_keys = [] # Store the ordered list of launch keys
        self.setup_ros_communications()
        self.setup_sio_handlers()
        
        # Store last known data to combine messages
        self.last_odom = None
        self.last_geo_pose = None

    def setup_ros_communications(self):
        self.callback_group = ReentrantCallbackGroup()
        
        self.topic_ns = self.yaml_config.get('topic_ns', '/default_ns/')
        self.service_ns = self.yaml_config.get('service_ns', '/default_ns/')
        
        # --- Subscribers ---
        self.create_subscription(Odometry, self.topic_ns + self.yaml_config.get('poses_source', 'odometry'), self.pose_callback, 10, callback_group=self.callback_group)
        self.create_subscription(GeoPoseStamped, self.topic_ns + self.yaml_config.get('geo_pose_source', 'geopose'), self.geo_pose_callback, 10, callback_group=self.callback_group)
        self.create_subscription(HelmState, self.topic_ns + self.yaml_config.get('helm_state_get', 'helm/state'), self.helm_state_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Bool, self.topic_ns + self.yaml_config.get('controller_state_get', 'controller_state'), self.controller_state_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Power, self.topic_ns + self.yaml_config.get('gpio_power_get', 'gpio_power_state'), self.power_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Int16MultiArray, self.topic_ns + self.yaml_config.get('roslaunch_state_get', 'roslaunch_state'), self.roslaunch_state_callback, 10, callback_group=self.callback_group)

        # --- Static Service Clients ---
        self.set_helm_state_client = self.create_client(SetString, self.service_ns + self.yaml_config.get('helm_state_set', 'mvp_helm/change_state'), callback_group=self.callback_group)
        self.set_controller_state_client = self.create_client(SetBool, self.service_ns + self.yaml_config.get('controller_state_set', 'controller/set'), callback_group=self.callback_group)
        self.pub_waypoints_client = self.create_client(SendWaypoints, self.service_ns + self.yaml_config.get('pub_waypoints_service', 'mvp_helm/set_waypoints'), callback_group=self.callback_group)
        
        # --- Dynamically Create Service Clients from C2 Config ---
        try:
            c2_params = self.c2_config.get('/alpha_rise/mvp_c2_commander', {}).get('ros__parameters', {})
            
            gpio_devices = c2_params.get('gpio_devices', [])
            base_power_srv = self.service_ns + self.yaml_config.get('set_power_service', 'gpio_manager/set_power/')
            for device_name in gpio_devices:
                full_service_name = f"{base_power_srv}{device_name}"
                self.power_clients[device_name] = self.create_client(SetBool, full_service_name, callback_group=self.callback_group)

            launch_packages = c2_params.get('launch_packages', [])
            launch_files = c2_params.get('launch_files', [])
            for package, filename in zip(launch_packages, launch_files):
                launch_key = f"{package}/{filename}"
                self.launch_keys.append(launch_key)
                full_service_name = f"{self.service_ns}roslaunch/{launch_key}"
                self.launch_clients[launch_key] = self.create_client(SetBool, full_service_name, callback_group=self.callback_group)
            self.get_logger().info(f"Dynamically created {len(self.power_clients)} power clients and {len(self.launch_clients)} launch clients.")
        except Exception as e:
            self.get_logger().error(f"Failed to parse C2 config for dynamic services: {e}")

    def setup_sio_handlers(self):
        # This handler listens for commands relayed from the browser
        @self.sio.on('ros_action')
        def handle_ros_action(data):
            action = data.get('action')
            self.get_logger().info(f"Received forwarded GUI action '{action}', data: {data}")
            if action == 'change_state': self.call_set_helm_state(data.get('value'))
            elif action == 'controller_state': self.call_set_controller_state(data.get('value'))
            elif action == 'publish_waypoints': self.call_publish_waypoints(data.get('waypoints'))
            elif action == 'set_power': self.call_set_power(data.get('name'), data.get('status'))
            elif action == 'launch_file': self.call_launch_file(data.get('key'), data.get('status'))

    # --- Methods to emit status updates TO the server ---
    def roslaunch_state_callback(self, msg):
        if not self.sio.connected: return
        statuses = list(msg.data)
        if len(statuses) != len(self.launch_keys):
            self.get_logger().warning(f"Launch status array size ({len(statuses)}) does not match number of launch files ({len(self.launch_keys)}).")
            return
        self.sio.emit('launch_status_update', {'keys': self.launch_keys, 'statuses': statuses})

    def power_callback(self, msg):
        if not self.sio.connected: return
        power_data = { 'voltage': getattr(msg, 'voltage', 0.0), 'current': getattr(msg, 'current', 0.0), 'channels': getattr(msg, 'channel_states', []), 'channel_names': getattr(msg, 'channel_names', []) }
        self.sio.emit('power_update', power_data)

    def helm_state_callback(self, msg):
        if not self.sio.connected: return
        self.sio.emit('helm_state_update', {'current_state': msg.name, 'transitions': msg.transitions})

    def controller_state_callback(self, msg):
        if not self.sio.connected: return
        self.sio.emit('controller_state_update', {'state': msg.data})

    def publish_combined_pose(self):
        if not self.sio.connected: return
        if not self.last_odom or not self.last_geo_pose: return
        time_diff = abs(self.last_odom.header.stamp.sec - self.last_geo_pose.header.stamp.sec)
        if time_diff > 1.0: return
        quad = [self.last_geo_pose.pose.orientation.x, self.last_geo_pose.pose.orientation.y, self.last_geo_pose.pose.orientation.z, self.last_geo_pose.pose.orientation.w]
        euler_angles = euler_from_quaternion(quad)
        pose_data = {
            "lat": self.last_geo_pose.pose.position.latitude, "lon": self.last_geo_pose.pose.position.longitude, "alt": self.last_geo_pose.pose.position.altitude,
            "roll": euler_angles[0] * 180 / np.pi, "pitch": euler_angles[1] * 180 / np.pi, "yaw": euler_angles[2] * 180 / np.pi,
            "frame_id": self.last_odom.header.frame_id, "child_frame_id": self.last_odom.child_frame_id,
            "x": self.last_odom.pose.pose.position.x, "y": self.last_odom.pose.pose.position.y, "z": self.last_odom.pose.pose.position.z,
            "u": self.last_odom.twist.twist.linear.x, "v": self.last_odom.twist.twist.linear.y, "w": self.last_odom.twist.twist.linear.z,
            "p": self.last_odom.twist.twist.angular.x * 180 / np.pi, "q": self.last_odom.twist.twist.angular.y * 180 / np.pi, "r": self.last_odom.twist.twist.angular.z * 180 / np.pi,
        }
        self.sio.emit('vehicle_pose_update', pose_data)
        self.last_odom, self.last_geo_pose = None, None

    # --- Callbacks for ROS messages ---
    def pose_callback(self, msg):
        self.last_odom = msg
        self.publish_combined_pose()

    def geo_pose_callback(self, msg):
        self.last_geo_pose = msg
        self.publish_combined_pose()

    # --- Methods to call ROS services ---
    def call_launch_file(self, launch_key, status):
        client = self.launch_clients.get(launch_key)
        if client is None: return self.get_logger().error(f"No launch service client for '{launch_key}'.")
        if not client.wait_for_service(timeout_sec=1.0): return self.get_logger().error(f"Launch service '{client.srv_name}' not available.")
        req = SetBool.Request()
        req.data = bool(status)
        self.get_logger().info(f"Calling launch service for '{launch_key}' with status: {status}")
        client.call_async(req)

    def call_set_power(self, name, status):
        client = self.power_clients.get(name)
        if client is None: return self.get_logger().error(f"No power service client for '{name}'.")
        if not client.wait_for_service(timeout_sec=1.0): return self.get_logger().error(f"Service '{client.srv_name}' not available.")
        req = SetBool.Request()
        req.data = bool(status)
        client.call_async(req)

    def call_set_helm_state(self, state_name):
        if not self.set_helm_state_client.wait_for_service(timeout_sec=1.0): return self.get_logger().error(f"Service '{self.set_helm_state_client.srv_name}' not available.")
        self.set_helm_state_client.call_async(SetString.Request(data=state_name))

    def call_set_controller_state(self, value):
        if not self.set_controller_state_client.wait_for_service(timeout_sec=1.0): return self.get_logger().error(f"Service '{self.set_controller_state_client.srv_name}' not available.")
        self.set_controller_state_client.call_async(SetBool.Request(data=bool(value)))

    def call_publish_waypoints(self, waypoints_data):
        if not self.pub_waypoints_client.wait_for_service(timeout_sec=1.0): return self.get_logger().error(f"Service '{self.pub_waypoints_client.srv_name}' not available.")
        req = SendWaypoints.Request(type='geopath')
        for wp_data in waypoints_data:
            wpt = Waypoint()
            wpt.ll_wpt.latitude, wpt.ll_wpt.longitude, wpt.ll_wpt.altitude = float(wp_data['lat']), float(wp_data['lon']), float(wp_data['alt'])
            req.wpt.append(wpt)
        self.pub_waypoints_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    sio = sio_client_lib.Client(logger=True, engineio_logger=True)
    
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
    server_url = 'http://localhost:5000'
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