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
from sensor_msgs.msg import NavSatFix
from mvp_msgs.msg import Waypoint
from std_srvs.srv import SetBool
from rcl_interfaces.srv import GetParameters
from tf_transformations import euler_from_quaternion

# --- Dynamic Imports and Serialization ---
from rosidl_runtime_py.utilities import get_message, get_service
from rosidl_runtime_py.convert import message_to_ordereddict

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
                ('c2_commander_node', ''),
            ]
        )
        self.topic_ns = self.get_parameter('topic_ns').value
        self.service_ns = self.get_parameter('service_ns').value
        self.c2_commander_node_name = self.get_parameter('c2_commander_node').value
        
        # Load Schemas (as string maps)
        # We declare them dynamically to handle arbitrary dictionaries
        self.declare_parameter('topic_schema.dummy', 'dummy') 
        self.declare_parameter('service_schema.dummy', 'dummy')
        
        # Helper to get dict from params
        self.topic_schema = self._get_param_dict('topic_schema')
        self.service_schema = self._get_param_dict('service_schema')
        
        self.get_logger().info(f"Loaded Topic Schema: {self.topic_schema}")
        self.get_logger().info(f"Loaded Service Schema: {self.service_schema}")

        self.power_clients = {}
        self.launch_clients = {}
        self.launch_keys = []
        self.gpio_device_keys = []
        
        # Storage for dynamically created interfaces
        self.active_subscribers = {}
        self.active_services = {}

        # To track dynamically added subscribers (GPS)
        self.dynamic_subscribers = {}

        self.setup_ros_communications()
        self.setup_sio_handlers()

        # --- Asynchronously fetch parameters from C2 Commander node ---
        if self.c2_commander_node_name:
            self.get_logger().info(f"Will attempt to fetch parameters from '{self.c2_commander_node_name}'.")
            self.param_fetch_timer = self.create_timer(1.0, self.try_fetch_and_setup_dynamic_clients)
        else:
            self.get_logger().error("'c2_commander_node' not specified in config. Cannot setup dynamic clients.")

        # --- Start Dynamic Discovery Timer ---
        # Checks every 2 seconds for new topics matching the schema
        self.discovery_timer = self.create_timer(2.0, self.discover_and_subscribe_topics)
        self.service_discovery_timer = self.create_timer(5.0, self.discover_and_setup_services)

    def _get_param_dict(self, prefix):
        """Helper to extract a dictionary of parameters starting with a prefix."""
        # Note: This is a simplified way to get params. Real implementations might iterate list_parameters
        # But here we rely on the yaml structure loading.
        # rclpy doesn't have a direct 'get_param_map', so we infer from known structure
        # or we iterate via get_parameters_by_prefix if available, or just use list_parameters
        params = {}
        try:
            # We must use list_parameters to find what was loaded from YAML
            prefix_len = len(prefix) + 1
            all_names = self.get_parameters_by_prefix(prefix)
            for key, value in all_names.items():
                if key != 'dummy':
                    params[key] = value.value
        except Exception as e:
            self.get_logger().warn(f"Could not load schema for {prefix}: {e}")
        return params

    def get_topic_abs(self, suffix):
        return self.topic_ns + suffix

    def get_service_abs(self, suffix):
        return self.service_ns + suffix

    def setup_ros_communications(self):
        self.callback_group = ReentrantCallbackGroup()
        
        # --- Synchronized Pose Subscribers (Explicit, Complex Logic) ---
        odom_topic = self.get_topic_abs(self.get_parameter('poses_source').value)
        geopose_topic = self.get_topic_abs(self.get_parameter('geo_pose_source').value)
        
        # We need to find types for these explicit topics or assume defaults
        # For simplicity, we keep explicit imports for the complex sync logic
        odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)
        geopose_sub = message_filters.Subscriber(self, GeoPoseStamped, geopose_topic)
        
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, geopose_sub], queue_size=10, slop=0.2
        )
        self.time_synchronizer.registerCallback(self.synchronized_pose_callback)
        self.get_logger().info(f"Synchronizing pose on odom topic '{odom_topic}' and geopose topic '{geopose_topic}'.")

    # -------------------------------------------------------------------------
    # Dynamic Discovery Logic
    # -------------------------------------------------------------------------

    def create_generic_callback(self, event_key):
        """
        Factory to create a closure for a generic callback.
        event_key: The event name used by the frontend/events.py
        """
        def callback(msg):
            if not self.sio.connected:
                return
            
            # Auto-serialize
            try:
                data_dict = message_to_ordereddict(msg)
                
                # Emit generic standard event
                payload = {'key': event_key, 'data': data_dict}
                self.sio.emit('generic_ros_update', payload)
            except Exception as e:
                self.get_logger().error(f"Error in generic callback for {event_key}: {e}")
        return callback

    def discover_and_subscribe_topics(self):
        """Scans ROS graph for topics matching schema and subscribes."""
        topic_names_and_types = self.get_topic_names_and_types()
        
        # Iterate over our schema definition
        for suffix, event_key in self.topic_schema.items():
            full_topic_name = self.get_topic_abs(suffix)
            
            # Check if we are already subscribed
            if full_topic_name in self.active_subscribers:
                continue

            # Check if topic exists in the graph
            matches = [t for t in topic_names_and_types if t[0] == full_topic_name]
            if matches:
                topic_name, topic_types = matches[0]
                msg_type_str = topic_types[0] # Take the first type
                
                try:
                    # Dynamically import the message class
                    MsgClass = get_message(msg_type_str)
                    
                    # Create Subscription using the standard generic callback
                    # No custom transforms are applied; frontend must parse raw message structure.
                    sub = self.create_subscription(
                        MsgClass,
                        topic_name,
                        self.create_generic_callback(event_key),
                        10,
                        callback_group=self.callback_group
                    )
                    self.active_subscribers[full_topic_name] = sub
                    self.get_logger().info(f"Dynamically subscribed to '{topic_name}' -> Event '{event_key}'")
                
                except Exception as e:
                    self.get_logger().error(f"Failed to subscribe to {topic_name}: {e}")

    def discover_and_setup_services(self):
        """Scans schema for services and creates clients."""
        # This mirrors the topic logic. 
        for suffix, service_type_str in self.service_schema.items():
            full_srv_name = self.get_service_abs(suffix)
            
            if full_srv_name in self.active_services:
                continue
            
            try:
                SrvClass = get_service(service_type_str)
                client = self.create_client(SrvClass, full_srv_name, callback_group=self.callback_group)
                self.active_services[full_srv_name] = {'client': client, 'type': SrvClass}
                self.get_logger().info(f"Dynamically created client for '{full_srv_name}'")
            except Exception as e:
                self.get_logger().error(f"Failed to create service client {full_srv_name}: {e}")

    # -------------------------------------------------------------------------
    # Legacy / Complex Logic (Pose Sync, C2 Commander)
    # -------------------------------------------------------------------------

    def discover_navsatfix_topics(self):
        """Discover all topics of type sensor_msgs/msg/NavSatFix"""
        topic_names_and_types = self.get_topic_names_and_types()
        navsatfix_topics = []
        for name, types in topic_names_and_types:
            if 'sensor_msgs/msg/NavSatFix' in types:
                is_subscribed = name in self.dynamic_subscribers
                navsatfix_topics.append({
                    'name': name,
                    'type': 'sensor_msgs/msg/NavSatFix',
                    'subscribed': is_subscribed
                })
        return navsatfix_topics

    def try_fetch_and_setup_dynamic_clients(self):
        if self.param_fetch_timer: 
            self.param_fetch_timer.cancel()
        if self.dynamic_clients_configured: 
            return

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

            base_power_srv = self.get_service_abs('gpio_manager/set_power/')
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
            
            # Emit keys to frontend so they can map the raw arrays
            if self.sio.connected:
                self.sio.emit('update_launch_keys', {'keys': self.launch_keys})
                self.sio.emit('update_power_keys', {'keys': self.gpio_device_keys})
        except Exception as e:
            self.get_logger().error(f"Failed to process parameter fetch result: {e}. Retrying in 5s...")
            self.param_fetch_timer = self.create_timer(5.0, self.try_fetch_and_setup_dynamic_clients)

    def create_dynamic_gps_subscriber(self, topic_name):
        """Create a new GPS subscriber dynamically"""
        if topic_name in self.dynamic_subscribers:
            self.get_logger().warn(f"Subscriber for {topic_name} already exists")
            return None
        
        def make_callback(topic):
            def callback(msg):
                self.dynamic_gps_callback(msg, topic)
            return callback
        
        subscriber = self.create_subscription(
            NavSatFix, 
            topic_name, 
            make_callback(topic_name), 
            10, 
            callback_group=self.callback_group
        )
        self.dynamic_subscribers[topic_name] = subscriber
        self.get_logger().info(f"Created new GPS subscriber for topic: {topic_name}")
        return subscriber

    def setup_sio_handlers(self):
        @self.sio.on('ros_action')
        def handle_ros_action(data):
            action = data.get('action')
            self.get_logger().info(f"Received forwarded GUI action '{action}'")
            
            if action == 'change_state':
                self.call_service_generic('mvp_helm/change_state', data.get('value'))
            elif action == 'controller_state':
                self.call_service_generic('controller/set', data.get('value'))
            elif action == 'publish_waypoints':
                self.call_publish_waypoints(data.get('waypoints'))
            elif action == 'set_power':
                self.call_set_power(data.get('name'), data.get('status'))
            elif action == 'launch_file':
                self.call_launch_file(data.get('key'), data.get('status'))
        
        @self.sio.on('subscribe_new_gps_topic')
        def handle_subscribe_new_gps_topic(data):
            topic_name = data.get('topic')
            if self.create_dynamic_gps_subscriber(topic_name):
                self.get_logger().info(f"Successfully subscribed to {topic_name}. Notifying client.")
                self.sio.emit('gps_topic_subscribed', {'topic': topic_name})
        
        @self.sio.on('discover_gps_topics')
        def handle_discover_gps_topics(data):
            topics = self.discover_navsatfix_topics()
            self.sio.emit('gps_topics_discovered', {'topics': topics})
        
        @self.sio.on('unsubscribe_gps_topic')
        def handle_unsubscribe_gps_topic(data):
            topic_name = data.get('topic')
            if topic_name in self.dynamic_subscribers:
                subscriber_to_destroy = self.dynamic_subscribers.pop(topic_name)
                self.destroy_subscription(subscriber_to_destroy)
                self.get_logger().info(f"Unsubscribed from and destroyed subscription for GPS topic: {topic_name}")
                self.sio.emit('gps_topic_unsubscribed', {'topic': topic_name})

    def synchronized_pose_callback(self, odom_msg, geo_pose_msg):
        if not self.sio.connected: 
            return
        
        header_timestamp = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec / 1e9
        
        q = geo_pose_msg.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # This is a computed/synthetic event, so we construct the dict manually
        pose_data = {
            "header_timestamp": header_timestamp,
            "lat": geo_pose_msg.pose.position.latitude, "lon": geo_pose_msg.pose.position.longitude, "alt": geo_pose_msg.pose.position.altitude,
            "roll": np.rad2deg(euler[0]), "pitch": np.rad2deg(euler[1]), "yaw": np.rad2deg(euler[2]),
            "x": odom_msg.pose.pose.position.x, "y": odom_msg.pose.pose.position.y, "z": odom_msg.pose.pose.position.z,
            "u": odom_msg.twist.twist.linear.x, "v": odom_msg.twist.twist.linear.y, "w": odom_msg.twist.twist.linear.z,
            "p": np.rad2deg(odom_msg.twist.twist.angular.x), "q": np.rad2deg(odom_msg.twist.twist.angular.y), "r": np.rad2deg(odom_msg.twist.twist.angular.z),
        }
        self.sio.emit('generic_ros_update', {'key': 'vehicle_pose_update', 'data': pose_data})

    def dynamic_gps_callback(self, gps_msg, topic_name):
        if not self.sio.connected: 
            return
        gps_data = {
            "lat": gps_msg.latitude, "lon": gps_msg.longitude, "alt": gps_msg.altitude, 
            'pos_cov': list(gps_msg.position_covariance) 
        }
        self.sio.emit('dynamic_gps_update', {'topic': topic_name, 'data': gps_data})

    def _check_service(self, client, timeout_sec=1.0):
        if not client.wait_for_service(timeout_sec=timeout_sec): 
            self.get_logger().error(f"Service '{client.srv_name}' not available.")
            return False
        return True

    def call_service_generic(self, suffix, value):
        """Finds a dynamically created client for the suffix and calls it."""
        full_name = self.get_service_abs(suffix)
        srv_info = self.active_services.get(full_name)
        
        if not srv_info:
            self.get_logger().error(f"Service client for {suffix} not found.")
            return
            
        client = srv_info['client']
        SrvType = srv_info['type']
        
        if self._check_service(client):
            # We assume a standard single-field request for now or construct based on type
            # This is a simplification. A full generic implementation would require
            # inspecting the SrvType.Request structure and mapping the 'value' input.
            req = SrvType.Request()
            if hasattr(req, 'data'):
                req.data = value
            elif hasattr(req, 'value'): # some custom srvs
                 req.value = value
            
            client.call_async(req)

    def call_launch_file(self, launch_key, status):
        client = self.launch_clients.get(launch_key)
        if client and self._check_service(client):
            self.get_logger().info(f"Calling launch service for '{launch_key}' with status: {status}")
            client.call_async(SetBool.Request(data=bool(status)))

    def call_set_power(self, name, status):
        client = self.power_clients.get(name)
        if client and self._check_service(client):
            client.call_async(SetBool.Request(data=bool(status)))

    def call_publish_waypoints(self, waypoints_data):
        # We handle this specifically because it's a complex type
        suffix = 'mvp_helm/set_waypoints'
        full_name = self.get_service_abs(suffix)
        srv_info = self.active_services.get(full_name)
        
        if srv_info and self._check_service(srv_info['client']):
            req = srv_info['type'].Request()
            req.type = 'waypoint'
            for wp_data in waypoints_data:
                wpt = Waypoint()
                wpt.ll_wpt.latitude = wp_data['lat']
                wpt.ll_wpt.longitude = wp_data['lon']
                wpt.ll_wpt.altitude = wp_data['alt']
                wpt.u = wp_data['surge']
                req.wpt.append(wpt)
            srv_info['client'].call_async(req)

def main(args=None):
    rclpy.init(args=args)
    
    sio = sio_client_lib.Client(logger=True)

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
    while rclpy.ok():
        try:
            print(f"Attempting to connect to {server_url}...")
            sio.connect(server_url)
            break
        except sio_client_lib.exceptions.ConnectionError:
            print("Connection failed. Retrying in 3 seconds...")
            time.sleep(3)
        except Exception as e:
            print(f"An unexpected error occurred during connection: {e}")
            time.sleep(3)

    if not rclpy.ok() or not sio.connected:
        print("Could not connect to WebSocket server or ROS was shut down. Shutting down node.")
        if sio.connected:
            sio.disconnect()
        if rclpy.ok():
            rclpy.shutdown()
        return

    ros_node = RosInterfaceNode(sio)
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
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