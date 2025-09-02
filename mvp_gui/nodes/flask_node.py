import rclpy
from rclpy.node import Node
import threading
from ament_index_python.packages import get_package_share_directory
from mvp_gui.app_factory import create_app

# Global variable to hold the SocketIO server instance
sio_server = None

def run_flask_app(app, sio):
    """Function to run the Flask-SocketIO server."""
    global sio_server
    sio_server = sio
    # use_reloader=False is critical for running in this context
    sio.run(app, host='0.0.0.0', port=5001, debug=False, use_reloader=False, allow_unsafe_werkzeug=True)

class FlaskNode(Node):
    def __init__(self):
        super().__init__('flask_node')
        self.get_logger().info('Flask_node started.')
        
        # Get package share directory to locate templates, static files, and database
        pkg_share_dir = get_package_share_directory('mvp_gui_2')
        
        # --- Get status thresholds from params ---
        self.declare_parameters(
            namespace='status_thresholds',
            parameters=[
                ('roll.yellow', 15.0), ('roll.red', 30.0),
                ('pitch.yellow', 10.0), ('pitch.red', 20.0),
                ('altimeter.yellow', 3.0), ('altimeter.red', 1.0),
                ('voltage.yellow', 11.5), ('voltage.red', 11.0),
                ('current.yellow', 10.0), ('current.red', 15.0),
                ('cpu_temp.yellow', 75.0), ('cpu_temp.red', 85.0),
                ('cpu_usage.yellow', 80.0), ('cpu_usage.red', 95.0),
                ('mem_usage.yellow', 80.0), ('mem_usage.red', 95.0),
            ]
        )
        param_keys = [
            'roll.yellow', 'roll.red', 'pitch.yellow', 'pitch.red', 'altimeter.yellow', 'altimeter.red',
            'voltage.yellow', 'voltage.red', 'current.yellow', 'current.red',
            'cpu_temp.yellow', 'cpu_temp.red', 'cpu_usage.yellow', 'cpu_usage.red',
            'mem_usage.yellow', 'mem_usage.red',
        ]
        params = self.get_parameters([f'status_thresholds.{key}' for key in param_keys])
        
        status_thresholds = {}
        for param in params:
            # param.name is 'status_thresholds.roll.yellow', split it
            key_path = param.name.split('.')[1:] # results in ['roll', 'yellow']
            d = status_thresholds
            for key in key_path[:-1]:
                d = d.setdefault(key, {})
            d[key_path[-1]] = param.value
        
        # Create the Flask app using the factory, passing in the thresholds
        self.app, self.sio = create_app(pkg_share_dir, status_thresholds)
        
        # Run the flask app in a separate thread
        self.flask_thread = threading.Thread(target=run_flask_app, args=(self.app, self.sio), daemon=True)
        self.flask_thread.start()

def main(args=None):
    rclpy.init(args=args)
    flask_node = FlaskNode()
    
    try:
        rclpy.spin(flask_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Node shutdown
        flask_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()