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
        
        # Create the Flask app using the factory
        self.app, self.sio = create_app(pkg_share_dir)
        
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