from mvp_gui import app, sio_server, db
import threading
import time
import sys
import os

def find_ros_workspace(start_path):
    """
    Traverses up from start_path to find a ROS2 workspace root.
    A directory is considered a workspace root if it contains an 'install/setup.bash' file.
    """
    current_path = os.path.abspath(start_path)
    # Loop until we hit the root directory
    while os.path.dirname(current_path) != current_path:
        install_setup_path = os.path.join(current_path, 'install', 'setup.bash')
        if os.path.exists(install_setup_path):
            print(f"Found ROS2 workspace at: {current_path}")
            return current_path
        current_path = os.path.dirname(current_path)
    
    # Check the final root directory as well
    install_setup_path = os.path.join(current_path, 'install', 'setup.bash')
    if os.path.exists(install_setup_path):
        print(f"Found ROS2 workspace at: {current_path}")
        return current_path

    print("WARNING: ROS2 workspace root not found.")
    return None

def setup_app_config(app_instance):
    """
    Detects the virtual environment and ROS workspace, saving paths to the config.
    """
    # Find the python executable of the current environment
    python_executable = sys.executable
    # The 'activate' script is typically in the same directory as the python binary
    venv_bin_dir = os.path.dirname(python_executable)
    activate_script_path = os.path.join(venv_bin_dir, 'activate')
    
    # Check if we are in a virtual environment by looking for the 'activate' script
    if 'VIRTUAL_ENV' in os.environ and os.path.exists(activate_script_path):
        app_instance.config['VENV_ACTIVATE_PATH'] = activate_script_path
        print(f"Found venv activation script: {activate_script_path}")
    else:
        app_instance.config['VENV_ACTIVATE_PATH'] = None
        print("INFO: Not running in a detectable virtual environment.")
    
    # Find ROS workspace starting from the directory of this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_path = find_ros_workspace(script_dir)
    app_instance.config['ROS_WORKSPACE_PATH'] = workspace_path


def main():
    """Main entry point for the application."""
    print("Starting server...")
    
    # Set up dynamic configuration before running the app
    with app.app_context():
        setup_app_config(app)
    
    # Run the Flask-SocketIO server
    # use_reloader=False is important to prevent running startup code twice
    sio_server.run(app, host='0.0.0.0', port=5001, debug=True, use_reloader=False)

if __name__ == "__main__":
    main()