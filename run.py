from mvp_gui import app, sio_server, db
import threading
import time
import sys
import os

def setup_app_config(app_instance):
    """
    Detects the virtual environment's activation script and saves it to the config.
    This ensures that subprocesses can activate the correct environment.
    """
    # Find the python executable of the current environment
    python_executable = sys.executable
    # The 'activate' script is typically in the same directory as the python binary
    venv_bin_dir = os.path.dirname(python_executable)
    activate_script_path = os.path.join(venv_bin_dir, 'activate')
    
    if os.path.exists(activate_script_path):
        app_instance.config['VENV_ACTIVATE_PATH'] = activate_script_path
        print(f"Found venv activation script: {activate_script_path}")
    else:
        app_instance.config['VENV_ACTIVATE_PATH'] = None
        print("WARNING: Could not find the virtual environment's 'activate' script.")

def main():
    """Main entry point for the application."""
    print("Starting server...")
    
    # Set up dynamic configuration before running the app
    setup_app_config(app)
    
    # Run the Flask-SocketIO server
    # use_reloader=False is important to prevent running startup code twice
    sio_server.run(app, host='0.0.0.0', port=5000, debug=True, use_reloader=False)

if __name__ == "__main__":
    main()