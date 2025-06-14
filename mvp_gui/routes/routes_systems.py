from flask import request, redirect, url_for, render_template, jsonify, current_app
from mvp_gui import app, sio_server, env
# Import the new manager module instead of the old class
import mvp_gui.ros_interface_manager as ros_interface_manager
import time
import yaml
import os

# There is no longer a class to instantiate. We will call the module functions directly.

def get_launch_files_from_config():
    """Loads YAML configs and extracts the list of launch files."""
    try:
        config_path = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'config.yaml')
        with open(config_path, 'r') as file:
            main_config = yaml.safe_load(file)

        c2_config_path_str = main_config.get('c2_config_yaml')
        c2_config_path = os.path.expanduser(c2_config_path_str)
        with open(c2_config_path, 'r') as file:
            c2_config = yaml.safe_load(file)

        c2_params = c2_config.get('/alpha_rise/mvp_c2_commander', {}).get('ros__parameters', {})
        launch_packages = c2_params.get('launch_packages', [])
        launch_files_list = c2_params.get('launch_files', [])
        
        launch_keys = [f"{pkg}/{file}" for pkg, file in zip(launch_packages, launch_files_list)]
        return launch_keys
    except Exception as e:
        print(f"Error loading launch files from YAML: {e}")
        return []

@app.route('/', methods=['GET', 'POST'])
def systems_page():
    if request.method == 'POST':
        if 'mvpgui_start' in request.form:
            # Get the dynamically detected venv path from the app config
            venv_path = current_app.config.get('VENV_ACTIVATE_PATH')
            python_exec_path = None
            if venv_path:
                # Construct the full path to the Python executable within the venv
                python_exec_path = os.path.join(os.path.dirname(venv_path), 'python3')

            # Pass both paths to the start function
            ros_interface_manager.start_mvp_gui_node_process(env, venv_path, python_exec_path)
            time.sleep(2)
        elif 'mvpgui_stop' in request.form:
            # Call the new stop function from the manager module
            ros_interface_manager.stop_mvp_gui_node_process(env)
        return redirect(url_for('systems_page'))

    # Call the new status function from the manager module
    mvpgui_status = ros_interface_manager.is_running()
    launch_files = get_launch_files_from_config()

    return render_template(
        "systems.html", 
        mvpgui_status=str(mvpgui_status),
        launch_files=launch_files,
        current_page="systems"
    )

# The 'ros_action' handler has been moved to mvp_gui/events.py
# to centralize all socket event handling.

@app.route('/current_system_status')
def current_status():
    """API endpoint for current system status."""
    return jsonify({
        # Call the new status function from the manager module
        "mvpgui_status": {"data": ros_interface_manager.is_running()},
    })