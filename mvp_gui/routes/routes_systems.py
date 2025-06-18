from flask import request, redirect, url_for, render_template, jsonify, current_app
from mvp_gui import app, sio_server, env
# Import the new manager module instead of the old class
import mvp_gui.ros_interface_manager as ros_interface_manager
import time
import yaml
import os
import sys

# There is no longer a class to instantiate. We will call the module functions directly.

@app.route('/', methods=['GET', 'POST'])
def systems_page():
    if request.method == 'POST':
        # This route now handles AJAX requests from the start/stop buttons.
        # It performs the action and returns a JSON response.
        # The client-side JavaScript will handle the page refresh.
        if 'mvpgui_start' in request.form:
            # Get the dynamically detected paths from the app config
            venv_path = current_app.config.get('VENV_ACTIVATE_PATH')
            ros_workspace_path = current_app.config.get('ROS_WORKSPACE_PATH')
            
            if venv_path:
                # Construct the full path to the Python executable within the venv
                python_exec_path = os.path.join(os.path.dirname(venv_path), 'python3')
            else:
                # If no venv, use the same python that is running the Flask app
                python_exec_path = sys.executable

            # Pass all paths to the start function
            ros_interface_manager.start_mvp_gui_node_process(env, venv_path, python_exec_path, ros_workspace_path)
            time.sleep(2)
        elif 'mvpgui_stop' in request.form:
            # Call the new stop function from the manager module
            ros_interface_manager.stop_mvp_gui_node_process(env)
        
        # After performing the action, return a JSON response for the AJAX call.
        return jsonify({'status': 'ok', 'message': 'Action processed.'})

    # For GET requests, render the page as usual.
    mvpgui_status = ros_interface_manager.is_running()
    # Get launch files from the app config cache, populated by the ROS node
    launch_files = current_app.config.get('_launch_keys_cache', [])

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