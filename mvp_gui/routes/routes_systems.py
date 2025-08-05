# mvp_gui/routes/routes_systems.py
from flask import render_template, jsonify, Blueprint, current_app

systems_bp = Blueprint('systems_bp', __name__)

@systems_bp.route('/', methods=['GET'])
def systems_page():
    # Get launch files from the app config cache, populated by the ROS node
    launch_files = current_app.config.get('_launch_keys_cache', [])
    return render_template("systems.html", 
                           launch_files=launch_files, 
                           current_page="systems")