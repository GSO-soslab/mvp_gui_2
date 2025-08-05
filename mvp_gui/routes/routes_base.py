from flask import render_template, jsonify
from mvp_gui import app, db
from mvp_gui.models import Waypoint
import xml.etree.ElementTree as ET


@app.context_processor
def inject_load():
    """
    Injects data needed for the initial page load.
    Real-time data is now sent via WebSockets, so this context processor
    should only load static or persistent data.
    """
    with app.app_context():
        # These are persistent and good to load initially
        waypoints = Waypoint.query.order_by(Waypoint.id).all()
        
        # The rest of the data (vitals, poses, states) is now real-time.
        # The frontend will be populated by WebSockets.
        # We pass empty placeholders or initial values.
        initial_data = {
            'vitals': {'voltage': 0, 'current': 0},
            'computer_info': {'cpu_temp': 0, 'cpu_usage': 0, 'mem_usage': 0},
            'poses': {'lat': 0, 'lon': 0, 'yaw': 0, 'z': 0},
            'items': [], # Power items are now real-time
            'waypoints': waypoints,
            'states': [], # Helm states are real-time
            'controller_state': {'state': 'Unknown'}
        }
        return initial_data


@app.route("/vehicle_status", methods=['GET', 'POST'])
def vehicle_status_page():
    # The data is now provided by WebSockets. We just render the template.
    return render_template("vehicle_status.html", current_page='vehicle_status')


# This endpoint is no longer the primary way to get data, as it's pushed via WebSockets.
# It can be kept for debugging or for components not yet converted to WebSockets.
@app.route('/path/to/api/endpoint')
def get_latest_yaw():
    # This logic should be moved to the ROS node, which then pushes data.
    # Returning mock data for now.
    orientation_data = {'roll': 0, 'pitch': 0, 'yaw': 0}
    orientation_setpoint = {'roll': 0, 'pitch': 0, 'yaw': 0}
    return jsonify({"orientation_data": orientation_data, "orientation_setpoint": orientation_setpoint})


# This endpoint is deprecated and will be removed.
# Real-time data is now sent via the 'vehicle_pose_update' WebSocket event.
@app.route('/vehicle_status/states')
def home_state_data():
    return jsonify({"message": "This endpoint is deprecated. Use WebSocket for real-time data."}), 410