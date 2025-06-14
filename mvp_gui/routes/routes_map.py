from flask import render_template, request, jsonify, redirect, url_for, send_from_directory
from mvp_gui import app, db, sio_server, yaml_config
from mvp_gui.models import Waypoint, CurrentWaypoints
import os

@app.route('/map', methods=['GET', 'POST'])
def map_page():
    host_ip = app.config['HOST_IP']
    
    with app.app_context():
        # Fetch only persistent data for initial render
        waypoints = Waypoint.query.order_by(Waypoint.id).all()
        waypoints_data = [{"id": waypoint.id, "lat": float(waypoint.lat), "lon": float(waypoint.lon), "alt": float(waypoint.alt)} for waypoint in waypoints]

        cwaypoints = CurrentWaypoints.query.all()
        current_waypoints_data = [{"id": cwaypoint.id, "lat": float(cwaypoint.lat), "lon": float(cwaypoint.lon), "alt": float(cwaypoint.alt)} for cwaypoint in cwaypoints]

    # This POST handler is for the legacy form-based buttons.
    # Modern interaction is handled via direct WebSocket events from the client.
    if request.method == 'POST':
        if 'states' in request.form:
            sio_server.emit('ros_action', {'action': 'change_state', 'value': request.form.get('states')})
        elif 'controller_disable' in request.form:
            sio_server.emit('ros_action', {'action': 'controller_state', 'value': False})
        elif 'controller_enable' in request.form:
            sio_server.emit('ros_action', {'action': 'controller_state', 'value': True})
        elif 'publish_waypoints' in request.form:
            # This branch is now superseded by the socketio handler, but is kept for compatibility.
            sio_server.emit('publish_waypoints_request')
        return redirect(url_for('map_page'))

    return render_template("map.html", 
                           items_jsn=waypoints_data, 
                           citems_jsn=current_waypoints_data, 
                           # Pass empty or default data for real-time elements
                           vehicle_jsn={"lat": 0, "lon": 0, "yaw": 0, "alt": 0}, 
                           host_ip=host_ip, 
                           pose_jsn=[], # History is now managed on the client
                           topside_jsn={"lat": 0, "lon": 0, "alt": 0},
                           topsidehistory_jsn=[],
                           secondary_jsn={"lat": 0, "lon": 0, "alt": 0},
                           secondaryhistory_jsn=[],
                           current_page="map")


# The 'publish_waypoints_request' handler has been moved to mvp_gui/events.py


@app.route('/waypoint_drag', methods=['POST'])
def waypoint_drag():
    data = request.json
    waypoint = Waypoint.query.get(data['id'])
    if waypoint:
        waypoint.lon = data['lng']
        waypoint.lat = data['lat']
        waypoint.alt = data['alt']
        db.session.commit()
    return jsonify({"success": True})

@app.route('/tiles/<path:filename>')
def serve_tiles(filename):
    TILES_DIR_1 = yaml_config.get('tiles_dir_1', 'path/to/tiles1')
    TILES_DIR_2 = yaml_config.get('tiles_dir_2', 'path/to/tiles2')
    if os.path.exists(TILES_DIR_1):
        for tile_dir in sorted(os.listdir(TILES_DIR_1)):
            loc_dir_1 = os.path.join(TILES_DIR_1, tile_dir)
            loc_dir_2 = os.path.join(TILES_DIR_2, tile_dir)
            full_path = os.path.join(loc_dir_1, filename)
            if os.path.exists(full_path):
                return send_from_directory(loc_dir_2, filename)    
    return 'No Tile Available'