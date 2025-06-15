from flask import render_template, request, jsonify, redirect, url_for, send_from_directory
from mvp_gui import app, db, sio_server, yaml_config
from mvp_gui.models import Waypoint
import os
from werkzeug.exceptions import NotFound


@app.route('/map', methods=['GET', 'POST'])
def map_page():
    # Get host IP from the request to ensure it's correct for the client.
    # This is crucial for the offline map tile URL.
    host_ip = request.host.split(':')[0]
    
    with app.app_context():
        # Fetch only persistent data for initial render
        waypoints = Waypoint.query.order_by(Waypoint.id).all()
        # CORRECTED: Since lat/lon are stored as Float, no conversion is needed.
        # waypoint.alt is still a Decimal from Numeric, so it needs to be converted to float.
        waypoints_data = [{"id": waypoint.id, "lat": waypoint.lat, "lon": waypoint.lon, "alt": float(waypoint.alt)} for waypoint in waypoints]

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

# This route replaces the old '/tiles/<path:filename>' to serve map tiles
# correctly and securely. It expects a URL like /tiles/z/x/y.png
@app.route('/tiles/<int:z>/<int:x>/<int:y>.png')
def serve_tiles(z, x, y):
    """
    Serves map tiles from the offline storage directory specified in config.yaml.
    This function expects a tile structure like: {tiles_dir}/{tile_set_name}/{z}/{x}/{y}.png
    """
    # Use the path from config. The original had two confusing paths, we simplify to one.
    tiles_dir_config = yaml_config.get('tiles_dir_1')
    if not tiles_dir_config:
        app.logger.error("'tiles_dir_1' is not defined in config.yaml")
        raise NotFound()

    # Make path absolute to avoid ambiguity. Assume config path is relative to project root.
    # app.root_path is the 'mvp_gui' folder.
    project_root = os.path.abspath(os.path.join(app.root_path, '..'))
    tiles_base_dir = os.path.join(project_root, tiles_dir_config)

    if not os.path.isdir(tiles_base_dir):
        app.logger.error(f"Offline map base directory not found at: {tiles_base_dir}")
        raise NotFound()

    # The original code iterated through subdirectories, assuming multiple tile sets
    # (e.g., 'satellite', 'streets'). We maintain that logic.
    for tile_set_name in sorted(os.listdir(tiles_base_dir)):
        tile_set_root = os.path.join(tiles_base_dir, tile_set_name)
        if not os.path.isdir(tile_set_root):
            continue
            
        # The directory containing the final image file (e.g., .../15/8404/)
        tile_directory = os.path.join(tile_set_root, str(z), str(x))

        # The filename to be served (e.g., '5443.png')
        tile_filename = f"{y}.png"

        # Check if the specific tile file exists before serving
        if os.path.exists(os.path.join(tile_directory, tile_filename)):
            # send_from_directory is safe and handles headers correctly.
            return send_from_directory(tile_directory, tile_filename)

    # If the tile wasn't found in any tile set, return a 404.
    # This is standard behavior for tile servers and map libraries handle it gracefully.
    raise NotFound()


@app.route('/map_page_assets/<path:filename>')
def serve_map_page_assets(filename):
    """
    Serves CSS and JS files specifically for the map page from the templates/map_page directory.
    This is a non-standard approach to fulfill the user request.
    """
    assets_dir = os.path.join(app.root_path, 'templates', 'map_page')
    return send_from_directory(assets_dir, filename)