from flask import render_template, request, jsonify, redirect, url_for, send_from_directory, Response
from mvp_gui import app, db, sio_server, yaml_config
from mvp_gui.models import Waypoint
import os
import sqlite3
from werkzeug.exceptions import NotFound


@app.route('/map', methods=['GET', 'POST'])
def map_page():
    # The host IP is no longer needed; the template will use 
    # the request context to build full URLs.
    
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

# This route is modified to serve .png raster tiles from an .mbtiles file.
# It now expects a URL like /tiles/z/x/y.png
@app.route('/tiles/<int:z>/<int:x>/<int:y>.png')
def serve_tiles(z, x, y):
    """
    Serves map tiles (.png raster format) from an .mbtiles file found within the
    offline storage directory specified in config.yaml.
    """
    # Use the path from config.
    tiles_dir_config = yaml_config.get('tiles_dir_1')
    if not tiles_dir_config:
        app.logger.error("'tiles_dir_1' is not defined in config.yaml")
        raise NotFound()

    # Make path absolute. app.root_path is the 'mvp_gui' folder.
    project_root = os.path.abspath(os.path.join(app.root_path, '..'))
    tiles_base_dir = os.path.join(project_root, tiles_dir_config)

    if not os.path.isdir(tiles_base_dir):
        app.logger.error(f"Offline map base directory not found at: {tiles_base_dir}")
        raise NotFound()

    # Find the first .mbtiles file in the directory.
    mbtiles_file_path = None
    try:
        for filename in sorted(os.listdir(tiles_base_dir)):
            if filename.endswith(".mbtiles"):
                mbtiles_file_path = os.path.join(tiles_base_dir, filename)
                break  # Use the first one found
    except FileNotFoundError:
        app.logger.error(f"Offline map base directory could not be read: {tiles_base_dir}")
        raise NotFound()

    if not mbtiles_file_path:
        app.logger.error(f"No .mbtiles file found in directory: {tiles_base_dir}")
        raise NotFound()

    # MBTiles use TMS tile scheme (flipped Y). XYZ scheme used by map clients needs conversion.
    y_flipped = (2**z) - 1 - y

    tile_data = None
    try:
        # Connect to the SQLite database (.mbtiles file) in read-only mode.
        con = sqlite3.connect(f"file:{mbtiles_file_path}?mode=ro", uri=True)
        cur = con.cursor()
        cur.execute(
            "SELECT tile_data FROM tiles WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?",
            (z, x, y_flipped)
        )
        result = cur.fetchone()
        if result:
            tile_data = result[0]
        con.close()
    except sqlite3.Error as e:
        app.logger.error(f"Database error for {mbtiles_file_path}: {e}")
        raise NotFound()

    if tile_data:
        # Return the tile data with the correct content type for a PNG image.
        return Response(tile_data, mimetype='image/png')
    else:
        # Tile not found in the database for the given z, x, y.
        # Map libraries handle 404s gracefully.
        app.logger.warning(f"Tile not found for z={z}, x={x}, y={y} in {mbtiles_file_path}")
        raise NotFound()


@app.route('/fonts/<fontstack>/<font_range>.pbf')
def serve_fonts(fontstack, font_range):
    """
    Serves font glyphs (.pbf format) for the offline map.
    This is used for vector tiles and is unlikely to be called for raster maps.
    """
    tiles_dir_config = yaml_config.get('tiles_dir_1')
    if not tiles_dir_config:
        app.logger.error("'tiles_dir_1' is not defined in config.yaml")
        raise NotFound()

    project_root = os.path.abspath(os.path.join(app.root_path, '..'))
    fonts_base_dir = os.path.join(project_root, tiles_dir_config, 'fonts')
    font_stack_dir = os.path.join(fonts_base_dir, fontstack)
    font_filename = f"{font_range}.pbf"

    # Security check: ensure fontstack doesn't contain '..' to prevent path traversal
    if '..' in fontstack or not os.path.isdir(font_stack_dir):
        app.logger.warning(f"Font stack not found or invalid path: {font_stack_dir}")
        raise NotFound()

    if os.path.exists(os.path.join(font_stack_dir, font_filename)):
        return send_from_directory(font_stack_dir, font_filename, mimetype='application/x-protobuf')
        
    raise NotFound()


@app.route('/map_page_assets/<path:filename>')
def serve_map_page_assets(filename):
    """
    Serves CSS and JS files specifically for the map page from the templates/map_page directory.
    This is a non-standard approach to fulfill the user request.
    """
    assets_dir = os.path.join(app.root_path, 'templates', 'map_page')
    return send_from_directory(assets_dir, filename)