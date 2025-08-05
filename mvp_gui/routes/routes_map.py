# mvp_gui/routes/routes_map.py
from flask import (render_template, request, jsonify, redirect, url_for, 
                   send_from_directory, Response, Blueprint, current_app)
from ..web_utils import db, sio_server
from ..models import Waypoint
import os
import sqlite3
from werkzeug.exceptions import NotFound
from ament_index_python.packages import get_package_share_directory

map_bp = Blueprint('map_bp', __name__)

@map_bp.route('/map', methods=['GET', 'POST'])
def map_page():
    with current_app.app_context():
        waypoints = Waypoint.query.order_by(Waypoint.id).all()
        waypoints_data = [{"id": w.id, "lat": w.lat, "lon": w.lon, "alt": w.alt, "surge": w.surge} for w in waypoints]

    if request.method == 'POST':
        if 'states' in request.form:
            sio_server.emit('ros_action', {'action': 'change_state', 'value': request.form.get('states')})
        elif 'controller_disable' in request.form:
            sio_server.emit('ros_action', {'action': 'controller_state', 'value': False})
        elif 'controller_enable' in request.form:
            sio_server.emit('ros_action', {'action': 'controller_state', 'value': True})
        elif 'publish_waypoints' in request.form:
            sio_server.emit('publish_waypoints_request')
        return redirect(url_for('map_bp.map_page'))

    return render_template("map.html", 
                           items_jsn=waypoints_data, 
                           vehicle_jsn={"lat": 0, "lon": 0, "yaw": 0, "alt": 0}, 
                           current_page="map")

@map_bp.route('/waypoint_drag', methods=['POST'])
def waypoint_drag():
    data = request.json
    waypoint = Waypoint.query.get(data['id'])
    if waypoint:
        waypoint.lon = data['lng']
        waypoint.lat = data['lat']
        waypoint.alt = data['alt']
        waypoint.surge = data['surge']
        db.session.commit()
    return jsonify({"success": True})

@map_bp.route('/tiles/<int:z>/<int:x>/<int:y>.png')
def serve_tiles(z, x, y):
    pkg_share_dir = get_package_share_directory('mvp_gui_2')
    # This path is now relative to the package share directory
    tiles_dir = os.path.join(pkg_share_dir, 'mvp_gui_offline_map')

    if not os.path.isdir(tiles_dir):
        current_app.logger.error(f"Offline map base directory not found at: {tiles_dir}")
        raise NotFound()

    mbtiles_files = [os.path.join(tiles_dir, f) for f in sorted(os.listdir(tiles_dir)) if f.endswith(".mbtiles")]
    if not mbtiles_files:
        current_app.logger.error(f"No .mbtiles files found in directory: {tiles_dir}")
        raise NotFound()

    y_flipped = (2**z) - 1 - y

    for mbtiles_file_path in mbtiles_files:
        tile_data = None
        try:
            con = sqlite3.connect(f"file:{mbtiles_file_path}?mode=ro", uri=True)
            cur = con.cursor()
            cur.execute("SELECT tile_data FROM tiles WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?", (z, x, y_flipped))
            result = cur.fetchone()
            if result: tile_data = result[0]
            con.close()
        except sqlite3.Error as e:
            current_app.logger.error(f"Database error while accessing {mbtiles_file_path}: {e}")
            continue

        if tile_data:
            return Response(tile_data, mimetype='image/png')

    current_app.logger.warning(f"Tile not found for z={z}, x={x}, y={y} in {tiles_dir}")
    raise NotFound()