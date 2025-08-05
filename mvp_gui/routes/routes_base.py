# mvp_gui/routes/routes_base.py
from flask import render_template, jsonify, Blueprint, current_app
from ..web_utils import db
from ..models import Waypoint

base_bp = Blueprint('base_bp', __name__)

@base_bp.context_processor
def inject_load():
    with current_app.app_context():
        waypoints = Waypoint.query.order_by(Waypoint.id).all()
        initial_data = {
            'vitals': {'voltage': 0, 'current': 0},
            'computer_info': {'cpu_temp': 0, 'cpu_usage': 0, 'mem_usage': 0},
            'poses': {'lat': 0, 'lon': 0, 'yaw': 0, 'z': 0},
            'items': [],
            'waypoints': waypoints,
            'states': [],
            'controller_state': {'state': 'Unknown'}
        }
        return initial_data

@base_bp.route("/vehicle_status", methods=['GET', 'POST'])
def vehicle_status_page():
    return render_template("vehicle_status.html", current_page='vehicle_status')