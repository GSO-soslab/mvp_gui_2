from flask import Flask, current_app
from .web_utils import db, sio_server
import os
import json

def create_app(pkg_share_dir, status_thresholds=None):
    """
    Creates and configures the Flask application and its extensions.
    """
    template_folder = os.path.join(pkg_share_dir, 'templates')
    static_folder = os.path.join(pkg_share_dir, 'static')

    # Ensure the instance folder exists for the database
    instance_path = os.path.join(pkg_share_dir, 'instance')
    os.makedirs(instance_path, exist_ok=True)
    
    app = Flask(
        __name__,
        instance_path=instance_path,
        template_folder=template_folder,
        static_folder=static_folder,
        instance_relative_config=True # Good practice
    )

    # --- Configuration ---
    app.config['SECRET_KEY'] = 'abcd1234'
    # Place the SQLite database in the instance folder within the package share directory
    db_path = os.path.join(app.instance_path, 'mvp_gui.db')
    app.config['SQLALCHEMY_DATABASE_URI'] = f'sqlite:///{db_path}'
    app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
    
    # A cache for dynamic data from the ROS node
    app.config['_launch_keys_cache'] = []
    app.config['_gps_topics_cache'] = []
    
    # A cache for status thresholds from the ROS params
    app.config['STATUS_THRESHOLDS'] = status_thresholds or {}

    # Initialize extensions with the app
    db.init_app(app)
    sio_server.init_app(app, cors_allowed_origins="*")

    # --- Import and Register Components ---
    with app.app_context():
        # Import models and create database tables
        from . import models
        db.create_all()

        @app.context_processor
        def inject_global_data():
            """Injects data into all templates."""
            waypoints = models.Waypoint.query.order_by(models.Waypoint.id).all()
            thresholds = current_app.config.get('STATUS_THRESHOLDS', {})
            initial_data = {
                'vitals': {'voltage': 0, 'current': 0},
                'computer_info': {'cpu_temp': 0, 'cpu_usage': 0, 'mem_usage': 0},
                'poses': {'lat': 0, 'lon': 0, 'yaw': 0, 'z': 0},
                'items': [],
                'waypoints': waypoints,
                'states': [],
                'controller_state': {'state': 'Unknown'},
                'status_thresholds_json': json.dumps(thresholds)
            }
            return initial_data

        # Import and register Blueprints for routes
        from .routes.routes_base import base_bp
        from .routes.routes_map import map_bp
        from .routes.routes_mission import mission_bp
        from .routes.routes_power_manager import power_manager_bp
        from .routes.routes_systems import systems_bp
        
        app.register_blueprint(base_bp)
        app.register_blueprint(map_bp)
        app.register_blueprint(mission_bp)
        app.register_blueprint(power_manager_bp)
        app.register_blueprint(systems_bp)

        # Import events file to register all SocketIO event handlers
        from . import events

    return app, sio_server