from flask import Flask
from flask_socketio import SocketIO
from flask_sqlalchemy import SQLAlchemy
import yaml
import os

# Load configuration from YAML file
config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'config.yaml')
if not os.path.exists(config_path):
    raise FileNotFoundError(f"Configuration file not found at: {config_path}")

with open(config_path, 'r') as file:
    yaml_config = yaml.safe_load(file)

# Initialize Flask app
app = Flask(__name__)
app.config['SECRET_KEY'] = 'abcd1234'
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///mvp_gui.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

# Set other configurations from YAML
app.config['HOST_IP'] = yaml_config.get('host_ip', '127.0.0.1')
app.config['_launch_keys_cache'] = []
SUBPROCESS_TIMEOUT = yaml_config.get('subprocess_timeout', 5)


# Setup environment for subprocesses
env = os.environ.copy()
project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
env['PYTHONPATH'] = env.get('PYTHONPATH', '') + ':' + project_path

# Initialize extensions
db = SQLAlchemy(app)
sio_server = SocketIO(app, logger=False, engineio_logger=False)
# ssh_connection has been removed

# Import routes, models, and events after initializing extensions to avoid circular imports
from mvp_gui import models
from mvp_gui.routes import routes_base, routes_map, routes_mission, routes_power_manager, routes_systems
from mvp_gui import events

with app.app_context():
    db.create_all()