from flask_sqlalchemy import SQLAlchemy
from flask_socketio import SocketIO

# Create extension instances without attaching them to an app yet
db = SQLAlchemy()
sio_server = SocketIO(logger=False, engineio_logger=False)