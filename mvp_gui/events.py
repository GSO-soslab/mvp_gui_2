from flask import current_app, request
from flask_socketio import join_room # Import the join_room function
from mvp_gui import sio_server, db
from mvp_gui.models import Waypoint

# Define a constant for the room name to avoid typos
BROADCAST_ROOM = 'all_clients_room'

# --- Handlers for built-in events ---
@sio_server.on('connect')
def handle_connect():
    """Logs when a client connects and adds them to the broadcast room."""
    join_room(BROADCAST_ROOM) # Add the new client to our room
    print(f'Client connected: {request.sid}, and joined room: "{BROADCAST_ROOM}"')

@sio_server.on('disconnect')
def handle_disconnect():
    """Logs when a client disconnects."""
    # The client is automatically removed from rooms on disconnect.
    print(f'Client disconnected: {request.sid}')

# --- Handlers for events FROM ros_interface.py, relayed TO browsers ---
# We now emit to the room instead of using `broadcast=True`.

@sio_server.on('vehicle_pose_update')
def handle_vehicle_pose_update(data):
    """Relay vehicle pose from ROS node to all browser clients in the room."""
    sio_server.emit('vehicle_pose_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('power_update')
def handle_power_update(data):
    """Relay power status from ROS node to all browser clients in the room."""
    sio_server.emit('power_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('helm_state_update')
def handle_helm_state_update(data):
    """Relay helm state from ROS node to all browser clients in the room."""
    sio_server.emit('helm_state_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('controller_state_update')
def handle_controller_state_update(data):
    """Relay controller state from ROS node to all browser clients in the room."""
    sio_server.emit('controller_state_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('launch_status_update')
def handle_launch_status_update(data):
    """Relay launch status from ROS node to all browser clients in the room."""
    sio_server.emit('launch_status_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)
    
# --- Handlers for events FROM browsers ---

@sio_server.on('ros_action')
def handle_ros_action(data):
    """
    Relay a command from a browser client to all other clients in the room (specifically the ROS node).
    """
    print(f"Relaying browser action from sid={request.sid} to room '{BROADCAST_ROOM}': {data}")
    sio_server.emit('ros_action', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('publish_waypoints_request')
def handle_publish_waypoints_request():
    """
    A browser client requested to publish waypoints.
    1. Get waypoints from DB.
    2. Emit a 'ros_action' for all clients in the room (specifically the ROS node) to handle.
    """
    print(f"Browser sid={request.sid} requested to publish waypoints.")
    with current_app.app_context():
        waypoints = Waypoint.query.order_by(Waypoint.id).all()
        waypoints_payload = [{"lat": str(w.lat), "lon": str(w.lon), "alt": str(w.alt)} for w in waypoints]
        # Emit to the room. The ROS node is in the room and will receive this.
        sio_server.emit('ros_action', {'action': 'publish_waypoints', 'waypoints': waypoints_payload}, to=BROADCAST_ROOM)