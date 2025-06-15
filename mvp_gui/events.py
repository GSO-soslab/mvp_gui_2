from flask import current_app, request
from flask_socketio import join_room # Import the join_room function
from mvp_gui import sio_server, db
from mvp_gui.models import Waypoint

# Define a constant for the room name to avoid typos
BROADCAST_ROOM = 'all_clients_room'

# --- Server-side cache for stateful data ---
last_published_path = []
last_vehicle_pose = None # Cache for vehicle pose

# --- Handlers for built-in events ---
@sio_server.on('connect')
def handle_connect():
    """
    Logs when a client connects, adds them to the broadcast room,
    and sends them the last known state.
    """
    join_room(BROADCAST_ROOM) # Add the new client to our room
    print(f'Client connected: {request.sid}, and joined room: "{BROADCAST_ROOM}"')
    
    # Immediately send the last known state to the connecting client
    if last_published_path:
        sio_server.emit('published_path_update', last_published_path, to=request.sid)
    if last_vehicle_pose:
        sio_server.emit('vehicle_pose_update', last_vehicle_pose, to=request.sid)

@sio_server.on('disconnect')
def handle_disconnect():
    """Logs when a client disconnects."""
    # The client is automatically removed from rooms on disconnect.
    print(f'Client disconnected: {request.sid}')

# --- Handlers for events FROM ros_interface.py, relayed TO browsers ---
# We now emit to the room instead of using `broadcast=True`.

@sio_server.on('vehicle_pose_update')
def handle_vehicle_pose_update(data):
    """Relay vehicle pose from ROS node to all browser clients in the room and cache it."""
    global last_vehicle_pose
    last_vehicle_pose = data # Store the latest pose
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
    
@sio_server.on('published_path_update')
def handle_published_path_update(data):
    """Relay published path from ROS node to all browser clients, and cache it."""
    global last_published_path
    last_published_path = data  # Cache the latest path
    sio_server.emit('published_path_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('update_launch_keys')
def handle_update_launch_keys(data):
    """
    Event handler for when the ROS node sends the list of launch files.
    This updates the server's cache in the Flask app config.
    """
    keys = data.get('keys', [])
    print(f"Received launch key update from ROS node: {keys}")
    # Update the cache stored in the application config
    current_app.config['_launch_keys_cache'] = keys
    
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
        # CORRECTED: Since lat/lon are stored as Float, no conversion is needed.
        # w.alt is still a Decimal from Numeric, so it needs to be converted to float.
        waypoints_payload = [{"lat": w.lat, "lon": w.lon, "alt": float(w.alt)} for w in waypoints]
        # Emit to the room. The ROS node is in the room and will receive this.
        sio_server.emit('ros_action', {'action': 'publish_waypoints', 'waypoints': waypoints_payload}, to=BROADCAST_ROOM)