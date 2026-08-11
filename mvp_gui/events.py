from flask import current_app, request
from flask_socketio import join_room # Import the join_room function
from .web_utils import sio_server, db
from .models import Waypoint

# Define a constant for the room name to avoid typos
BROADCAST_ROOM = 'all_clients_room'

# --- Server-side cache for stateful data ---
last_published_path = []
last_vehicle_pose = None # Cache for vehicle pose
last_altimeter_range = None # Cache for vehicle altimeter range
last_launch_status = None # Cache for launch status

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
    if last_altimeter_range:
        sio_server.emit('altimeter_update', last_altimeter_range, to=request.sid)

    # Send cached GPS topics if they exist
    cached_gps_topics = current_app.config.get('_gps_topics_cache', [])
    if cached_gps_topics:
        sio_server.emit('gps_topics_discovered', {'topics': cached_gps_topics}, to=request.sid)

    # Send cached launch keys if they exist
    cached_launch_keys = current_app.config.get('_launch_keys_cache', [])
    if cached_launch_keys:
        print(f"Sending cached launch keys to new client {request.sid}")
        sio_server.emit('update_launch_keys', {'keys': cached_launch_keys}, to=request.sid)
        
    # Send cached launch status if it exists
    if last_launch_status:
        print(f"Sending cached launch status to new client {request.sid}")
        sio_server.emit('launch_status_update', last_launch_status, to=request.sid)


@sio_server.on('disconnect')
def handle_disconnect():
    """Logs when a client disconnects."""
    # The client is automatically removed from rooms on disconnect.
    print(f'Client disconnected: {request.sid}')

# --- Handlers for events FROM ros interface node, relayed TO broadcast room(flask node / browser) ---
# We now emit to the room instead of using `broadcast=True`.

@sio_server.on('vehicle_pose_update')
def handle_vehicle_pose_update(data):
    """Relay vehicle pose from ROS node to all browser clients in the room and cache it."""
    global last_vehicle_pose
    last_vehicle_pose = data # Store the latest pose
    sio_server.emit('vehicle_pose_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('altimeter_update')
def handle_altimeter_update(data):
    """Relay vehicle altimeter data from ROS node to all browser clients in the room and cache it."""
    global last_altimeter_range
    last_altimeter_range = data # Store the latest altimeter range
    sio_server.emit('altimeter_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('power_update')
def handle_power_update(data):
    """Relay power status from ROS node to all browser clients in the room."""
    sio_server.emit('power_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('lumen_update')
def handle_lumen_update(data):
    """Relay lumen brightness from ROS node to all browser clients in the room."""
    sio_server.emit('lumen_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('power_info_update')
def handle_power_info_update(data):
    """Relay power status from ROS node to all browser clients in the room."""
    sio_server.emit('power_info_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('computer_info_update')
def handle_computer_info_update(data):
    """Relay power status from ROS node to all browser clients in the room."""
    sio_server.emit('computer_info_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)

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
    """Relay launch status from ROS node to all browser clients in the room and cache it."""
    global last_launch_status
    last_launch_status = data # Cache the latest status
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
    This updates the server's cache in the Flask app config and relays the update
    to all connected browser clients.
    """
    keys = data.get('keys', [])
    print(f"Received launch key update from ROS node, relaying to browsers: {keys}")
    # Update the cache stored in the application config
    current_app.config['_launch_keys_cache'] = keys
    # Relay this update to all browser clients in the room.
    sio_server.emit('update_launch_keys', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('gps_topics_discovered')
def handle_gps_topics_discovered(data):
    """
    Relay the list of discovered GPS topics from the ROS node
    to all browser clients in the room and cache it.
    """
    topics = data.get('topics', [])
    current_app.config['_gps_topics_cache'] = topics
    sio_server.emit('gps_topics_discovered', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('gps_topic_subscribed')
def handle_gps_topic_subscribed(data):
    """Relay subscription confirmation from ROS node to all browser clients."""
    sio_server.emit('gps_topic_subscribed', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('gps_topic_unsubscribed')
def handle_gps_topic_unsubscribed(data):
    """Relay unsubscription confirmation from ROS node to all browser clients."""
    sio_server.emit('gps_topic_unsubscribed', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('dynamic_gps_update')
def handle_dynamic_gps_update(data):
    """Relay dynamic GPS data from ROS node to all browser clients."""
    sio_server.emit('dynamic_gps_update', data, to=BROADCAST_ROOM, skip_sid=request.sid)
    
# --- Handlers for events FROM browsers ---

def _renumber_waypoints():
    """
    Helper function to re-number waypoint IDs to be sequential, starting from 1.
    This should be called within an application context.
    """
    waypoints_data = [(w.lat, w.lon, w.alt, w.surge) for w in Waypoint.query.order_by(Waypoint.id).all()]
    db.session.query(Waypoint).delete()
    for i, data in enumerate(waypoints_data):
        new_waypoint = Waypoint(id=i + 1, lat=data[0], lon=data[1], alt=data[2], surge=data[3])
        db.session.add(new_waypoint)
    db.session.commit()

def _get_all_waypoints_payload():
    """
    Helper function to fetch all waypoints and return them as a JSON-serializable list.
    This should be called within an application context.
    """
    all_waypoints = Waypoint.query.order_by(Waypoint.id).all()
    return [
        {"id": w.id, "lat": w.lat, "lon": w.lon, "alt": w.alt, "surge": w.surge} 
        for w in all_waypoints
    ]

@sio_server.on('add_waypoint')
def handle_add_waypoint(data):
    """
    Handles a request from a browser client to add a new waypoint.
    Adds it to the database, renumbers all waypoints, and broadcasts the full list.
    """
    print(f"Received add waypoint request from sid={request.sid}: {data}")
    with current_app.app_context():
        new_waypoint = Waypoint(
            lat=float(data.get('lat', 0)),
            lon=float(data.get('lon', 0)),
            alt=0.0,
            surge=0.0
        )
        db.session.add(new_waypoint)
        db.session.commit()
        
        _renumber_waypoints()
        
        waypoints_payload = _get_all_waypoints_payload()
        sio_server.emit('waypoints_updated', {'waypoints': waypoints_payload}, to=BROADCAST_ROOM)
        print("Broadcasted updated waypoint list to all clients after adding.")

@sio_server.on('delete_waypoint')
def handle_delete_waypoint(data):
    """
    Handles a request from a browser client to delete a waypoint.
    Deletes it, renumbers remaining waypoints, and broadcasts the full list.
    """
    waypoint_id = data.get('id')
    print(f"Received delete waypoint request from sid={request.sid} for id={waypoint_id}")
    if waypoint_id is None:
        print("Error: Waypoint delete request missing 'id'.")
        return

    with current_app.app_context():
        waypoint = Waypoint.query.get(int(waypoint_id))
        
        if waypoint:
            db.session.delete(waypoint)
            db.session.commit()
            print(f"Waypoint {waypoint_id} deleted successfully.")

            _renumber_waypoints()
            
            waypoints_payload = _get_all_waypoints_payload()
            sio_server.emit('waypoints_updated', {'waypoints': waypoints_payload}, to=BROADCAST_ROOM)
            print("Broadcasted updated waypoint list to all clients after deleting.")
        else:
            print(f"Error: Waypoint with id {waypoint_id} not found for deletion.")

@sio_server.on('update_waypoint')
def handle_update_waypoint(data):
    """
    Handles a request from a browser client to update a single waypoint.
    Updates the database and then broadcasts the full, updated list of waypoints
    to all clients to ensure synchronization.
    """
    print(f"Received waypoint update request from sid={request.sid}: {data}")
    with current_app.app_context():
        waypoint_id = data.get('id')
        if waypoint_id is None:
            print("Error: Waypoint update request missing 'id'.")
            return

        waypoint = Waypoint.query.get(int(waypoint_id))
        
        if waypoint:
            waypoint.lat = float(data.get('lat', waypoint.lat))
            waypoint.lon = float(data.get('lon', waypoint.lon))
            waypoint.alt = float(data.get('alt', waypoint.alt))
            waypoint.surge = float(data.get('surge', waypoint.surge))
            db.session.commit()
            print(f"Waypoint {waypoint_id} updated successfully.")

            waypoints_payload = _get_all_waypoints_payload()
            sio_server.emit('waypoints_updated', {'waypoints': waypoints_payload}, to=BROADCAST_ROOM)
            print("Broadcasted updated waypoint list to all clients.")
        else:
            print(f"Error: Waypoint with id {waypoint_id} not found for update.")


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
        waypoints_payload = [{"lat": w.lat, "lon": w.lon, "alt": w.alt, "surge": w.surge} for w in waypoints]
        sio_server.emit('ros_action', {'action': 'publish_waypoints', 'waypoints': waypoints_payload}, to=BROADCAST_ROOM)

@sio_server.on('discover_gps_topics')
def handle_discover_gps_topics(data):
    """
    A browser client requested to discover GPS topics.
    Relay this request to the ROS node which will handle the discovery
    and respond with the list of available topics.
    """
    print(f"Browser sid={request.sid} requested GPS topic discovery.")
    sio_server.emit('discover_gps_topics', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('subscribe_new_gps_topic')
def handle_subscribe_new_gps_topic(data):
    """Relay subscribe request from browser to ROS node."""
    print(f"Relaying subscribe request for GPS topic: {data}")
    sio_server.emit('subscribe_new_gps_topic', data, to=BROADCAST_ROOM, skip_sid=request.sid)

@sio_server.on('unsubscribe_gps_topic')
def handle_unsubscribe_gps_topic(data):
    """Relay unsubscribe request from browser to ROS node."""
    print(f"Relaying unsubscribe request for GPS topic: {data}")
    sio_server.emit('unsubscribe_gps_topic', data, to=BROADCAST_ROOM, skip_sid=request.sid)