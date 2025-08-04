from flask import render_template, request, redirect, url_for, jsonify
from mvp_gui import app, db, events
from mvp_gui.models import Waypoint
from mvp_gui.forms import WaypointForm
import xml.etree.ElementTree as ET
import os

def renumber_waypoints():
    """
    Renumbers all waypoints in the database to have sequential IDs, starting from 1.
    This function is called after any operation that adds or removes waypoints
    to ensure data integrity for other parts of the application that expect
    sequential waypoint IDs.

    NOTE: Modifying primary keys is generally an anti-pattern in database design.
    This is a workaround for a specific application requirement.
    """
    with app.app_context():
        # Step 1: Fetch all waypoints, ordered by their current ID, and store their data.
        # This ensures that we maintain the existing order of waypoints.
        waypoints_data = [(w.lat, w.lon, w.alt, w.surge) for w in Waypoint.query.order_by(Waypoint.id).all()]

        # Step 2: Delete all waypoints from the table.
        # This is a bulk delete operation.
        db.session.query(Waypoint).delete()

        # Step 3: Re-insert all waypoints with new, sequential IDs.
        for i, data in enumerate(waypoints_data):
            # Create a new Waypoint object with an explicit ID.
            new_waypoint = Waypoint(id=i + 1, lat=data[0], lon=data[1], alt=data[2], surge=data[3])
            db.session.add(new_waypoint)
        
        # Step 4: Commit the entire transaction to the database.
        db.session.commit()

def generate_waypoints_from_kml(file_path, replace_flag):
    tree = ET.parse(file_path)
    root = tree.getroot()

    if replace_flag:
        db.session.query(Waypoint).delete()
        db.session.commit()

    for placemark in root.findall('.//{http://www.opengis.net/kml/2.2}Placemark'):
        coordinates = placemark.find('.//{http://www.opengis.net/kml/2.2}coordinates').text
        lon, lat, alt = map(float, coordinates.strip().split(','))
        surge = float(0)
        waypoint = Waypoint(lat=lat, lon=lon, alt=alt, surge=surge)
        db.session.add(waypoint)
    db.session.commit()
    # After adding all waypoints, renumber them to ensure sequential IDs.
    renumber_waypoints()

@app.route("/mission", methods=['GET', 'POST'])
def mission_page():
    # Check for a flag from the reset operation
    no_pose_available = request.args.get('no_pose_reset') == 'true'
    waypoints = Waypoint.query.order_by(Waypoint.id).all()
    
    if request.method == 'POST':
        if 'add' in request.form:
            return redirect(url_for('add_waypoint_page'))
        
        elif 'delete' in request.form:
            waypoint_id = request.form['delete']
            entry = Waypoint.query.get(waypoint_id)
            if entry:
                db.session.delete(entry)
                db.session.commit()
                # After deleting, renumber all subsequent waypoints.
                renumber_waypoints()
            return redirect(url_for('mission_page'))

        elif 'edit' in request.form:
            edit_id = request.form['edit']
            return redirect(url_for('edit_waypoint_page', edit_id=edit_id))
        
        elif 'copy' in request.form:
            copy_id = request.form['copy']
            entry = Waypoint.query.get(copy_id)
            if entry:
                new_entry = Waypoint(lat=entry.lat, lon=entry.lon, alt=entry.alt, surge=entry.surge)
                db.session.add(new_entry)
                db.session.commit()
                # After copying (which adds a waypoint), renumber all.
                renumber_waypoints()
            return redirect(url_for('mission_page'))

    return render_template("mission.html", waypoints=waypoints, current_page="mission", no_pose_available=no_pose_available)

@app.route("/mission/reset_waypoints", methods=['POST'])
def reset_waypoints():
    """
    Deletes all waypoints and creates a single new one based on vehicle pose.
    If no vehicle pose is available, all waypoints are simply deleted.
    """
    with app.app_context():
        # Step 1: Delete all waypoints from the table.
        db.session.query(Waypoint).delete()

        # Step 2: Check for vehicle pose and create a new waypoint.
        if events.last_vehicle_pose:
            # An offset of 0.0001 degrees latitude is roughly 11 meters.
            lat_offset = events.last_vehicle_pose.get('lat', 0) + 0.0001
            lon_offset = events.last_vehicle_pose.get('lon', 0)
            alt_offset = events.last_vehicle_pose.get('alt', 0)
            # CORRECTED: The vehicle pose data uses 'u' for surge velocity.
            surge_offset = events.last_vehicle_pose.get('u', 0)

            new_waypoint = Waypoint(lat=lat_offset, lon=lon_offset, alt=alt_offset, surge=surge_offset)
            db.session.add(new_waypoint)
            db.session.commit()
            
            # Since we added one, renumber it to have ID 1.
            renumber_waypoints()
            return redirect(url_for('mission_page'))
        else:
            # No vehicle pose is available. Just commit the deletion.
            db.session.commit()
            # Redirect with a flag to indicate why the list is empty.
            return redirect(url_for('mission_page', no_pose_reset='true'))


@app.route('/mission/upload', methods=['POST'])
def upload_file():
    upload_folder = 'kml_uploads/'
    os.makedirs(upload_folder, exist_ok=True)
    
    if 'fileToUpload' not in request.files:
        return 'No file part', 400
    file = request.files['fileToUpload']
    if file.filename == '':
        return 'No selected file', 400

    if file:
        action = request.form.get('action')
        replace = (action == 'replace')
        file_path = os.path.join(upload_folder, file.filename)
        file.save(file_path)
        generate_waypoints_from_kml(file_path, replace)

    return redirect(url_for('mission_page'))

@app.route('/mission/edit_waypoint', methods=['GET', 'POST'])
def edit_waypoint_page():
    edit_id = request.args.get('edit_id')
    entry = Waypoint.query.get_or_404(edit_id)
    form = WaypointForm(obj=entry)

    if form.validate_on_submit():
        form.populate_obj(entry)
        db.session.commit()
        return redirect(url_for('mission_page'))
    
    return render_template('secondary_pages/edit_waypoint.html', form=form, entry=entry)

@app.route('/mission/add_waypoint', methods=['GET', 'POST'])
def add_waypoint_page():
    form = WaypointForm()
    if form.validate_on_submit():
        new_waypoint = Waypoint()
        form.populate_obj(new_waypoint)
        db.session.add(new_waypoint)
        db.session.commit()
        # After adding the new waypoint, renumber all to ensure sequential IDs.
        renumber_waypoints()
        return redirect(url_for('mission_page'))
    return render_template('secondary_pages/add_waypoints.html', form=form)

# This endpoint is deprecated. Helm/Controller states are now pushed via WebSocket.
@app.route('/mission/states')
def controller_helm_state_data():
    return jsonify({"message": "This endpoint is deprecated. Use WebSocket for real-time data."}), 410