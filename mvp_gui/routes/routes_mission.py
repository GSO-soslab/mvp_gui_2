from flask import render_template, request, redirect, url_for, jsonify
from mvp_gui import app, db
from mvp_gui.models import Waypoint
from mvp_gui.forms import WaypointForm
import xml.etree.ElementTree as ET
import os

def generate_waypoints_from_kml(file_path, replace_flag):
    tree = ET.parse(file_path)
    root = tree.getroot()

    if replace_flag:
        db.session.query(Waypoint).delete()
        db.session.commit()

    for placemark in root.findall('.//{http://www.opengis.net/kml/2.2}Placemark'):
        coordinates = placemark.find('.//{http://www.opengis.net/kml/2.2}coordinates').text
        lon, lat, alt = map(float, coordinates.strip().split(','))
        waypoint = Waypoint(lat=lat, lon=lon, alt=alt)
        db.session.add(waypoint)
    db.session.commit()

@app.route("/mission", methods=['GET', 'POST'])
def mission_page():
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
            return redirect(url_for('mission_page'))

        elif 'edit' in request.form:
            edit_id = request.form['edit']
            return redirect(url_for('edit_waypoint_page', edit_id=edit_id))
        
        elif 'copy' in request.form:
            copy_id = request.form['copy']
            entry = Waypoint.query.get(copy_id)
            if entry:
                new_entry = Waypoint(lat=entry.lat, lon=entry.lon, alt=entry.alt)
                db.session.add(new_entry)
                db.session.commit()
            return redirect(url_for('mission_page'))

    return render_template("mission.html", waypoints=waypoints, current_page="mission")

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
    
    return render_template('edit_waypoint.html', form=form, entry=entry)

@app.route('/mission/add_waypoint', methods=['GET', 'POST'])
def add_waypoint_page():
    form = WaypointForm()
    if form.validate_on_submit():
        new_waypoint = Waypoint()
        form.populate_obj(new_waypoint)
        db.session.add(new_waypoint)
        db.session.commit()
        return redirect(url_for('mission_page'))
    return render_template('add_waypoints.html', form=form)

# This endpoint is deprecated. Helm/Controller states are now pushed via WebSocket.
@app.route('/mission/states')
def controller_helm_state_data():
    return jsonify({"message": "This endpoint is deprecated. Use WebSocket for real-time data."}), 410