# mvp_gui/routes/routes_mission.py
from flask import render_template, request, redirect, url_for, Blueprint, current_app
from ..web_utils import db
from ..models import Waypoint
from ..forms import WaypointForm
from .. import events
import xml.etree.ElementTree as ET
import os
from ament_index_python.packages import get_package_share_directory

mission_bp = Blueprint('mission_bp', __name__)

def renumber_waypoints():
    with current_app.app_context():
        waypoints_data = [(w.lat, w.lon, w.alt, w.surge) for w in Waypoint.query.order_by(Waypoint.id).all()]
        db.session.query(Waypoint).delete()
        for i, data in enumerate(waypoints_data):
            new_waypoint = Waypoint(id=i + 1, lat=data[0], lon=data[1], alt=data[2], surge=data[3])
            db.session.add(new_waypoint)
        db.session.commit()

def generate_waypoints_from_kml(file_path, replace_flag):
    tree = ET.parse(file_path)
    root = tree.getroot()
    if replace_flag:
        db.session.query(Waypoint).delete()
    for placemark in root.findall('.//{http://www.opengis.net/kml/2.2}Placemark'):
        coordinates = placemark.find('.//{http://www.opengis.net/kml/2.2}coordinates').text
        lon, lat, alt = map(float, coordinates.strip().split(','))
        waypoint = Waypoint(lat=lat, lon=lon, alt=alt, surge=0.0)
        db.session.add(waypoint)
    db.session.commit()
    renumber_waypoints()

@mission_bp.route("/mission", methods=['GET', 'POST'])
def mission_page():
    no_pose_available = request.args.get('no_pose_reset') == 'true'
    waypoints = Waypoint.query.order_by(Waypoint.id).all()
    
    if request.method == 'POST':
        action_url = url_for('mission_bp.mission_page')
        if 'add' in request.form:
            return redirect(url_for('mission_bp.add_waypoint_page'))
        elif 'delete' in request.form:
            entry = Waypoint.query.get(request.form['delete'])
            if entry:
                db.session.delete(entry)
                db.session.commit()
                renumber_waypoints()
            return redirect(action_url)
        elif 'edit' in request.form:
            return redirect(url_for('mission_bp.edit_waypoint_page', edit_id=request.form['edit']))
        elif 'copy' in request.form:
            entry = Waypoint.query.get(request.form['copy'])
            if entry:
                new_entry = Waypoint(lat=entry.lat, lon=entry.lon, alt=entry.alt, surge=entry.surge)
                db.session.add(new_entry)
                db.session.commit()
                renumber_waypoints()
            return redirect(action_url)

    return render_template("mission.html", waypoints=waypoints, current_page="mission", no_pose_available=no_pose_available)

@mission_bp.route("/mission/reset_waypoints", methods=['POST'])
def reset_waypoints():
    with current_app.app_context():
        db.session.query(Waypoint).delete()
        if events.last_vehicle_pose:
            pose = events.last_vehicle_pose
            new_waypoint = Waypoint(lat=pose.get('lat', 0) + 0.0001, lon=pose.get('lon', 0), alt=pose.get('alt', 0), surge=pose.get('u', 0))
            db.session.add(new_waypoint)
            db.session.commit()
            renumber_waypoints()
            return redirect(url_for('mission_bp.mission_page'))
        else:
            db.session.commit()
            return redirect(url_for('mission_bp.mission_page', no_pose_reset='true'))

@mission_bp.route('/mission/upload', methods=['POST'])
def upload_file():
    pkg_share_dir = get_package_share_directory('mvp_gui_2')
    upload_folder = os.path.join(pkg_share_dir, 'kml_uploads')
    os.makedirs(upload_folder, exist_ok=True)
    
    if 'fileToUpload' not in request.files or request.files['fileToUpload'].filename == '':
        return 'No file selected', 400
    
    file = request.files['fileToUpload']
    replace = (request.form.get('action') == 'replace')
    file_path = os.path.join(upload_folder, file.filename)
    file.save(file_path)
    generate_waypoints_from_kml(file_path, replace)

    return redirect(url_for('mission_bp.mission_page'))

@mission_bp.route('/mission/edit_waypoint', methods=['GET', 'POST'])
def edit_waypoint_page():
    entry = Waypoint.query.get_or_404(request.args.get('edit_id'))
    form = WaypointForm(obj=entry)
    if form.validate_on_submit():
        form.populate_obj(entry)
        db.session.commit()
        return redirect(url_for('mission_bp.mission_page'))
    return render_template('secondary_pages/edit_waypoint.html', form=form, entry=entry)

@mission_bp.route('/mission/add_waypoint', methods=['GET', 'POST'])
def add_waypoint_page():
    form = WaypointForm()
    if form.validate_on_submit():
        new_waypoint = Waypoint()
        form.populate_obj(new_waypoint)
        db.session.add(new_waypoint)
        db.session.commit()
        renumber_waypoints()
        return redirect(url_for('mission_bp.mission_page'))
    return render_template('secondary_pages/add_waypoints.html', form=form)