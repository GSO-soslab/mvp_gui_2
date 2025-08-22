from flask import render_template, Blueprint

base_bp = Blueprint('base_bp', __name__)

@base_bp.route("/vehicle_status", methods=['GET', 'POST'])
def vehicle_status_page():
    return render_template("vehicle_status.html", current_page='vehicle_status')