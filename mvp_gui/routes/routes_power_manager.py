# mvp_gui/routes/routes_power_manager.py
from flask import render_template, Blueprint

power_manager_bp = Blueprint('power_manager_bp', __name__)

@power_manager_bp.route("/power_manager") 
def power_manager_page():
    return render_template("power_manager.html", current_page='power_manager')