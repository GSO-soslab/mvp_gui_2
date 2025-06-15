from flask import render_template, request, redirect, url_for, jsonify
from mvp_gui import app, sio_server

@app.route("/power_manager") 
def power_manager_page():
    # Power items are now dynamically loaded in the browser via WebSockets.
    # We no longer pass them from the server at render time.
    return render_template("power_manager.html", current_page='power_manager')

# This endpoint is deprecated and has been removed.
# Real-time data is now sent via the 'power_update' WebSocket event.