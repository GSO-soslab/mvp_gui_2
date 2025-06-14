from flask import render_template, request, redirect, url_for, jsonify
from mvp_gui import app, sio_server
from decimal import Decimal

@app.route("/power_manager") 
def power_manager_page():
    # Power items are loaded from config and passed to the template.
    # State and actions are now managed in real-time via WebSockets on the client.
    power_items = app.config.get('POWER_ITEMS', [])
    return render_template("power_manager.html", power_items=power_items, current_page='power_manager')

# This endpoint is deprecated and has been removed.
# Real-time data is now sent via the 'power_update' WebSocket event.