from mvp_gui import db
from decimal import Decimal

# --- Persistent Configuration and Data ---

class Waypoint(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    lat = db.Column(db.Numeric(10, 8), nullable=False)
    lon = db.Column(db.Numeric(10, 8), nullable=False)
    alt = db.Column(db.Numeric(10, 2), nullable=False)
    def __repr__(self):
        return f'<Waypoint {self.id}>'