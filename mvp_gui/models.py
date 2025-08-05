from .web_utils import db

# --- Persistent Data ---
class Waypoint(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    lat = db.Column(db.Float, nullable=False)
    lon = db.Column(db.Float, nullable=False)
    alt = db.Column(db.Float, nullable=False)
    surge = db.Column(db.Float, nullable=False)

    def __repr__(self):
        return f'<Waypoint {self.id}>'