from mvp_gui import db
# 'Decimal' is no longer needed as we are using Float for coordinates.

# --- Persistent Configuration and Data ---

class Waypoint(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    # CHANGED: Use db.Float to store coordinates as 64-bit floating-point numbers.
    # This directly matches the precision from the map and avoids conversion/rounding issues.
    lat = db.Column(db.Float, nullable=False)
    lon = db.Column(db.Float, nullable=False)
    # Altitude can remain as Numeric since its precision requirements are lower.
    alt = db.Column(db.Numeric(10, 2), nullable=False)
    def __repr__(self):
        return f'<Waypoint {self.id}>'