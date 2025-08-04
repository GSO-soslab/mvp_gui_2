from flask_wtf import FlaskForm
from wtforms import SubmitField, FloatField
from wtforms.validators import DataRequired

class WaypointForm(FlaskForm):
    lat = FloatField(label='Latitude [deg]', validators=[DataRequired()])
    lon = FloatField(label='Longitude [deg]', validators=[DataRequired()])
    alt = FloatField(label='Altitude [m]', validators=[DataRequired()])
    surge = FloatField(label='Surge Velocity [m/s]', validators=[DataRequired()])
    submit = SubmitField('Submit')