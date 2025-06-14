from flask_wtf import FlaskForm
from wtforms import StringField, SubmitField, FloatField, IntegerField
from wtforms.validators import DataRequired

class WaypointForm(FlaskForm):
    lat = FloatField(label='latitude', validators=[DataRequired()])
    lon = FloatField(label='longitude', validators=[DataRequired()])
    alt = FloatField(label='altitude', validators=[DataRequired()])
    submit = SubmitField('Submit')