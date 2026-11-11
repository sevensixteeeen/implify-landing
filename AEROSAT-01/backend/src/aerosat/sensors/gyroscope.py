"""Gyroscope model: angular rate, rad/s. Three channels (x, y, z; z = roll).

Roll is a scripted kinematic profile, not simulated dynamics. In the 1-DOF model the pitch and
yaw axes have zero truth and output bias and noise only.
"""
from aerosat.sensors.pipeline import Sensor


class Gyroscope(Sensor):
    sensor_id = "gyro"
    unit = "rad/s"
    truth_sources = (
        ("x", lambda truth: 0.0),
        ("y", lambda truth: 0.0),
        ("z", lambda truth: truth.roll_rate_radps),
    )
