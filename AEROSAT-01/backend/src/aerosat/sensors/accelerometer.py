"""Accelerometer model: specific force, m/s^2. Three channels (x, y, z; z = up).

The accelerometer outputs specific force, not kinematic acceleration (FR-SEN-07): +g at rest on
the pad, (F_T - F_D)/m in powered flight, about 0 in drag-free fall. In the 1-DOF model the
lateral axes have zero truth and output bias and noise only.
"""
from aerosat.sensors.pipeline import Sensor


class Accelerometer(Sensor):
    sensor_id = "accel"
    unit = "m/s^2"
    truth_sources = (
        ("x", lambda truth: 0.0),
        ("y", lambda truth: 0.0),
        ("z", lambda truth: truth.specific_force_mps2),
    )
