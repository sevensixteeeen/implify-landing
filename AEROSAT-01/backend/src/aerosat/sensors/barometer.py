"""Barometer model: static pressure, Pa. One channel. Truth source: p(h)."""
from aerosat.sensors.pipeline import Sensor


class Barometer(Sensor):
    sensor_id = "baro"
    unit = "Pa"
    truth_sources = (("pressure", lambda truth: truth.pressure_pa),)
