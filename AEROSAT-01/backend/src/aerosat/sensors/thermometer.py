"""Temperature sensor model: ambient air temperature, K. One channel. Truth source: T(h).

Idealised as measuring ambient air with a first-order thermal lag. A real board-mounted sensor
mostly measures the board. This is a simplification, not a claim.
"""
from aerosat.sensors.pipeline import Sensor


class Thermometer(Sensor):
    sensor_id = "temp"
    unit = "K"
    truth_sources = (("temperature", lambda truth: truth.temperature_k),)
