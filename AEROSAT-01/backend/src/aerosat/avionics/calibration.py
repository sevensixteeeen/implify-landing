"""Pad reference pressure, temperature, accel bias.

ARCHITECTURE section 6.1. While in PAD_IDLE, over the calibration period: p_ref is the mean
barometer reading, T_ref the mean temperature reading, and b_a the mean z-accelerometer reading
minus g0. Uses measurements only.
"""
from collections.abc import Iterable
from dataclasses import dataclass

from aerosat.core.constants import G0_MPS2
from aerosat.core.faults import ConfigurationFault, ConfigurationReason
from aerosat.core.types import SensorSample


@dataclass(frozen=True, slots=True)
class CalibrationResult:
    reference_pressure_pa: float
    reference_temperature_k: float
    accel_bias_mps2: float
    """Estimated z-accelerometer bias b_a: mean reading minus g0."""
    accel_sample_count: int = 0
    """Valid z-accelerometer samples in the mean (N_cal of the disagreement detector, F1_DECISIONS 6b)."""


class PadCalibration:
    """Accumulates pad readings and returns their means."""

    _CHANNELS = (("baro", "pressure"), ("temp", "temperature"), ("accel", "z"))

    def __init__(self) -> None:
        self._sum = {key: 0.0 for key in self._CHANNELS}
        self._count = {key: 0 for key in self._CHANNELS}

    def add(self, samples: Iterable[SensorSample]) -> None:
        for sample in samples:
            key = (sample.sensor_id, sample.channel)
            if key in self._sum:
                self._sum[key] += sample.value
                self._count[key] += 1

    def result(self) -> CalibrationResult:
        """Means over the valid readings added so far.

        The caller adds only valid samples: a saturated or non-finite reading never reaches a
        mean. Raises `ConfigurationFault` if a required sensor gave no valid reading.
        """
        missing = [f"{sensor}.{channel}" for (sensor, channel), count in self._count.items() if count == 0]
        if missing:
            raise ConfigurationFault(
                ConfigurationReason.NO_VALID_CALIBRATION_SAMPLES, f"sensors.{missing[0].split('.')[0]}",
                f"pad calibration needs valid readings from {', '.join(missing)}; the sensor is disabled, "
                "produced no samples, or produced only saturated or non-finite samples during calibration",
            )
        mean = {key: self._sum[key] / self._count[key] for key in self._CHANNELS}
        return CalibrationResult(
            reference_pressure_pa=mean[("baro", "pressure")],
            reference_temperature_k=mean[("temp", "temperature")],
            accel_bias_mps2=mean[("accel", "z")] - G0_MPS2,
            accel_sample_count=self._count[("accel", "z")],
        )
