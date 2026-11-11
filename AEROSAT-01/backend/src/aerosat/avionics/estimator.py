"""Barometric altitude; complementary filter.

ARCHITECTURE section 6.2. The avionics model assumes constant gravity g0 and the layer-1
standard-atmosphere pressure law. It deliberately differs from the truth model.
"""
from aerosat.core.constants import G0_MPS2, LAPSE_RATE_KPM, R_AIR_JPKGK

_BARO_EXPONENT = R_AIR_JPKGK * LAPSE_RATE_KPM / G0_MPS2  # dimensionless, about 0.190263


def baro_altitude_agl_m(pressure_pa: float, reference_pressure_pa: float, reference_temperature_k: float) -> float:
    """Altitude above the pad (m) from a pressure reading, inverting the layer-1 pressure law.

    h_baro = (T_ref / L) * (1 - (p / p_ref)^(R*L/g0))
    """
    return reference_temperature_k / LAPSE_RATE_KPM * (1.0 - (pressure_pa / reference_pressure_pa) ** _BARO_EXPONENT)


class ComplementaryFilter:
    """Blends fast-but-drifting accelerometer integration with slow-but-stable barometric altitude.

    `gain_h` is dimensionless; `gain_v_hz` is in 1/s. Both are applied once per barometer sample.
    """

    def __init__(self, gain_h: float, gain_v_hz: float, period_s: float) -> None:
        self._gain_h = gain_h
        self._gain_v_hz = gain_v_hz
        self._period_s = period_s
        self.altitude_agl_m = 0.0
        self.velocity_mps = 0.0

    def predict(self, acceleration_mps2: float) -> None:
        """Advance one avionics period using the estimated vertical acceleration."""
        period_s = self._period_s
        self.altitude_agl_m += self.velocity_mps * period_s + 0.5 * acceleration_mps2 * period_s * period_s
        self.velocity_mps += acceleration_mps2 * period_s

    def correct(self, baro_altitude_m: float) -> None:
        """Pull the estimate toward a barometric altitude."""
        residual_m = baro_altitude_m - self.altitude_agl_m
        self.altitude_agl_m += self._gain_h * residual_m
        self.velocity_mps += self._gain_v_hz * residual_m


def estimated_acceleration_mps2(specific_force_z_mps2: float, accel_bias_mps2: float) -> float:
    """Vertical kinematic acceleration: bias-corrected specific force minus g0. About 0 at rest."""
    return (specific_force_z_mps2 - accel_bias_mps2) - G0_MPS2
