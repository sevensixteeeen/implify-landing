"""g(h).

ARCHITECTURE section 4.4. The truth model uses inverse-square gravity; a constant-gravity mode
exists so results can be checked against closed-form solutions (FR-SIM-10).
"""
from aerosat.core.constants import G0_MPS2, R0_M


def inverse_square_gravity_mps2(altitude_msl_m: float) -> float:
    """Gravitational acceleration (m/s^2) at a geometric altitude above mean sea level (m)."""
    ratio = R0_M / (R0_M + altitude_msl_m)
    return G0_MPS2 * ratio * ratio


def constant_gravity_mps2(altitude_msl_m: float) -> float:
    """Standard gravity g0 (m/s^2) at every altitude."""
    return G0_MPS2
