"""Geopotential altitude, T, p, rho, speed of sound.

ARCHITECTURE section 4.5. Three models:

- `StandardAtmosphere`: the two-layer standard atmosphere, valid for 0-20 km geopotential altitude.
- `ConstantDensityAtmosphere`: a simplified reference atmosphere with fixed values at every
  altitude. Not a realistic altitude-dependent model.
- `VacuumAtmosphere`: no air (zero density and pressure), so no drag.

Every model enforces the 0-20 km range and raises `ModelRangeFault` outside it.
"""
import math
from dataclasses import dataclass

from aerosat.core.constants import G0_MPS2, GAMMA, LAPSE_RATE, P0_PA, R0_M, R_AIR, T0_K
from aerosat.core.faults import ModelRangeFault

H_TROPOPAUSE_M = 11_000.0
"""Geopotential altitude where the isothermal layer starts, m."""

H_MAX_M = 20_000.0
"""Upper limit of the model, geopotential altitude, m."""

_PRESSURE_EXPONENT = G0_MPS2 / (R_AIR * LAPSE_RATE)  # dimensionless, about 5.25588
_T11_K = T0_K - LAPSE_RATE * H_TROPOPAUSE_M
_P11_PA = P0_PA * (_T11_K / T0_K) ** _PRESSURE_EXPONENT

REFERENCE_SPEED_OF_SOUND_MPS = math.sqrt(GAMMA * R_AIR * T0_K)
"""Speed of sound at T0, about 340.3 m/s. Used by the simplified reference atmospheres."""


@dataclass(frozen=True, slots=True)
class AirState:
    """Atmospheric properties at one altitude."""

    temperature_k: float
    pressure_pa: float
    density_kgpm3: float
    speed_of_sound_mps: float


def geopotential_altitude_m(altitude_msl_m: float) -> float:
    """Geopotential altitude H (m) for a geometric altitude h above mean sea level (m)."""
    return R0_M * altitude_msl_m / (R0_M + altitude_msl_m)


def geometric_altitude_m(geopotential_m: float) -> float:
    """Geometric altitude h above mean sea level (m) for a geopotential altitude H (m)."""
    return R0_M * geopotential_m / (R0_M - geopotential_m)


def _geopotential_in_range_m(altitude_msl_m: float, clamp: bool) -> float:
    """Geopotential altitude, checked against 0-20 km. With `clamp`, it is limited instead of rejected."""
    geopotential_m = geopotential_altitude_m(altitude_msl_m)
    if 0.0 <= geopotential_m <= H_MAX_M:
        return geopotential_m
    if clamp:
        return min(max(geopotential_m, 0.0), H_MAX_M)
    raise ModelRangeFault(
        f"Altitude left the 0-20 km model range (geopotential altitude {geopotential_m:.3f} m)."
    )


def standard_air_at_geopotential(geopotential_m: float) -> AirState:
    """Standard-atmosphere properties at a geopotential altitude H in metres, 0 <= H <= 20 000."""
    if not 0.0 <= geopotential_m <= H_MAX_M:
        raise ModelRangeFault(
            f"Altitude left the 0-20 km model range (geopotential altitude {geopotential_m:.3f} m)."
        )
    if geopotential_m <= H_TROPOPAUSE_M:
        temperature_k = T0_K - LAPSE_RATE * geopotential_m
        pressure_pa = P0_PA * (temperature_k / T0_K) ** _PRESSURE_EXPONENT
    else:
        temperature_k = _T11_K
        pressure_pa = _P11_PA * math.exp(-G0_MPS2 * (geopotential_m - H_TROPOPAUSE_M) / (R_AIR * _T11_K))
    return AirState(
        temperature_k=temperature_k,
        pressure_pa=pressure_pa,
        density_kgpm3=pressure_pa / (R_AIR * temperature_k),
        speed_of_sound_mps=math.sqrt(GAMMA * R_AIR * temperature_k),
    )


class StandardAtmosphere:
    """Two-layer standard atmosphere, 0-20 km geopotential altitude."""

    name = "standard"

    def air(self, altitude_msl_m: float, *, clamp: bool = False) -> AirState:
        """Properties at a geometric altitude above mean sea level (m).

        `clamp` is for intermediate integrator stage evaluations only: the altitude is limited to
        the model range instead of raising. Accepted states are always checked without it.
        """
        return standard_air_at_geopotential(_geopotential_in_range_m(altitude_msl_m, clamp))


class ConstantDensityAtmosphere:
    """Simplified reference atmosphere: every property is constant at all altitudes.

    Density comes from the explicit scenario parameter and has no default. Pressure, temperature
    and speed of sound are the fixed reference values p0, T0 and sqrt(gamma*R*T0).
    """

    name = "constant_density"

    def __init__(self, air_density_kgpm3: float) -> None:
        if isinstance(air_density_kgpm3, bool) or not isinstance(air_density_kgpm3, (int, float)):
            raise ValueError(f"air_density_kgpm3 must be a number, got {air_density_kgpm3!r}")
        if not math.isfinite(air_density_kgpm3) or air_density_kgpm3 <= 0.0:
            raise ValueError(f"air_density_kgpm3 must be finite and positive, got {air_density_kgpm3!r}")
        self.air_density_kgpm3 = float(air_density_kgpm3)
        self._air = AirState(T0_K, P0_PA, self.air_density_kgpm3, REFERENCE_SPEED_OF_SOUND_MPS)

    def air(self, altitude_msl_m: float, *, clamp: bool = False) -> AirState:
        _geopotential_in_range_m(altitude_msl_m, clamp)
        return self._air


class VacuumAtmosphere:
    """No air: density and pressure are zero, so there is no drag.

    Temperature and speed of sound keep the reference values T0 and sqrt(gamma*R*T0) so that the
    truth record and the Mach diagnostic stay finite.
    """

    name = "vacuum"
    _air = AirState(T0_K, 0.0, 0.0, REFERENCE_SPEED_OF_SOUND_MPS)

    def air(self, altitude_msl_m: float, *, clamp: bool = False) -> AirState:
        _geopotential_in_range_m(altitude_msl_m, clamp)
        return self._air
