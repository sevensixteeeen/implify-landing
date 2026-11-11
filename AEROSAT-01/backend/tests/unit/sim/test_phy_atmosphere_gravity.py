"""PHY-01, PHY-02, PHY-03 (TEST_PLAN section 4): atmosphere, gravity, geopotential altitude."""
import math

import pytest

from aerosat.core.constants import G0_MPS2, P0_PA, T0_K
from aerosat.core.faults import FaultCode, ModelRangeFault
from aerosat.sim.atmosphere import (
    ConstantDensityAtmosphere,
    StandardAtmosphere,
    VacuumAtmosphere,
    geometric_altitude_m,
    geopotential_altitude_m,
    standard_air_at_geopotential,
)
from aerosat.sim.gravity import constant_gravity_mps2, inverse_square_gravity_mps2

# Oracle for PHY-01: the ICAO Standard Atmosphere (ICAO Doc 7488, same model as ISO 2533:1975),
# tabulated against geopotential altitude H. Entered by hand; pressure converted from hPa to Pa.
#
# Verification status (2026-11-11): Doc 7488 itself was not available to check against. The rows
# were compared with secondary published tables instead:
#   - H = 0, 11 000, 20 000 m: agree with the ISA layer table (288.15 K, 101 325 Pa, 1.225;
#     216.65 K, 22 632 Pa, 0.3639; 216.65 K, 5 474.9 Pa, 0.0880) to the digits printed there.
#   - H = 1 000 m: 281.65 K, 89 874.6 Pa and 1.11164 kg/m^3 each appear in a published ISA table
#     (one table prints 89 874.4 Pa, a 2e-6 relative difference).
#   - H = 5 000 m: only the temperature (255.65 K) was confirmed. Tables found for pressure and
#     density at 5 km were indexed by geometric altitude, which is a different quantity.
# The remaining digits still need checking against the printed document.
#   H (m)     T (K)     p (Pa)      rho (kg/m^3)
ICAO_TABLE = [
    (0.0, 288.150, 101_325.0, 1.22500),
    (1_000.0, 281.650, 89_874.6, 1.11164),
    (5_000.0, 255.650, 54_019.9, 0.736116),
    (11_000.0, 216.650, 22_632.0, 0.363918),
    (20_000.0, 216.650, 5_474.89, 0.0880349),
]


@pytest.mark.parametrize(("h_m", "temperature_k", "pressure_pa", "density_kgpm3"), ICAO_TABLE)
def test_phy_01_matches_published_standard_atmosphere(
    h_m: float, temperature_k: float, pressure_pa: float, density_kgpm3: float
) -> None:
    air = standard_air_at_geopotential(h_m)
    assert air.temperature_k == pytest.approx(temperature_k, rel=1e-4)
    assert air.pressure_pa == pytest.approx(pressure_pa, rel=1e-4)
    assert air.density_kgpm3 == pytest.approx(density_kgpm3, rel=1e-4)


def test_phy_02_pressure_and_density_are_continuous_at_11_km() -> None:
    below = standard_air_at_geopotential(11_000.0)
    above = standard_air_at_geopotential(11_000.0 + 1e-6)
    assert abs(above.pressure_pa - below.pressure_pa) / below.pressure_pa <= 1e-9
    assert abs(above.density_kgpm3 - below.density_kgpm3) / below.density_kgpm3 <= 1e-9
    assert above.temperature_k == below.temperature_k


def test_phy_02_pressure_and_density_strictly_decrease_with_altitude() -> None:
    samples = [standard_air_at_geopotential(float(h)) for h in range(0, 20_001, 50)]
    for lower, upper in zip(samples, samples[1:]):
        assert upper.pressure_pa < lower.pressure_pa
        assert upper.density_kgpm3 < lower.density_kgpm3


@pytest.mark.parametrize("geopotential_m", [-0.001, -500.0, 20_000.001, 25_000.0])
def test_phy_02_model_range_fault_outside_0_to_20_km(geopotential_m: float) -> None:
    with pytest.raises(ModelRangeFault) as raised:
        standard_air_at_geopotential(geopotential_m)
    assert raised.value.code is FaultCode.MODEL_RANGE
    with pytest.raises(ModelRangeFault):
        StandardAtmosphere().air(geometric_altitude_m(geopotential_m))


def test_phy_02_range_limits_themselves_are_accepted() -> None:
    assert standard_air_at_geopotential(0.0).pressure_pa == P0_PA
    assert standard_air_at_geopotential(20_000.0).temperature_k == pytest.approx(216.65, abs=1e-9)


def test_phy_03_gravity_at_sea_level_is_exactly_g0() -> None:
    assert inverse_square_gravity_mps2(0.0) == G0_MPS2
    assert constant_gravity_mps2(12_345.0) == G0_MPS2


def test_phy_03_gravity_and_geopotential_altitude_at_10_km() -> None:
    # Hand calculation: g0 * (6356766 / 6366766)^2 = 9.775868 m/s^2; 6356766 * 10000 / 6366766 = 9984.293 m.
    assert inverse_square_gravity_mps2(10_000.0) == pytest.approx(9.775868, abs=1e-6)
    assert geopotential_altitude_m(10_000.0) == pytest.approx(9_984.293, abs=1e-3)


def test_geometric_and_geopotential_altitude_are_inverses() -> None:
    for h_m in (0.0, 1.0, 2_000.0, 15_000.0):
        assert geometric_altitude_m(geopotential_altitude_m(h_m)) == pytest.approx(h_m, abs=1e-6)


# --- Simplified reference atmospheres (ARCHITECTURE section 4.5, FR-SIM-10, FR-SCN-05). ---

def test_constant_density_atmosphere_holds_the_reference_values_at_every_altitude() -> None:
    atmosphere = ConstantDensityAtmosphere(1.225)
    for altitude_m in (0.0, 1_500.0, 19_000.0):
        air = atmosphere.air(altitude_m)
        assert air.density_kgpm3 == 1.225
        assert air.pressure_pa == 101_325.0
        assert air.temperature_k == 288.15
        assert air.speed_of_sound_mps == pytest.approx(340.3, abs=0.05)


@pytest.mark.parametrize("bad", [0.0, -1.0, math.nan, math.inf, None, "1.225", True])
def test_constant_density_rejects_invalid_density(bad: object) -> None:
    with pytest.raises(ValueError):
        ConstantDensityAtmosphere(bad)  # type: ignore[arg-type]


def test_simplified_atmospheres_keep_the_20_km_limit() -> None:
    too_high_m = geometric_altitude_m(20_000.5)
    for atmosphere in (ConstantDensityAtmosphere(1.225), VacuumAtmosphere()):
        with pytest.raises(ModelRangeFault):
            atmosphere.air(too_high_m)
        with pytest.raises(ModelRangeFault):
            atmosphere.air(-1.0)


def test_vacuum_has_no_air() -> None:
    air = VacuumAtmosphere().air(1_000.0)
    assert air.density_kgpm3 == 0.0
    assert air.pressure_pa == 0.0
    assert air.temperature_k == T0_K
