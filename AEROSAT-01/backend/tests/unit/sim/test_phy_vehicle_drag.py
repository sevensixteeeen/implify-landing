"""PHY-04, PHY-05 (TEST_PLAN section 4): drag, thrust table, impulse and mass."""
import math

import pytest

from aerosat.sim.dynamics import drag_force_n, specific_force_mps2
from aerosat.sim.simulator import SimulatorConfig
from aerosat.sim.vehicle import RollProfile, ThrustCurve, Vehicle


def test_phy_04_drag_magnitude_by_hand() -> None:
    # 0.5 * 1.225 * 0.005 * 100^2 = 30.625 N
    assert drag_force_n(1.225, 0.005, 100.0) == pytest.approx(30.625, abs=1e-9)


def test_phy_04_drag_opposes_motion_and_vanishes_at_rest() -> None:
    assert drag_force_n(1.225, 0.005, 100.0) > 0.0   # subtracted in (F_T - F_D)/m: decelerates ascent
    assert drag_force_n(1.225, 0.005, -100.0) < 0.0  # decelerates descent
    assert drag_force_n(1.225, 0.005, -100.0) == -drag_force_n(1.225, 0.005, 100.0)
    assert drag_force_n(1.225, 0.005, 0.0) == 0.0


def test_phy_05_ref01_total_impulse_is_4575_newton_seconds(ref01_config: SimulatorConfig) -> None:
    # Trapezoids: 75 + 4350 + 150 = 4575 N*s.
    curve = ref01_config.vehicle.thrust_curve
    assert curve.total_impulse_ns == pytest.approx(4575.0, rel=1e-12)
    assert curve.impulse_ns(0.1) == pytest.approx(75.0, rel=1e-12)
    assert curve.impulse_ns(3.0) == pytest.approx(4425.0, rel=1e-12)
    assert curve.impulse_ns(0.05) == pytest.approx(0.5 * 750.0 * 0.05, rel=1e-12)  # half-way up the ramp


def test_phy_05_ref01_mass_before_and_after_the_burn(ref01_config: SimulatorConfig) -> None:
    vehicle = ref01_config.vehicle
    assert vehicle.mass_kg(-1.0) == 20.0
    assert vehicle.mass_kg(0.0) == 20.0
    assert vehicle.mass_kg(3.2) == 15.0
    assert vehicle.mass_kg(100.0) == 15.0


def test_phy_05_mass_never_increases(ref01_config: SimulatorConfig) -> None:
    vehicle = ref01_config.vehicle
    masses = [vehicle.mass_kg(t_ms / 1000) for t_ms in range(-100, 3400)]
    assert all(later <= earlier for earlier, later in zip(masses, masses[1:]))
    assert masses[0] == 20.0 and masses[-1] == 15.0


def test_phy_05_thrust_is_zero_outside_the_table_and_interpolated_inside(ref01_config: SimulatorConfig) -> None:
    vehicle = ref01_config.vehicle
    assert vehicle.thrust_n(-0.001) == 0.0
    assert vehicle.thrust_n(3.2001) == 0.0
    assert vehicle.thrust_n(1000.0) == 0.0
    assert vehicle.thrust_n(0.0) == 0.0
    assert vehicle.thrust_n(0.05) == pytest.approx(750.0, rel=1e-12)
    assert vehicle.thrust_n(0.1) == 1500.0
    assert vehicle.thrust_n(2.0) == 1500.0
    assert vehicle.thrust_n(3.1) == pytest.approx(750.0, rel=1e-12)
    assert vehicle.thrust_n(3.2) == 0.0


def test_unpowered_vehicle_keeps_its_mass(unpowered_vehicle: Vehicle) -> None:
    assert unpowered_vehicle.thrust_n(1.0) == 0.0
    assert unpowered_vehicle.mass_kg(0.0) == 15.0
    assert unpowered_vehicle.mass_kg(50.0) == 15.0


def test_zero_propellant_is_valid_and_mass_validation_is_strict() -> None:
    assert Vehicle(dry_mass_kg=15.0, propellant_mass_kg=0.0, cd_area_m2=0.005).mass_kg(0.0) == 15.0
    for bad in (
        {"dry_mass_kg": 0.0}, {"dry_mass_kg": -1.0}, {"dry_mass_kg": math.nan}, {"dry_mass_kg": math.inf},
        {"dry_mass_kg": "15"}, {"dry_mass_kg": True},
        {"propellant_mass_kg": -0.001}, {"propellant_mass_kg": math.nan}, {"propellant_mass_kg": math.inf},
        {"cd_area_m2": -0.005}, {"cd_area_m2": math.nan},
    ):
        values = {"dry_mass_kg": 15.0, "propellant_mass_kg": 5.0, "cd_area_m2": 0.005} | bad
        with pytest.raises(ValueError):
            Vehicle(**values)


def test_thrust_breakpoints_must_be_integer_milliseconds_in_order() -> None:
    with pytest.raises(ValueError):
        ThrustCurve([(0, 0.0), (100.5, 1500.0)])  # type: ignore[list-item]
    with pytest.raises(ValueError):
        ThrustCurve([(100, 0.0), (100, 1500.0)])
    with pytest.raises(ValueError):
        Vehicle(dry_mass_kg=0.0, propellant_mass_kg=1.0, cd_area_m2=0.005)


def test_specific_force_is_acceleration_plus_gravity() -> None:
    assert specific_force_mps2(0.0, 9.80665) == 9.80665          # at rest on the ground: +g
    assert specific_force_mps2(-9.80665, 9.80665) == 0.0         # drag-free fall: 0


def test_roll_profile_is_interpolated_and_held(ref01_config: SimulatorConfig) -> None:
    # REF-01 (FR-SIM-11): 0 rad/s until 3.0 s, ramp to 0.5 rad/s at 6.2 s, then held. Kinematic only.
    profile = ref01_config.roll_profile
    assert profile.roll_rate_radps(1.0) == 0.0
    assert profile.roll_rate_radps(3.0) == 0.0
    assert profile.roll_rate_radps(4.6) == pytest.approx(0.25, rel=1e-12)
    assert profile.roll_rate_radps(6.2) == 0.5
    assert profile.roll_rate_radps(60.0) == 0.5
    # Angle by hand: triangle 0.5 * 3.2 s * 0.5 rad/s = 0.8 rad at 6.2 s, then 0.5 rad/s.
    assert profile.roll_angle_rad(3.0) == 0.0
    assert profile.roll_angle_rad(6.2) == pytest.approx(0.8, rel=1e-12)
    assert profile.roll_angle_rad(10.2) == pytest.approx(0.8 + 2.0, rel=1e-12)
    assert RollProfile(()).roll_rate_radps(5.0) == 0.0
    assert math.isfinite(profile.roll_angle_rad(600.0))
