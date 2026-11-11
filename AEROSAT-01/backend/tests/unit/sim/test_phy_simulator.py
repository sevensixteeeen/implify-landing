"""PHY-11 to PHY-16 (TEST_PLAN section 4): ground constraints, determinism, time base, validity, commands."""
import math
from dataclasses import replace

import pytest

from aerosat.core.faults import FaultCode, ModelRangeFault
from aerosat.core.types import VehicleCommand
from aerosat.sim.atmosphere import VacuumAtmosphere
from aerosat.sim.gravity import constant_gravity_mps2, inverse_square_gravity_mps2
from aerosat.sim.simulator import DragDevice, Simulator, SimulatorConfig
from aerosat.sim.vehicle import ThrustCurve, Vehicle

G = 9.80665


def test_phy_11_pad_hold_when_thrust_is_below_weight() -> None:
    """100 N of thrust on a 20 kg vehicle (weight about 196 N) for 5 s: it must not move."""
    vehicle = Vehicle(dry_mass_kg=15.0, propellant_mass_kg=5.0, cd_area_m2=0.005,
                      thrust_curve=ThrustCurve([(0, 100.0), (5000, 100.0)]))
    sim = Simulator(SimulatorConfig(vehicle=vehicle, pad_altitude_msl_m=500.0))
    expected_specific_force = inverse_square_gravity_mps2(500.0)
    for _ in range(5_000):
        state = sim.step()
        assert state.altitude_msl_m == 500.0
        assert state.altitude_agl_m == 0.0
        assert state.velocity_mps == 0.0
        assert state.acceleration_mps2 == 0.0
        assert state.specific_force_mps2 == expected_specific_force
        assert state.on_ground
    assert state.thrust_n == 100.0
    assert sim.truth_events.liftoff_tick is None


def test_phy_11_ref01_leaves_the_pad_when_thrust_exceeds_weight(ref01_config: SimulatorConfig) -> None:
    """Hand calculation: thrust ramps at 15 000 N/s and passes the 196.133 N weight 13.08 ms after
    ignition (3000 ms), so the first tick with h > h_pad is 3014."""
    sim = Simulator(ref01_config)
    for _ in range(3_013):
        state = sim.step()
        assert state.altitude_agl_m == 0.0 and state.velocity_mps == 0.0 and state.on_ground
        assert state.specific_force_mps2 == G  # pad at sea level: exactly g0
    state = sim.step()
    assert state.tick == 3_014
    assert state.altitude_agl_m > 0.0 and state.velocity_mps > 0.0 and not state.on_ground
    assert sim.truth_events.liftoff_tick == 3_014


def test_phy_12_touchdown_after_a_vacuum_drop_from_100_m(unpowered_vehicle: Vehicle) -> None:
    """Oracle: t = sqrt(2h/g), v = -sqrt(2gh)."""
    sim = Simulator(SimulatorConfig(
        vehicle=unpowered_vehicle, atmosphere=VacuumAtmosphere(), gravity_mps2=constant_gravity_mps2,
        initial_altitude_agl_m=100.0, post_touchdown_hold_ms=5000,
    ))
    while sim.truth_events.touchdown_tick is None:
        sim.step()
    events = sim.truth_events
    assert abs(events.touchdown_time_s - math.sqrt(2 * 100.0 / G)) <= 1e-3
    assert abs(events.impact_velocity_mps - (-math.sqrt(2 * G * 100.0))) <= 1e-3

    touchdown_tick = events.touchdown_tick
    assert not sim.finished
    while not sim.finished:
        state = sim.step()
        assert state.altitude_agl_m == 0.0
        assert state.velocity_mps == 0.0
        assert state.acceleration_mps2 == 0.0
        assert state.specific_force_mps2 == G
        assert state.on_ground
    assert sim.tick == touchdown_tick + 5_000  # post-touchdown hold


def test_phy_13_truth_is_bit_identical_between_two_runs(ref01_config: SimulatorConfig) -> None:
    def run() -> list:
        sim = Simulator(ref01_config)
        return [sim.state] + [sim.step() for _ in range(12_000)]

    first, second = run(), run()
    assert first == second
    assert first[-1].altitude_agl_m > 100.0  # the comparison covered real flight, not just the pad


@pytest.mark.skip(reason="PHY-13 seed part: needs the mission runner with sensors and avionics (build step 4)")
def test_phy_13_truth_identical_across_seeds_with_drag_device_disabled() -> None:
    raise NotImplementedError


def test_phy_14_simulator_time_is_tick_over_1000(ref01_config: SimulatorConfig) -> None:
    sim = Simulator(ref01_config)
    assert sim.state.tick == 0 and sim.state.t_sim_s == 0.0
    for _ in range(2_500):
        state = sim.step()
        assert state.t_sim_s == state.tick / 1000
    assert state.tick == 2_500 and state.t_sim_s == 2.5


def test_phy_15_validity_warning_when_mach_exceeds_0_8(ref01_config: SimulatorConfig) -> None:
    """Constructed case: REF-01 with twice the thrust goes supersonic. One warning per excursion."""
    strong = replace(ref01_config.vehicle, thrust_curve=ThrustCurve([(0, 0.0), (100, 3000.0), (3000, 3000.0), (3200, 0.0)]))
    sim = Simulator(replace(ref01_config, vehicle=strong))
    previous = sim.state
    while not sim.warnings:
        assert sim.tick < 7_000, "no VALIDITY warning was recorded"
        previous, state = sim.state, sim.step()
    warning = sim.warnings[0]
    assert warning.code is FaultCode.VALIDITY
    assert warning.tick == state.tick
    assert state.mach > 0.8 >= previous.mach
    for _ in range(200):  # still above the limit: no duplicate warnings
        assert sim.step().mach > 0.8
    assert len(sim.warnings) == 1


def test_phy_15_ref01_itself_records_no_validity_warning(ref01_config: SimulatorConfig) -> None:
    sim = Simulator(ref01_config)
    peak = max(sim.step().mach for _ in range(8_000))
    assert sim.warnings == ()
    assert 0.5 < peak < 0.8  # ARCHITECTURE 4.12 expects roughly 0.66


def test_phy_15_run_stops_with_model_range_above_20_km(unpowered_vehicle: Vehicle) -> None:
    """Constructed case: start at 19 900 m moving up at 300 m/s in the standard atmosphere."""
    sim = Simulator(SimulatorConfig(vehicle=unpowered_vehicle, initial_altitude_agl_m=19_900.0,
                                    initial_velocity_mps=300.0))
    assert sim.warnings and sim.warnings[0].tick == 0  # already above Mach 0.8 at the initial state
    with pytest.raises(ModelRangeFault) as raised:
        for _ in range(5_000):
            sim.step()
    assert raised.value.code is FaultCode.MODEL_RANGE
    assert 19_900.0 < sim.state.altitude_agl_m <= 20_063.2  # last accepted state is inside the range


def test_phy_16_drag_area_changes_exactly_one_tick_after_the_command(ref01_config: SimulatorConfig) -> None:
    sim = Simulator(ref01_config)
    for _ in range(5_000):  # mid-boost, moving upward
        sim.step()

    def effective_cd_area(state) -> float:
        return state.drag_n / (0.5 * state.density_kgpm3 * state.velocity_mps * abs(state.velocity_mps))

    before = sim.step()                                        # tick n: command issued after this state
    after = sim.step([VehicleCommand.DEPLOY_DRAG_DEVICE])      # tick n + 1
    assert after.tick == before.tick + 1
    assert effective_cd_area(before) == pytest.approx(0.005, rel=1e-12)
    assert effective_cd_area(after) == pytest.approx(1.5, rel=1e-12)
    assert sim.cd_area_m2 == 1.5
    sim.step([VehicleCommand.DEPLOY_DRAG_DEVICE])              # a repeated command changes nothing
    assert sim.cd_area_m2 == 1.5


def test_phy_16_command_has_no_effect_when_the_drag_device_is_disabled(ref01_config: SimulatorConfig) -> None:
    sim = Simulator(replace(ref01_config, drag_device=DragDevice(enabled=False, cd_area_m2=1.5)))
    for _ in range(4_000):
        sim.step()
    sim.step([VehicleCommand.DEPLOY_DRAG_DEVICE])
    assert sim.cd_area_m2 == 0.005


def test_euler_integrator_is_selectable(ref01_config: SimulatorConfig) -> None:
    """FR-SIM-04: explicit Euler can be selected and gives a slightly different trajectory."""
    rk4, euler = Simulator(ref01_config), Simulator(replace(ref01_config, integrator="euler"))
    for _ in range(6_000):
        a, b = rk4.step(), euler.step()
    assert a.altitude_agl_m != b.altitude_agl_m
    assert a.altitude_agl_m == pytest.approx(b.altitude_agl_m, rel=1e-3)
    with pytest.raises(ValueError):
        replace(ref01_config, integrator="rk45")
