"""Closed-loop flight harness for the flight-event (EVT) tests.

Wires simulator -> sensors -> avionics in the fixed per-tick order of ARCHITECTURE section 7.
The mission runner (build step 4) will do this in production; the tests need it earlier.
The avionics only ever receives sensor samples and its tick.
"""
import copy
import functools
import json
from dataclasses import dataclass, field, replace
from pathlib import Path

import pytest

from aerosat.avionics.flight_computer import AvionicsConfig, FlightComputer, avionics_config_from_scenario
from aerosat.core.types import AvionicsEstimate, FlightEvent, FlightEventType, VehicleCommand
from aerosat.sensors.suite import SensorSuite, suite_config_from_scenario
from aerosat.sim.simulator import DragDevice, Simulator, SimulatorConfig, TruthEvents
from aerosat.sim.vehicle import RollProfile, ThrustCurve, Vehicle

ROOT = Path(__file__).resolve().parents[4]
REF01 = json.loads((ROOT / "scenarios" / "ref-01.json").read_text(encoding="utf-8"))
AVIONICS_PERIOD_TICKS = 5


def ref01_avionics_config(**overrides) -> AvionicsConfig:
    return replace(avionics_config_from_scenario(REF01["avionics"], REF01["vehicle"]["drag_device"]), **overrides)


def ref01_simulator_config() -> SimulatorConfig:
    vehicle = REF01["vehicle"]
    return SimulatorConfig(
        vehicle=Vehicle(vehicle["dry_mass_kg"], vehicle["propellant_mass_kg"],
                        vehicle["drag_coefficient"] * vehicle["reference_area_m2"],
                        ThrustCurve([(p["t_ms"], p["thrust_n"]) for p in vehicle["thrust_curve"]])),
        t_ignition_ms=REF01["timing"]["t_ignition_ms"],
        pad_altitude_msl_m=REF01["environment"]["pad_altitude_msl_m"],
        drag_device=DragDevice(vehicle["drag_device"]["enabled"], vehicle["drag_device"]["cd_area_m2"]),
        roll_profile=RollProfile([(p["t_ms"], p["roll_rate_radps"]) for p in vehicle["roll_profile"]]),
        post_touchdown_hold_ms=REF01["timing"]["post_touchdown_hold_ms"],
        max_duration_ms=REF01["timing"]["max_duration_ms"],
    )


@dataclass
class FlightRecord:
    """What one closed-loop run produced. Truth is kept only for comparison in tests."""

    truth_events: TruthEvents
    events: list[FlightEvent] = field(default_factory=list)
    commands: list[tuple[int, VehicleCommand]] = field(default_factory=list)
    ascent_errors: list[tuple[float, float]] = field(default_factory=list)
    """(altitude error m, velocity error m/s) at each avionics tick from true liftoff to true apogee."""
    frame_ticks: list[int] = field(default_factory=list)
    drag_area_change_tick: int | None = None
    final_tick: int = 0
    last_estimate: AvionicsEstimate | None = None

    def detected(self, event: FlightEventType) -> list[FlightEvent]:
        return [e for e in self.events if e.event is event]

    def detection_time_s(self, event: FlightEventType) -> float:
        (found,) = self.detected(event)
        return found.t_sim_s


def run_flight(seed: int, *, ideal: bool = False, pressure_spike: tuple[int, float] | None = None,
               stop_tick: int | None = None) -> FlightRecord:
    """Fly REF-01 once. `pressure_spike` = (tick, added Pa) corrupts one barometer sample."""
    sensors = copy.deepcopy(REF01["sensors"])
    sensors["ideal"] = ideal
    sim = Simulator(ref01_simulator_config())
    suite = SensorSuite(suite_config_from_scenario(sensors), master_seed=seed)
    computer = FlightComputer(ref01_avionics_config())
    record = FlightRecord(truth_events=sim.truth_events)

    state = sim.state
    pending: tuple[VehicleCommand, ...] = ()
    estimates: list[tuple[int, float, float, float, float]] = []
    initial_area = sim.cd_area_m2
    while True:
        samples = suite.sample(state)
        if pressure_spike is not None and state.tick == pressure_spike[0]:
            samples = [replace(s, value=s.value + pressure_spike[1]) if s.sensor_id == "baro" else s for s in samples]
        if state.tick % AVIONICS_PERIOD_TICKS == 0:
            output = computer.tick(state.tick, samples)
            record.events.extend(output.events)
            record.commands.extend((state.tick, command) for command in output.commands)
            record.last_estimate = output.estimate
            if output.frame is not None:
                record.frame_ticks.append(state.tick)
            if not state.on_ground and sim.truth_events.touchdown_tick is None:
                estimates.append((state.tick, output.estimate.altitude_agl_m, state.altitude_agl_m,
                                  output.estimate.velocity_mps, state.velocity_mps))
            pending = output.commands
        else:
            assert not samples, "a sensor sampled between avionics ticks"
        if sim.finished or (stop_tick is not None and state.tick >= stop_tick):
            break
        state = sim.step(pending)
        pending = ()
        if record.drag_area_change_tick is None and sim.cd_area_m2 != initial_area:
            record.drag_area_change_tick = state.tick

    record.truth_events = sim.truth_events
    record.final_tick = state.tick
    apogee_tick = sim.truth_events.apogee_tick
    if apogee_tick is not None:
        record.ascent_errors = [(h - true_h, v - true_v) for tick, h, true_h, v, true_v in estimates if tick <= apogee_tick]
    return record


@functools.lru_cache(maxsize=None)
def flight(seed: int, ideal: bool = False) -> FlightRecord:
    """A complete REF-01 flight, cached so several tests can inspect the same run."""
    return run_flight(seed, ideal=ideal)


@pytest.fixture(scope="session")
def ideal_flight() -> FlightRecord:
    return flight(0, True)
