"""F1-T01 to F1-T20 (docs/F1_DECISIONS.md section 6e.6): disagreement detector (Policy B), the
F1-D4 deployment restriction and the F1-D6 liftoff-anchored backup timer.

Oracles: hand-computed values from the formulas in section 6b/6e (F1-T01, F1-T02, F1-T14),
hand-built sample and estimate sequences (F1-T03 to F1-T12, F1-T17 to F1-T19), the simulator's
truth (F1-T15, F1-T16) and the source text (F1-T20).
"""
import ast
import copy
import dataclasses
import math
from pathlib import Path

import pytest

import aerosat.avionics.estimator as estimator_module
from aerosat.avionics.calibration import CalibrationResult
from aerosat.avionics.estimator import (
    _BARO_EXPONENT,
    PERSISTENCE,
    THRESHOLD,
    WINDOW,
    DisagreementFlag,
    DisagreementMonitor,
    FlagReason,
    baro_altitude_sigma_m,
    dead_reckoning_velocity_sigma_mps,
    disagreement_statistic,
)
from aerosat.avionics.flight_computer import FlightComputer, load_avionics_config
from aerosat.avionics.state_machine import FlightStateMachine, StateMachineConfig
from aerosat.core.constants import G0_MPS2, LAPSE_RATE_KPM, P0_PA, T0_K
from aerosat.core.faults import ConfigurationFault
from aerosat.core.types import DataQuality, EventSource, SensorSample, VehicleCommand
from aerosat.core.types import FlightEventType as Event
from aerosat.core.types import FlightState as State
from aerosat.sensors.suite import SensorSuite, suite_config_from_scenario
from aerosat.sim.simulator import Simulator

from .conftest import REF01, flight, ref01_avionics_config, ref01_simulator_config

SIGMA_P, SIGMA_A, KAPPA, PERIOD_S = 3.014, 0.0501, 0.007, 0.005
CAL = CalibrationResult(P0_PA, T0_K, 0.0, 200)
FLAG = DisagreementFlag(0, FlagReason.DISAGREEMENT, 9.0)


def monitor() -> DisagreementMonitor:
    return DisagreementMonitor(SIGMA_P, SIGMA_A, KAPPA, CAL, PERIOD_S)


def feed_ramp(m: DisagreementMonitor, count: int, slope_m: float, *, start: int = 0, t_s: float = 10.0,
              v: float = 50.0) -> list[float | None]:
    """D_k = slope * k at the pad pressure: every defined dD equals 10 * slope."""
    return [m.barometer_sample((start + k) * 20, slope_m * (start + k), P0_PA, 0.0, t_s, v) for k in range(count)]


def expected_spread(t_s: float, v: float) -> float:
    sigma_h = SIGMA_P * T0_K / LAPSE_RATE_KPM * _BARO_EXPONENT / P0_PA
    sigma_v = math.sqrt((SIGMA_A * t_s / math.sqrt(200)) ** 2 + SIGMA_A ** 2 * PERIOD_S * t_s)
    return math.sqrt(0.4 * sigma_h ** 2 + (0.2 * sigma_v) ** 2 + (KAPPA * abs(v) * 0.2) ** 2)


# ---------------------------------------------------------------- Policy B: statistic and flag

def test_f1_t01_q_d_matches_the_hand_computed_value() -> None:
    m = monitor()
    qs = feed_ramp(m, WINDOW, 0.05)
    assert qs[:WINDOW - 1] == [None] * (WINDOW - 1)
    assert qs[-1] == pytest.approx(0.5 / expected_spread(10.0, 50.0), rel=1e-12)
    # The sqrt(2/5) factor: with no dead-reckoning and no speed term the spread is sqrt(2/5) * sigma_h.
    sigma_h = baro_altitude_sigma_m(P0_PA, SIGMA_P, CAL)
    assert sigma_h == pytest.approx(SIGMA_P * T0_K / LAPSE_RATE_KPM * _BARO_EXPONENT / P0_PA, rel=1e-12)
    assert disagreement_statistic(1.0, sigma_h, 0.0, KAPPA, 0.0, 0.2) == pytest.approx(1.0 / (math.sqrt(2 / 5) * sigma_h))
    assert dead_reckoning_velocity_sigma_mps(0.0, SIGMA_A, 200, PERIOD_S) == 0.0


def test_f1_t02_speed_term_raises_the_allowed_disagreement_and_never_switches_off() -> None:
    sigma_h = baro_altitude_sigma_m(P0_PA, SIGMA_P, CAL)
    speeds = [0.0, 50.0, 100.0, 200.0, 1000.0]
    qs = [disagreement_statistic(2.0, sigma_h, 0.01, KAPPA, v, 0.2) for v in speeds]
    assert all(a > b for a, b in zip(qs, qs[1:]))
    allowed = [THRESHOLD * 2.0 / q for q in qs]            # |dD| at the threshold, m
    assert all(a < b for a, b in zip(allowed, allowed[1:]))
    big = disagreement_statistic(1000.0, sigma_h, 0.01, KAPPA, 1000.0, 0.2)
    assert math.isfinite(big) and big > THRESHOLD           # still evaluated, and still flags a large error


def test_f1_t03_threshold() -> None:
    below, above = monitor(), monitor()
    for tick in range(10):
        below.register(tick, THRESHOLD)                     # equal is not above
        below.register(tick, THRESHOLD * (1 - 1e-9))
    assert below.flag is None
    for tick in range(PERSISTENCE):
        above.register(tick, THRESHOLD * (1 + 1e-9))
    assert above.flag == DisagreementFlag(PERSISTENCE - 1, FlagReason.DISAGREEMENT, THRESHOLD * (1 + 1e-9))


def test_f1_t04_persistence() -> None:
    hi, lo = THRESHOLD + 1.0, THRESHOLD - 1.0
    m = monitor()
    for tick, q in enumerate([hi, hi, lo, hi, hi, None, hi, hi]):
        m.register(tick, q)
    assert m.flag is None
    m.register(8, hi)
    assert m.flag is not None and m.flag.tick == 8


def test_f1_t05_availability_barometer() -> None:
    m = monitor()
    assert feed_ramp(m, WINDOW - 1, 100.0) == [None] * (WINDOW - 1)
    m.barometer_rejected()                                  # breaks contiguity
    assert feed_ramp(m, WINDOW - 1, 100.0, start=20) == [None] * (WINDOW - 1)
    assert m.flag is None
    assert feed_ramp(m, 1, 100.0, start=34)[0] is not None


def pad_samples(tick: int, accel_saturated: bool = False) -> list[SensorSample]:
    s = lambda sid, ch, v, unit, sat=False: SensorSample(tick, tick / 1000, sid, ch, v, unit, sat)
    out = [s("accel", "z", G0_MPS2, "m/s^2", accel_saturated), s("gyro", "z", 0.0, "rad/s")]
    if tick % 20 == 0:
        out.append(s("baro", "pressure", P0_PA, "Pa"))
    if tick % 100 == 0:
        out.append(s("temp", "temperature", T0_K, "K"))
    return out


def test_f1_t06_availability_accelerometer_uncheckable() -> None:
    fc = FlightComputer(ref01_avionics_config())
    for t in range(0, 2000, 5):
        fc.tick(t, pad_samples(t))
    for t in range(2000, 2100, 5):                          # last valid sample at 1995: age 100 ms at 2095
        fc.tick(t, pad_samples(t, accel_saturated=True))
    assert fc.disagreement_flag is None
    out = fc.tick(2100, pad_samples(2100, accel_saturated=True))
    assert fc.disagreement_flag == DisagreementFlag(2100, FlagReason.UNCHECKABLE, None)
    assert out.estimate.quality is DataQuality.UNRELIABLE and out.disagreement is fc.disagreement_flag


def test_f1_t07_flag_is_latched() -> None:
    fc = FlightComputer(ref01_avionics_config())
    for t in range(0, 2000, 5):
        fc.tick(t, pad_samples(t))
    for t in range(2000, 2105, 5):
        fc.tick(t, pad_samples(t, accel_saturated=True))
    raised = fc.disagreement_flag
    for t in range(2105, 6000, 5):                          # the accelerometer recovers and both sensors agree
        out = fc.tick(t, pad_samples(t))
    assert fc.disagreement_flag == raised and out.estimate.quality is DataQuality.UNRELIABLE
    m = monitor()
    for tick in range(PERSISTENCE):
        m.register(tick, THRESHOLD + 1)
    first = m.flag
    for tick in range(PERSISTENCE, 200):
        m.register(tick, 0.0)
    assert m.flag == first


# ---------------------------------------------------------------- F1-D4 restriction (state machine)

def machine(**overrides) -> FlightStateMachine:
    values = dict(
        liftoff_specific_force_mps2=15.0, debounce_samples=10, apogee_lockout_ticks=1000,
        touchdown_window_ticks=2000, touchdown_altitude_range_m=3.0, touchdown_speed_mps=2.0,
        period_ticks=5, drag_device_enabled=True, deploy_delay_ticks=1000, backup_deployment_ticks=20_000,
        burnout_fallback_specific_force_mps2=1.0, burnout_fallback_samples=20,
        touchdown_fallback_window_ticks=2800, touchdown_fallback_acceleration_mps2=2.0,
        deploy_confirm_baro_descent_m=10.0, deploy_confirm_samples=5,
    )
    values.update(overrides)
    return FlightStateMachine(StateMachineConfig(**values))


class Driver:
    def __init__(self, fsm: FlightStateMachine) -> None:
        self.fsm, self.tick, self.events, self.commands = fsm, 0, [], []

    def feed(self, count: int, *, force: float = 9.8, altitude: float = 0.0, velocity: float = 0.0,
             quality: DataQuality = DataQuality.NOMINAL, flag: DisagreementFlag | None = None) -> None:
        for _ in range(count):
            events, commands = self.fsm.update(
                self.tick, specific_force_mps2=force, altitude_m=altitude, velocity_mps=velocity,
                baro_altitude_m=altitude, acceleration_mps2=9.0, estimate_quality=quality,
                accel_rejected=False, baro_sample_m=altitude, baro_rejected=False, disagreement=flag,
            )
            self.events += events
            self.commands += [(self.tick, c) for c in commands]
            self.tick += 5

    def until(self, tick: int, **kw) -> None:
        self.feed((tick - self.tick) // 5 + 1, **kw)

    def event(self, kind: Event):
        return [e for e in self.events if e.event is kind]


def to_apogee(d: Driver, *, flag: DisagreementFlag | None = None, quality: DataQuality = DataQuality.NOMINAL) -> None:
    """LIFTOFF at 145, BURNOUT at 445, coast to 1500 m, APOGEE at 1995."""
    d.feed(20)
    d.feed(60, force=30.0, velocity=100.0)
    d.feed(10, force=-2.0, velocity=200.0)
    d.feed(300, force=-2.0, velocity=50.0, altitude=1500.0, flag=flag, quality=quality)
    d.feed(10, force=0.0, velocity=-0.5, altitude=1500.0, flag=flag, quality=quality)
    assert [(e.event, e.tick) for e in d.events] == [(Event.LIFTOFF, 145), (Event.BURNOUT, 445), (Event.APOGEE, 1995)]


TIMER_TICK = 445 + 20_000


def test_f1_t08_flag_before_apogee_leaves_only_the_backup_timer() -> None:
    d = Driver(machine())
    to_apogee(d, flag=FLAG)
    (apogee,) = d.event(Event.APOGEE)
    assert apogee.quality is DataQuality.UNRELIABLE          # forced by the flag, whatever the estimate said
    d.until(TIMER_TICK, velocity=-12.0, altitude=1400.0, flag=FLAG)   # 100 m below the peak: path 2 conditions met
    assert d.commands == [(TIMER_TICK, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    (dep,) = d.event(Event.DRAG_DEVICE_DEPLOYED)
    assert (dep.source, dep.quality) == (EventSource.BACKUP_TIMER, DataQuality.UNRELIABLE)
    assert "DISAGREEMENT flag" in dep.reason


def test_f1_t09_flag_after_a_reliable_apogee_restores_the_timer() -> None:
    d = Driver(machine())
    to_apogee(d)
    (apogee,) = d.event(Event.APOGEE)
    assert apogee.quality is DataQuality.NOMINAL
    d.until(TIMER_TICK, velocity=-12.0, altitude=1400.0, flag=FLAG)   # flag raised during the deployment delay
    assert d.commands == [(TIMER_TICK, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert d.event(Event.APOGEE) == [apogee]                         # never rewritten
    assert d.event(Event.DRAG_DEVICE_DEPLOYED)[0].source is EventSource.BACKUP_TIMER


def test_f1_t10_flag_on_the_command_tick_blocks_the_command() -> None:
    d = Driver(machine())
    to_apogee(d)
    d.until(2990, velocity=-12.0, altitude=1400.0)
    d.feed(1, velocity=-12.0, altitude=1400.0, flag=FLAG)            # tick 2995 = apogee + delay
    assert d.commands == []
    d.until(TIMER_TICK, velocity=-12.0, altitude=1400.0, flag=FLAG)
    assert d.commands == [(TIMER_TICK, VehicleCommand.DEPLOY_DRAG_DEVICE)]


def test_f1_t11_flag_after_the_command_changes_nothing() -> None:
    d = Driver(machine())
    to_apogee(d)
    d.until(2995, velocity=-12.0, altitude=1400.0)
    d.until(TIMER_TICK + 1000, velocity=-12.0, altitude=1400.0, flag=FLAG)
    assert d.commands == [(2995, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert d.event(Event.DRAG_DEVICE_DEPLOYED)[0].source is EventSource.DETECTED


def test_f1_t12_barometric_confirmation_is_blocked_while_flagged() -> None:
    control = Driver(machine())
    to_apogee(control, quality=DataQuality.UNRELIABLE)
    control.until(3500, velocity=-12.0, altitude=1400.0, quality=DataQuality.UNRELIABLE)
    assert control.event(Event.DRAG_DEVICE_DEPLOYED)[0].source is EventSource.BAROMETRIC_CONFIRMATION
    d = Driver(machine())
    to_apogee(d, quality=DataQuality.UNRELIABLE)
    d.until(TIMER_TICK - 5, velocity=-12.0, altitude=1400.0, quality=DataQuality.UNRELIABLE, flag=FLAG)
    assert d.commands == []


def test_f1_t13_unflagged_behaviour_is_unchanged() -> None:
    """The same sequence with the argument omitted (pre-F1 callers) and with disagreement=None."""
    plan = [(20, 9.8, 0.0, 0.0), (60, 30.0, 100.0, 0.0), (10, -2.0, 200.0, 0.0), (300, -2.0, 50.0, 1500.0),
            (10, 0.0, -0.5, 1500.0), (400, 0.0, -12.0, 1400.0)]
    runs = []
    for pass_argument in (False, True):
        fsm, tick, events, commands = machine(liftoff_backup_ticks=23_200), 0, [], []
        for count, force, velocity, altitude in plan:
            for _ in range(count):
                kw = dict(specific_force_mps2=force, altitude_m=altitude, velocity_mps=velocity,
                          baro_altitude_m=altitude, acceleration_mps2=9.0, estimate_quality=DataQuality.NOMINAL,
                          accel_rejected=False, baro_sample_m=altitude, baro_rejected=False)
                if pass_argument:
                    kw["disagreement"] = None
                e, c = fsm.update(tick, **kw)
                events += [(x.event, x.tick, x.source, x.quality) for x in e]
                commands += [(tick, x) for x in c]
                tick += 5
        runs.append((events, commands))
    assert runs[0] == runs[1]
    assert runs[0][1] == [(2995, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    record = flight(0)
    assert record.disagreement is None
    assert [e.source for e in record.events] == [EventSource.DETECTED] * len(record.events)


# ---------------------------------------------------------------- F1-D6 liftoff-anchored timer

def scenario_variant(**changes) -> dict:
    s = copy.deepcopy(REF01)
    for path, value in changes.items():
        node = s
        *parents, leaf = path.split(".")
        for key in parents:
            node = node[key]
        node[leaf] = value
    return s


LOW_THRUST = [{"t_ms": 0, "thrust_n": 0.0}, {"t_ms": 100, "thrust_n": 280.0},
              {"t_ms": 16000, "thrust_n": 280.0}, {"t_ms": 16200, "thrust_n": 0.0}]


@pytest.mark.parametrize("changes, expected", [
    ({}, 23_200),
    ({"avionics.expected_coast_duration_ms": 23665, "avionics.backup_deployment_delay_ms": 26000}, 29_200),
    ({"vehicle.thrust_curve": LOW_THRUST, "avionics.expected_coast_duration_ms": 8956,
      "avionics.backup_deployment_delay_ms": 11000}, 27_200),
    ({"avionics.expected_coast_duration_ms": 19477, "avionics.backup_deployment_delay_ms": 21477}, 24_677),
    ({"avionics.expected_coast_duration_ms": 20763, "avionics.backup_deployment_delay_ms": 22763}, 25_963),
], ids=["ref01", "zero_drag", "low_thrust", "high4k", "high8k"])
def test_f1_t14_liftoff_timer_duration_is_derived_from_the_scenario(changes: dict, expected: int) -> None:
    assert load_avionics_config(scenario_variant(**changes)).liftoff_backup_delay_ms == expected


def test_f1_t14_configuration_values_are_validated() -> None:
    assert load_avionics_config(REF01).detector_kappa == 0.007
    with pytest.raises(ConfigurationFault, match="vehicle.thrust_curve"):
        load_avionics_config(scenario_variant(**{"vehicle.thrust_curve": []}))
    for name in ("detector_sigma_p_pa", "detector_sigma_a_mps2", "detector_kappa"):
        for bad in (0.0, -1.0, math.nan, "x"):
            with pytest.raises(ConfigurationFault, match=f"avionics.{name}"):
                load_avionics_config(scenario_variant(**{f"avionics.{name}": bad}))


def test_f1_t15_liftoff_timer_deploys_when_burnout_is_never_detected() -> None:
    d = Driver(machine(liftoff_backup_ticks=5_000))
    d.feed(20)
    d.until(145 + 5_000, force=30.0, velocity=100.0, altitude=800.0)    # powered for ever: no BURNOUT
    assert d.commands == [(5_145, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    (apogee,) = d.event(Event.APOGEE)
    assert (apogee.tick, apogee.source, apogee.quality) == (5_145, EventSource.BACKUP_TIMER, DataQuality.UNRELIABLE)
    assert d.fsm.state is State.DESCENT and d.event(Event.BURNOUT) == []


def test_f1_t15_closed_loop_accelerometer_stuck_at_80() -> None:
    """REF-01 seed 0, accelerometer stuck at 80 m/s^2 from 5.2 s (1 s before true burnout)."""
    sim = Simulator(ref01_simulator_config())
    suite = SensorSuite(suite_config_from_scenario(REF01["sensors"]), master_seed=0)
    fc = FlightComputer(ref01_avionics_config())
    events, commands, state, pending = [], [], sim.state, ()
    while state.tick < 30_000:
        samples = suite.sample(state)
        if state.tick >= 5_200:
            samples = [dataclasses.replace(s, value=80.0) if (s.sensor_id, s.channel) == ("accel", "z") else s
                       for s in samples]
        if state.tick % 5 == 0:
            out = fc.tick(state.tick, samples)
            events += out.events
            commands += [(state.tick, c, state.velocity_mps) for c in out.commands]
            pending = out.commands
        state = sim.step(pending)
        pending = ()
    liftoff = next(e for e in events if e.event is Event.LIFTOFF)
    assert [e for e in events if e.event is Event.BURNOUT] == []
    assert fc.disagreement_flag is not None and fc.disagreement_flag.reason is FlagReason.DISAGREEMENT
    assert [(t, c) for t, c, _ in commands] == [(liftoff.tick + 23_200, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert commands[0][2] < 0.0                                         # after true apogee
    apogee = next(e for e in events if e.event is Event.APOGEE)
    assert apogee.source is EventSource.BACKUP_TIMER


@pytest.mark.parametrize("seed, ideal", [(0, True)] + [(seed, False) for seed in range(10)])
def test_f1_t16_liftoff_timer_never_acts_first_in_a_nominal_flight(seed: int, ideal: bool) -> None:
    record = flight(seed, ideal)
    (apogee,) = record.detected(Event.APOGEE)
    (deployed,) = record.detected(Event.DRAG_DEVICE_DEPLOYED)
    assert record.disagreement is None
    assert (deployed.source, deployed.tick) == (EventSource.DETECTED, apogee.tick + 1000)


def test_f1_t17_two_timers_on_one_tick_give_one_command() -> None:
    d = Driver(machine(liftoff_backup_ticks=TIMER_TICK - 145))          # both expire at TIMER_TICK
    d.feed(20)
    d.feed(60, force=30.0, velocity=100.0)
    d.feed(10, force=-2.0, velocity=200.0)
    d.until(TIMER_TICK + 500, force=-2.0, velocity=50.0, altitude=1500.0, flag=FLAG)
    assert d.commands == [(TIMER_TICK, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    (dep,) = d.event(Event.DRAG_DEVICE_DEPLOYED)
    assert dep.source is EventSource.BACKUP_TIMER and "after detected BURNOUT" in dep.reason


def test_f1_t18_liftoff_never_detected_means_no_liftoff_timer() -> None:
    d = Driver(machine(liftoff_backup_ticks=1_000))
    d.feed(2_000)
    assert d.commands == [] and d.fsm.state is State.PAD_IDLE


def test_f1_t19_drag_device_disabled() -> None:
    d = Driver(machine(drag_device_enabled=False, liftoff_backup_ticks=TIMER_TICK - 145))
    to_apogee(d, flag=FLAG)
    d.until(TIMER_TICK + 500, velocity=-12.0, altitude=1400.0, flag=FLAG)
    assert d.commands == [] and d.event(Event.DRAG_DEVICE_DEPLOYED) == []
    assert d.fsm._timer_fired and d.fsm._liftoff_timer_fired and d.fsm.deployment_decided


def test_f1_t20_detector_is_pure_and_reads_no_truth() -> None:
    source = Path(estimator_module.__file__).read_text(encoding="utf-8")
    imported = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.ImportFrom):
            imported.add(node.module or "")
        elif isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
    assert not any(name.startswith(("aerosat.sim", "aerosat.sensors")) for name in imported), imported
    names = {n.id for n in ast.walk(ast.parse(source)) if isinstance(n, ast.Name)}
    names |= {n.attr for n in ast.walk(ast.parse(source)) if isinstance(n, ast.Attribute)}
    assert not any("truth" in name.lower() for name in names), names
