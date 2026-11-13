"""EVT-01 to EVT-12 (TEST_PLAN section 5): flight-event detection and state estimation.

Oracles: hand-built sample sequences (EVT-01 to EVT-03), the truth events recorded by the
simulator (EVT-05, EVT-06, EVT-08, EVT-12), known injected values (EVT-09), and the atmosphere
model (EVT-11). Detection tolerances are the ones fixed in TEST_PLAN section 5.
"""
import copy
import dataclasses
import math
import random

import pytest

from aerosat.avionics.calibration import PadCalibration
from aerosat.avionics.estimator import ComplementaryFilter, baro_altitude_agl_m
from aerosat.avionics.flight_computer import (
    REQUIRED_SENSORS,
    FlightComputer,
    avionics_config_from_scenario,
    check_estimator_gains,
    load_avionics_config,
)
from aerosat.avionics.state_machine import FlightStateMachine, StateMachineConfig
from aerosat.core.constants import G0_MPS2, P0_PA, T0_K
from aerosat.core.faults import ConfigurationFault, FaultCode
from aerosat.core.faults import ConfigurationReason as Reason
from aerosat.core.types import DataQuality, EventSource, RejectedSample, RejectionReason
from aerosat.core.types import FlightEventType as Event
from aerosat.core.types import FlightState as State
from aerosat.core.types import SensorSample, TruthState, VehicleCommand
from aerosat.sensors.suite import SensorSuite, suite_config_from_scenario
from aerosat.sim.atmosphere import StandardAtmosphere

from .conftest import REF01, FlightRecord, flight, ref01_avionics_config, run_flight

# TEST_PLAN section 5: allowed (detection time - truth time), seconds.
TOLERANCE_S = {
    Event.LIFTOFF: (0.0, 0.10),
    Event.BURNOUT: (-0.10, 0.15),
    Event.APOGEE: (-1.0, 1.0),
    Event.TOUCHDOWN: (0.0, 3.0),
}
ORDER = [Event.LIFTOFF, Event.BURNOUT, Event.APOGEE, Event.DRAG_DEVICE_DEPLOYED, Event.TOUCHDOWN]
STATE_ORDER = [State.PAD_IDLE, State.BOOST, State.COAST, State.DESCENT, State.LANDED]


def detection_errors_s(record: FlightRecord) -> dict[Event, float]:
    truth = record.truth_events
    truth_time_s = {
        Event.LIFTOFF: truth.liftoff_tick / 1000,
        Event.BURNOUT: truth.burnout_tick / 1000,
        Event.APOGEE: truth.apogee_tick / 1000,
        Event.TOUCHDOWN: truth.touchdown_time_s,
    }
    return {event: record.detection_time_s(event) - truth_time_s[event] for event in TOLERANCE_S}


def assert_within_tolerance(record: FlightRecord, label: str) -> dict[Event, float]:
    errors = detection_errors_s(record)
    for event, (low, high) in TOLERANCE_S.items():
        assert low <= errors[event] <= high, f"{label}: {event.value} detected {errors[event]:+.3f} s from truth"
    return errors


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
    """Feeds a state machine one avionics tick at a time and collects what it emits."""

    def __init__(self, fsm: FlightStateMachine) -> None:
        self.fsm, self.tick, self.events, self.commands = fsm, 0, [], []

    def feed(self, count: int, *, force: float | None = 9.8, altitude: float = 0.0, velocity: float = 0.0,
             baro: float | None = None, accel: float = 9.0, quality: DataQuality = DataQuality.NOMINAL,
             accel_rejected: bool = False, baro_rejected: bool = False) -> None:
        """`baro` is the barometric altitude (defaults to `altitude`) and `accel` the gravity-compensated
        estimated acceleration. `accel` defaults to 9.0 m/s^2, which never qualifies for the touchdown
        fallback, so tests of other criteria cannot trip it by accident. Every tick carries a valid
        barometer sample at `baro`, unless `baro_rejected`. `quality` is the estimate quality."""
        for _ in range(count):
            baro_m = altitude if baro is None else baro
            events, commands = self.fsm.update(
                self.tick, specific_force_mps2=None if accel_rejected else force, altitude_m=altitude,
                velocity_mps=velocity, baro_altitude_m=baro_m, acceleration_mps2=accel,
                estimate_quality=quality, accel_rejected=accel_rejected,
                baro_sample_m=None if baro_rejected else baro_m, baro_rejected=baro_rejected,
            )
            self.events += events
            self.commands += [(self.tick, command) for command in commands]
            self.tick += 5


# ---------------------------------------------------------------- EVT-01, EVT-02, EVT-03

def test_evt_01_each_transition_occurs_at_the_expected_sample() -> None:
    d = Driver(machine())
    d.feed(20, force=9.8)                               # ticks 0..95: on the pad
    assert d.fsm.state is State.PAD_IDLE
    d.feed(9, force=30.0)                               # ticks 100..140: nine samples above 15 m/s^2
    assert d.fsm.state is State.PAD_IDLE
    d.feed(1, force=30.0)                               # tick 145: the tenth
    assert d.fsm.state is State.BOOST and (d.events[-1].event, d.events[-1].tick) == (Event.LIFTOFF, 145)

    d.feed(50, force=30.0, velocity=100.0)              # ticks 150..395
    d.feed(9, force=-2.0, velocity=200.0)               # ticks 400..440
    assert d.fsm.state is State.BOOST
    d.feed(1, force=-2.0, velocity=200.0)               # tick 445
    assert d.fsm.state is State.COAST and (d.events[-1].event, d.events[-1].tick) == (Event.BURNOUT, 445)

    d.feed(300, force=-2.0, velocity=50.0, altitude=1000.0)   # ticks 450..1945: past the 1 s lock-out
    d.feed(9, force=0.0, velocity=-0.5, altitude=1500.0)      # ticks 1950..1990
    assert d.fsm.state is State.COAST
    d.feed(1, force=0.0, velocity=-0.5, altitude=1500.0)      # tick 1995
    assert d.fsm.state is State.DESCENT
    apogee = d.events[-1]
    assert (apogee.event, apogee.tick, apogee.flight_state) == (Event.APOGEE, 1995, State.DESCENT)
    assert apogee.detail == {"altitude_m": 1500.0} and apogee.t_sim_s == 1.995

    d.feed(199, force=0.0, velocity=-12.0, altitude=1400.0)   # ticks 2000..2990: delay not yet elapsed
    assert d.commands == []
    d.feed(1, force=0.0, velocity=-12.0, altitude=1400.0)     # tick 2995 = apogee + 1000
    assert d.commands == [(2995, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert (d.events[-1].event, d.events[-1].tick) == (Event.DRAG_DEVICE_DEPLOYED, 2995)

    altitude = 1400.0
    for _ in range(100):                                       # ticks 3000..3495: descending at 12 m/s
        altitude -= 0.06
        d.feed(1, force=0.0, velocity=-12.0, altitude=altitude)
    first_still_tick = d.tick                                  # 3500
    d.feed(400, force=9.8, velocity=0.0, altitude=0.0)         # 400 still samples: window not yet full of them
    assert d.fsm.state is State.DESCENT
    d.feed(1, force=9.8, velocity=0.0, altitude=0.0)           # the 401st spans 2.0 s
    assert d.fsm.state is State.LANDED
    assert (d.events[-1].event, d.events[-1].tick) == (Event.TOUCHDOWN, first_still_tick + 2000)
    assert [e.event for e in d.events] == ORDER


def test_evt_01_apogee_waits_for_the_lockout_after_burnout() -> None:
    d = Driver(machine())
    d.feed(10, force=30.0)
    d.feed(10, force=-2.0, velocity=100.0)
    burnout_tick = d.events[-1].tick
    d.feed(199, force=0.0, velocity=-5.0)          # negative velocity throughout, but inside the lock-out
    assert d.fsm.state is State.COAST
    d.feed(1, force=0.0, velocity=-5.0)
    assert (d.events[-1].event, d.events[-1].tick) == (Event.APOGEE, burnout_tick + 1000)


def test_evt_01_liftoff_and_burnout_count_accelerometer_samples_only() -> None:
    d = Driver(machine())
    for _ in range(30):                             # an accelerometer sample every second avionics tick
        d.feed(1, force=30.0)
        d.feed(1, force=None)
    assert (d.events[0].event, d.events[0].tick) == (Event.LIFTOFF, 90)   # the tenth sample, at tick 9 * 10


@pytest.mark.parametrize("stage", ["liftoff", "burnout", "apogee"])
def test_evt_02_debounce(stage: str) -> None:
    """Criterion true for N-1 samples then false: no transition. True for N: transition."""
    d = Driver(machine())
    if stage == "liftoff":
        true, false, before, after = dict(force=30.0), dict(force=9.8), State.PAD_IDLE, State.BOOST
    else:
        d.feed(10, force=30.0)
        if stage == "burnout":
            true, false, before, after = dict(force=-1.0), dict(force=5.0), State.BOOST, State.COAST
        else:
            d.feed(10, force=-1.0, velocity=100.0)
            d.feed(250, force=0.0, velocity=100.0)   # past the lock-out
            true, false = dict(force=0.0, velocity=-1.0), dict(force=0.0, velocity=1.0)
            before, after = State.COAST, State.DESCENT
    for _ in range(5):                               # repeated interrupted attempts
        d.feed(9, **true)
        d.feed(1, **false)
        assert d.fsm.state is before
    d.feed(9, **true)
    assert d.fsm.state is before
    d.feed(1, **true)
    assert d.fsm.state is after


@pytest.mark.parametrize("seed", range(20))
def test_evt_03_states_never_go_backward_under_adversarial_input(seed: int) -> None:
    rng = random.Random(seed)
    d = Driver(machine(deploy_delay_ticks=rng.choice([0, 5, 1000])))
    rank = 0
    for _ in range(20_000):
        mode = rng.random()
        force = None if mode < 0.1 else rng.uniform(-60.0, 90.0) if mode < 0.7 else rng.choice([30.0, -5.0])
        velocity = rng.uniform(-60.0, 60.0) if rng.random() < 0.7 else rng.choice([-0.5, 0.0, 0.5])
        altitude = rng.uniform(-5.0, 3000.0) if rng.random() < 0.7 else 0.0
        d.feed(1, force=force, altitude=altitude, velocity=velocity)
        assert STATE_ORDER.index(d.fsm.state) >= rank
        rank = STATE_ORDER.index(d.fsm.state)
    names = [e.event for e in d.events]
    assert len(names) == len(set(names))                      # each event at most once
    assert names == [e for e in ORDER if e in names]          # and in order
    assert len(d.commands) <= 1


# ---------------------------------------------------------------- EVT-04

def pad_truth(tick: int) -> TruthState:
    return TruthState(tick, tick / 1000, 0.0, 0.0, 0.0, 0.0, G0_MPS2, 20.0, 0.0, 0.0, P0_PA, T0_K, 1.225, 0.0, 0.0, 0.0, True)


def stays_on_pad_for_60_s(seed: int) -> bool:
    suite = SensorSuite(suite_config_from_scenario(REF01["sensors"]), master_seed=seed)
    computer = FlightComputer(ref01_avionics_config())
    for tick in range(0, 60_001, 5):
        output = computer.tick(tick, suite.sample(pad_truth(tick)))
        if output.events or output.flight_state is not State.PAD_IDLE:
            return False
    return computer.calibration is not None


@pytest.mark.parametrize("seed", range(10))
def test_evt_04_no_false_liftoff_seeds_0_to_9(seed: int) -> None:
    assert stays_on_pad_for_60_s(seed)


@pytest.mark.slow
def test_evt_04_no_false_liftoff_seeds_0_to_99() -> None:
    assert [seed for seed in range(100) if not stays_on_pad_for_60_s(seed)] == []


# ---------------------------------------------------------------- EVT-05, EVT-06, EVT-08, EVT-12

def test_evt_05_ref01_with_ideal_sensors(ideal_flight: FlightRecord) -> None:
    errors = assert_within_tolerance(ideal_flight, "ideal sensors")
    print("\nEVT-05 detection minus truth (s):", {e.value: round(v, 3) for e, v in errors.items()})


@pytest.mark.parametrize("seed", range(10))
def test_evt_06_ref01_with_default_noise_seeds_0_to_9(seed: int) -> None:
    assert_within_tolerance(flight(seed), f"seed {seed}")


@pytest.mark.slow
def test_evt_06_ref01_with_default_noise_seeds_0_to_99() -> None:
    errors = [assert_within_tolerance(flight(seed), f"seed {seed}") for seed in range(100)]
    print("\nEVT-06 detection minus truth over seeds 0-99 (s): min / mean / max")
    for event in TOLERANCE_S:
        values = [e[event] for e in errors]
        print(f"  {event.value:10} {min(values):+.3f} / {sum(values) / len(values):+.3f} / {max(values):+.3f}"
              f"   allowed {TOLERANCE_S[event][0]:+.2f} to {TOLERANCE_S[event][1]:+.2f}")


@pytest.mark.parametrize("which", ["ideal", *range(10)])
def test_evt_08_each_event_exactly_once_and_in_order(which) -> None:
    record = flight(0, True) if which == "ideal" else flight(which)
    assert [e.event for e in record.events] == ORDER
    assert [e.tick for e in record.events] == sorted(e.tick for e in record.events)
    assert [e.flight_state for e in record.events] == [State.BOOST, State.COAST, State.DESCENT, State.DESCENT, State.LANDED]
    assert all(e.tick % 5 == 0 and e.t_sim_s == e.tick / 1000 for e in record.events)


@pytest.mark.parametrize("which", ["ideal", 0, 1, 2])
def test_evt_12_deploy_command_issued_once_exactly_the_delay_after_detected_apogee(which) -> None:
    record = flight(0, True) if which == "ideal" else flight(which)
    (apogee,) = record.detected(Event.APOGEE)
    assert record.commands == [(apogee.tick + 1000, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    (deployed,) = record.detected(Event.DRAG_DEVICE_DEPLOYED)
    assert deployed.tick == apogee.tick + 1000
    assert record.drag_area_change_tick == apogee.tick + 1001     # applied by the simulator on the next tick


def test_evt_12_no_command_when_the_drag_device_is_disabled() -> None:
    d = Driver(machine(drag_device_enabled=False))
    d.feed(10, force=30.0)
    d.feed(10, force=-1.0, velocity=100.0)
    d.feed(250, force=0.0, velocity=-5.0)
    assert d.fsm.state is State.DESCENT
    d.feed(1_000, force=0.0, velocity=-12.0, altitude=500.0)
    assert d.commands == [] and Event.DRAG_DEVICE_DEPLOYED not in [e.event for e in d.events]


# ---------------------------------------------------------------- EVT-13: REF-01 pinned

# Detection ticks of (LIFTOFF, BURNOUT, APOGEE, DRAG_DEVICE_DEPLOYED, TOUCHDOWN) for REF-01, recorded
# on 2026-11-11 from commit 18f99ea (Python 3.14.10, Windows x86-64), after EVT-05 and EVT-06 passed.
# This is a regression pin, not an oracle: it proves nothing about correctness. Its job is to show
# that a later change to the avionics leaves the reference flight exactly as it was. A change to
# these numbers must be deliberate and explained.
REF01_PINNED_EVENT_TICKS = {
    "ideal": (3065, 6230, 24295, 25295, 187200),
    0: (3065, 6230, 24265, 25265, 187180),
    1: (3065, 6230, 24285, 25285, 187260),
    2: (3070, 6230, 24285, 25285, 187200),
    3: (3070, 6230, 24285, 25285, 187200),
    4: (3070, 6230, 24255, 25255, 187200),
    5: (3070, 6230, 24325, 25325, 187160),
    6: (3065, 6230, 24310, 25310, 187200),
    7: (3070, 6230, 24330, 25330, 187160),
    8: (3070, 6230, 24325, 25325, 187240),
    9: (3070, 6230, 24245, 25245, 187240),
}


@pytest.mark.parametrize("which", list(REF01_PINNED_EVENT_TICKS))
def test_evt_13_ref01_event_ticks_are_unchanged(which) -> None:
    record = flight(0, True) if which == "ideal" else flight(which)
    assert [e.event for e in record.events] == ORDER
    assert tuple(e.tick for e in record.events) == REF01_PINNED_EVENT_TICKS[which]
    deploy_tick = REF01_PINNED_EVENT_TICKS[which][3]
    assert record.commands == [(deploy_tick, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert record.drag_area_change_tick == deploy_tick + 1


# ---------------------------------------------------------------- EVT-19: backup deployment timer

def coasting_driver(**overrides) -> tuple[Driver, int]:
    """A machine driven through LIFTOFF and BURNOUT. Returns it with the BURNOUT detection tick."""
    d = Driver(machine(**overrides))
    d.feed(10, force=30.0)
    d.feed(10, force=-2.0, velocity=200.0)
    assert d.fsm.state is State.COAST and d.events[-1].event is Event.BURNOUT
    return d, d.events[-1].tick


def test_evt_19_backup_timer_does_not_run_before_burnout() -> None:
    d = Driver(machine(backup_deployment_ticks=100))
    d.feed(400, force=9.8)                               # 2 s on the pad
    assert d.fsm.state is State.PAD_IDLE and d.events == []
    d.feed(10, force=30.0)
    d.feed(400, force=30.0, velocity=100.0)              # 2 s of boost, far longer than the 100 ms timer
    assert d.fsm.state is State.BOOST and [e.event for e in d.events] == [Event.LIFTOFF]
    assert d.commands == []


def test_evt_19_backup_timer_expires_exactly_the_delay_after_detected_burnout() -> None:
    d, burnout_tick = coasting_driver()
    d.feed(3_999, force=0.0, velocity=50.0, altitude=1_800.0)       # still ascending: no apogee detected
    assert d.tick - 5 == burnout_tick + 19_995
    assert d.fsm.state is State.COAST and d.commands == []
    assert [e.event for e in d.events] == [Event.LIFTOFF, Event.BURNOUT]
    d.feed(1, force=0.0, velocity=50.0, altitude=1_812.5)           # burnout + 20 000 ticks
    expiry_tick = burnout_tick + 20_000
    assert d.fsm.state is State.DESCENT
    assert [e.event for e in d.events] == [Event.LIFTOFF, Event.BURNOUT, Event.APOGEE, Event.DRAG_DEVICE_DEPLOYED]
    assert d.commands == [(expiry_tick, VehicleCommand.DEPLOY_DRAG_DEVICE)]


def test_evt_19_inferred_apogee_metadata() -> None:
    d, burnout_tick = coasting_driver()
    d.feed(4_000, force=0.0, velocity=50.0, altitude=1_812.5)
    apogee, deployed = d.events[-2], d.events[-1]
    expiry_tick = burnout_tick + 20_000

    assert apogee.event is Event.APOGEE
    assert apogee.source is EventSource.BACKUP_TIMER                # inferred, not detected
    assert apogee.quality is DataQuality.UNRELIABLE
    assert (apogee.tick, apogee.t_sim_s) == (expiry_tick, expiry_tick / 1000)   # the actual time it was recorded
    assert apogee.flight_state is State.DESCENT
    assert apogee.detail == {"altitude_m": 1_812.5}
    assert "backup deployment timer expired" in apogee.reason
    assert "20000 ms" in apogee.reason and f"tick {burnout_tick}" in apogee.reason

    assert deployed.event is Event.DRAG_DEVICE_DEPLOYED
    assert (deployed.source, deployed.quality, deployed.tick) == (EventSource.BACKUP_TIMER, DataQuality.UNRELIABLE, expiry_tick)
    assert deployed.reason == apogee.reason


def test_evt_19_detected_events_are_marked_detected_and_nominal() -> None:
    d, _ = coasting_driver()
    d.feed(250, force=0.0, velocity=50.0)
    d.feed(10, force=0.0, velocity=-1.0, altitude=2_000.0)          # directly detected apogee
    d.feed(200, force=0.0, velocity=-12.0, altitude=1_990.0)        # normal deployment after the delay
    assert [e.event for e in d.events] == [Event.LIFTOFF, Event.BURNOUT, Event.APOGEE, Event.DRAG_DEVICE_DEPLOYED]
    for event in d.events:
        assert (event.source, event.quality, event.reason) == (EventSource.DETECTED, DataQuality.NOMINAL, None)


def test_evt_19_directly_detected_apogee_takes_precedence_and_cancels_the_timer() -> None:
    d, burnout_tick = coasting_driver()
    d.feed(250, force=0.0, velocity=50.0)
    d.feed(10, force=0.0, velocity=-1.0, altitude=2_000.0)
    (apogee,) = [e for e in d.events if e.event is Event.APOGEE]
    assert apogee.source is EventSource.DETECTED and apogee.tick < burnout_tick + 20_000
    d.feed(6_000, force=0.0, velocity=-12.0, altitude=1_500.0)      # run well past burnout + 20 s, still descending
    assert d.tick > burnout_tick + 20_000
    assert [e.event for e in d.events].count(Event.APOGEE) == 1     # no second, inferred apogee
    assert d.commands == [(apogee.tick + 1000, VehicleCommand.DEPLOY_DRAG_DEVICE)]   # normal path only, once
    assert all(e.source is EventSource.DETECTED for e in d.events)


def test_evt_19_detection_wins_when_both_conditions_hold_on_the_same_tick() -> None:
    """The tenth negative-velocity tick lands exactly on the timer's expiry tick."""
    d, burnout_tick = coasting_driver(backup_deployment_ticks=2_000)
    d.feed(390, force=0.0, velocity=50.0)                           # up to burnout + 1 950
    d.feed(10, force=0.0, velocity=-1.0, altitude=900.0)            # tenth is at burnout + 2 000
    (apogee,) = [e for e in d.events if e.event is Event.APOGEE]
    assert apogee.tick == burnout_tick + 2_000
    assert apogee.source is EventSource.DETECTED and apogee.quality is DataQuality.NOMINAL
    assert d.commands == []                                         # normal delay has not elapsed yet


def test_evt_19_inferred_apogee_never_triggers_the_normal_deployment_path() -> None:
    """Drag device disabled in the avionics: the timer infers apogee but nothing may be commanded,
    neither at expiry nor one normal delay later."""
    d, burnout_tick = coasting_driver(drag_device_enabled=False)
    d.feed(4_000, force=0.0, velocity=50.0, altitude=1_800.0)
    assert d.fsm.state is State.DESCENT and d.events[-1].source is EventSource.BACKUP_TIMER
    d.feed(1_000, force=0.0, velocity=-12.0, altitude=1_700.0)
    assert d.commands == [] and Event.DRAG_DEVICE_DEPLOYED not in [e.event for e in d.events]

    # Device enabled: exactly one command, at expiry, and none at expiry + the normal 1000-tick delay.
    d, burnout_tick = coasting_driver()
    d.feed(4_000, force=0.0, velocity=50.0, altitude=1_800.0)
    d.feed(1_000, force=0.0, velocity=-12.0, altitude=1_700.0)
    assert d.commands == [(burnout_tick + 20_000, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert [e.event for e in d.events].count(Event.DRAG_DEVICE_DEPLOYED) == 1


def test_evt_19_touchdown_is_still_detected_after_an_inferred_apogee() -> None:
    d, _ = coasting_driver()
    d.feed(4_000, force=0.0, velocity=50.0, altitude=1_800.0)
    d.feed(401, force=9.8, velocity=0.0, altitude=0.0)
    assert d.fsm.state is State.LANDED
    assert d.events[-1].event is Event.TOUCHDOWN and d.events[-1].source is EventSource.DETECTED


def test_evt_19_ref01_never_reaches_the_backup_timer() -> None:
    """In REF-01 apogee is detected about 18 s after burnout, before the 20 s timer."""
    for record in (flight(0, True), *(flight(seed) for seed in range(10))):
        burnout_tick = record.detected(Event.BURNOUT)[0].tick
        (apogee,) = record.detected(Event.APOGEE)
        assert apogee.tick < burnout_tick + 20_000
        assert all(e.source is EventSource.DETECTED and e.quality is DataQuality.NOMINAL and e.reason is None
                   for e in record.events)


def test_evt_19_flight_with_apogee_detection_suppressed_deploys_on_the_backup_timer() -> None:
    """Closed loop, REF-01, seed 0, with the apogee debounce made unreachable so only the timer can act."""
    blind = ref01_avionics_config(apogee_lockout_after_burnout_ms=599_000)   # lock-out outlasts the flight
    record = run_flight(0, avionics=blind)
    burnout_tick = record.detected(Event.BURNOUT)[0].tick
    (apogee,) = record.detected(Event.APOGEE)
    assert (apogee.tick, apogee.source, apogee.quality) == (burnout_tick + 20_000, EventSource.BACKUP_TIMER, DataQuality.UNRELIABLE)
    assert record.commands == [(burnout_tick + 20_000, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert record.drag_area_change_tick == burnout_tick + 20_001
    assert apogee.tick > record.truth_events.apogee_tick                    # after true apogee, not during ascent
    assert [e.event for e in record.events] == ORDER
    touchdown_error_s = record.detection_time_s(Event.TOUCHDOWN) - record.truth_events.touchdown_time_s
    assert 0.0 <= touchdown_error_s <= 3.0


def test_evt_19_backup_delay_configuration() -> None:
    assert (REF01["avionics"]["backup_deployment_delay_ms"], REF01["avionics"]["expected_coast_duration_ms"]) == (20_000, 18_047)
    config = load_avionics_config(REF01)
    assert (config.backup_deployment_delay_ms, config.expected_coast_duration_ms) == (20_000, 18_047)
    for name in ("backup_deployment_delay_ms", "expected_coast_duration_ms"):      # required: no default
        fault = rejected(scenario_with(f"avionics.{name}", ...))
        assert (fault.reason, fault.field) == (Reason.MISSING_FIELD, f"avionics.{name}")
        for bad, reason in ((0, Reason.OUT_OF_RANGE), (-1, Reason.OUT_OF_RANGE), (20000.0, Reason.INVALID_TYPE),
                            ("20000", Reason.INVALID_TYPE), (True, Reason.INVALID_TYPE), (None, Reason.INVALID_TYPE)):
            fault = rejected(scenario_with(f"avionics.{name}", bad))
            assert (fault.reason, fault.field) == (reason, f"avionics.{name}")
    # The timer must exceed the expected coast duration: equal is rejected, one millisecond more is accepted.
    for too_short in (18_047, 18_046, 1):
        fault = rejected(scenario_with("avionics.backup_deployment_delay_ms", too_short))
        assert (fault.reason, fault.field) == (Reason.OUT_OF_RANGE, "avionics.backup_deployment_delay_ms")
    load_avionics_config(scenario_with("avionics.backup_deployment_delay_ms", 18_048))
    fault = rejected(scenario_with("avionics.expected_coast_duration_ms", 20_000))
    assert fault.field == "avionics.backup_deployment_delay_ms"


def test_evt_19_ref01_expected_coast_duration_matches_the_simulated_flight(ideal_flight: FlightRecord) -> None:
    """18047 ms is the true burnout-to-apogee time of REF-01 (truth ticks 6200 and 24247)."""
    truth = ideal_flight.truth_events
    assert truth.apogee_tick - truth.burnout_tick == REF01["avionics"]["expected_coast_duration_ms"]


def test_evt_19_inferred_apogee_cannot_activate_normal_deployment_independently() -> None:
    """White-box regression for the `_apogee_inferred` guard.

    After the backup timer has inferred apogee and deployed, the deployment latch is cleared by hand,
    as if the backup deployment had not latched. The normal apogee-plus-delay path must still refuse
    to act, because the apogee was inferred, not detected. Without the guard it would issue a command
    one normal delay (1000 ticks) after the inferred apogee.
    """
    d, burnout_tick = coasting_driver()
    d.feed(4_000, force=0.0, velocity=50.0, altitude=1_800.0)
    assert d.events[-2].source is EventSource.BACKUP_TIMER and d.fsm._apogee_inferred is True
    assert d.commands == [(burnout_tick + 20_000, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    d.fsm._deployed = False
    d.feed(400, force=0.0, velocity=-12.0, altitude=1_700.0)        # 2 s: twice the normal delay
    assert d.commands == [(burnout_tick + 20_000, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert [e.event for e in d.events].count(Event.DRAG_DEVICE_DEPLOYED) == 1


# ---------------------------------------------------------------- EVT-15: burnout fallback

def boosting_driver(**overrides) -> Driver:
    d = Driver(machine(**overrides))
    d.feed(10, force=30.0)
    assert d.fsm.state is State.BOOST
    d.feed(100, force=40.0, velocity=100.0)
    return d


def test_evt_15_nineteen_qualifying_samples_then_one_not_gives_no_burnout() -> None:
    d = boosting_driver()
    for _ in range(3):                                              # repeated interrupted attempts
        d.feed(19, force=0.5, velocity=200.0)                       # below 1.0 but not below 0
        d.feed(1, force=1.5, velocity=200.0)                        # breaks the run
        assert d.fsm.state is State.BOOST
    assert [e.event for e in d.events] == [Event.LIFTOFF]


def test_evt_15_exactly_twenty_qualifying_samples_give_fallback_burnout() -> None:
    d = boosting_driver()
    start_tick = d.tick
    d.feed(19, force=0.5, velocity=200.0)
    assert d.fsm.state is State.BOOST
    d.feed(1, force=0.5, velocity=200.0)                            # the twentieth
    burnout = d.events[-1]
    assert d.fsm.state is State.COAST
    assert (burnout.event, burnout.tick) == (Event.BURNOUT, start_tick + 19 * 5)
    assert burnout.source is EventSource.FALLBACK_CRITERION
    assert burnout.quality is DataQuality.NOMINAL
    assert "1.0 m/s^2" in burnout.reason and "20 consecutive samples" in burnout.reason


def test_evt_15_threshold_is_strict_and_counts_accelerometer_samples_only() -> None:
    d = boosting_driver()
    d.feed(40, force=1.0, velocity=200.0)                           # exactly at the threshold: does not qualify
    assert d.fsm.state is State.BOOST
    for _ in range(19):                                             # ticks with no accelerometer sample do not count
        d.feed(1, force=0.5, velocity=200.0)
        d.feed(1, force=None, velocity=200.0)
    assert d.fsm.state is State.BOOST
    d.feed(1, force=0.5, velocity=200.0)
    assert d.fsm.state is State.COAST and d.events[-1].source is EventSource.FALLBACK_CRITERION


def test_evt_15_primary_criterion_takes_precedence() -> None:
    """Negative specific force satisfies both criteria; the primary one fires at its tenth sample."""
    d = boosting_driver()
    start_tick = d.tick
    d.feed(30, force=-2.0, velocity=200.0)
    (burnout,) = [e for e in d.events if e.event is Event.BURNOUT]
    assert burnout.tick == start_tick + 9 * 5
    assert (burnout.source, burnout.quality, burnout.reason) == (EventSource.DETECTED, DataQuality.NOMINAL, None)


def test_evt_15_primary_wins_when_both_complete_on_the_same_sample() -> None:
    d = boosting_driver()
    d.feed(10, force=0.5, velocity=200.0)                           # fallback count 10, primary 0
    d.feed(10, force=-2.0, velocity=200.0)                          # fallback reaches 20 as primary reaches 10
    (burnout,) = [e for e in d.events if e.event is Event.BURNOUT]
    assert burnout.source is EventSource.DETECTED


def test_evt_15_powered_flight_transients_shorter_than_twenty_samples_are_ignored() -> None:
    """Thrust at 40 m/s^2 with dips below the threshold of 1, 5, 10 and 19 samples."""
    d = boosting_driver()
    for dip in (1, 5, 10, 19):
        d.feed(dip, force=0.2, velocity=150.0)
        d.feed(50, force=40.0, velocity=150.0)
        assert d.fsm.state is State.BOOST
    assert [e.event for e in d.events] == [Event.LIFTOFF]
    d.feed(9, force=-5.0, velocity=150.0)                           # a negative dip shorter than the primary debounce
    d.feed(50, force=40.0, velocity=150.0)
    assert d.fsm.state is State.BOOST


def zero_drag_scenario(backup_ms: int = 26_000, coast_ms: int = 23_665) -> dict:
    """REF-01 with no aerodynamic drag, with its real coast duration declared."""
    scenario = copy.deepcopy(REF01)
    scenario["vehicle"]["drag_coefficient"] = 0.0
    scenario["avionics"]["expected_coast_duration_ms"] = coast_ms
    scenario["avionics"]["backup_deployment_delay_ms"] = backup_ms
    return scenario


@pytest.mark.parametrize("seed", range(10))
def test_evt_15_zero_drag_burnout_is_detected_within_tolerance(seed: int) -> None:
    record = run_flight(seed, scenario=zero_drag_scenario(), stop_tick=8_000)
    (burnout,) = record.detected(Event.BURNOUT)
    error_s = burnout.t_sim_s - record.truth_events.burnout_tick / 1000
    low, high = TOLERANCE_S[Event.BURNOUT]
    assert low <= error_s <= high, f"seed {seed}: BURNOUT detected {error_s:+.3f} s from truth"
    assert burnout.source is EventSource.FALLBACK_CRITERION and burnout.quality is DataQuality.NOMINAL
    assert [e.event for e in record.events] == [Event.LIFTOFF, Event.BURNOUT]


def test_evt_15_ref01_burnout_is_still_found_by_the_primary_criterion() -> None:
    for record in (flight(0, True), *(flight(seed) for seed in range(10))):
        (burnout,) = record.detected(Event.BURNOUT)
        assert (burnout.tick, burnout.source, burnout.reason) == (6_230, EventSource.DETECTED, None)


def test_evt_15_timer_with_a_correctly_declared_coast_duration() -> None:
    """Zero drag coasts for 23 665 ms, so the 20 000 ms REF-01 timer is rejected at load."""
    fault = rejected(zero_drag_scenario(backup_ms=20_000))
    assert (fault.reason, fault.field) == (Reason.OUT_OF_RANGE, "avionics.backup_deployment_delay_ms")

    # With a 26 000 ms timer: burnout by the fallback, apogee detected directly, the timer never fires.
    record = run_flight(0, scenario=zero_drag_scenario(), stop_tick=33_500)
    truth = record.truth_events
    assert truth.apogee_tick - truth.burnout_tick == 23_665           # the declared coast duration is the real one
    (burnout,) = record.detected(Event.BURNOUT)
    (apogee,) = record.detected(Event.APOGEE)
    assert burnout.source is EventSource.FALLBACK_CRITERION
    assert apogee.source is EventSource.DETECTED and apogee.tick < burnout.tick + 26_000
    assert abs(apogee.t_sim_s - truth.apogee_tick / 1000) <= 1.0
    assert record.commands == [(apogee.tick + 1000, VehicleCommand.DEPLOY_DRAG_DEVICE)]


def test_evt_15_backup_timer_starts_from_the_fallback_burnout() -> None:
    """Apogee detection suppressed: the timer expires exactly 26 000 ticks after the fallback burnout.

    The F1-D6 liftoff timer is disabled here to test the burnout timer on its own: with the fallback
    burnout detected 95 ms after true burnout, the liftoff timer (LIFTOFF + 29 200 ms) would act 30 ms
    earlier. Both orders are checked."""
    scenario = zero_drag_scenario()
    scenario["avionics"]["apogee_lockout_after_burnout_ms"] = 599_000
    both = run_flight(0, scenario=scenario, stop_tick=33_500)
    (liftoff,) = both.detected(Event.LIFTOFF)
    assert both.commands == [(liftoff.tick + 29_200, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert both.commands[0][0] > both.truth_events.apogee_tick
    burnout_only = dataclasses.replace(load_avionics_config(scenario), liftoff_backup_delay_ms=None)
    record = run_flight(0, scenario=scenario, stop_tick=33_500, avionics=burnout_only)
    (burnout,) = record.detected(Event.BURNOUT)
    (apogee,) = record.detected(Event.APOGEE)
    assert burnout.source is EventSource.FALLBACK_CRITERION
    assert (apogee.tick, apogee.source, apogee.quality) == (burnout.tick + 26_000, EventSource.BACKUP_TIMER, DataQuality.UNRELIABLE)
    assert apogee.tick > record.truth_events.apogee_tick             # after true apogee, not during ascent
    assert record.commands == [(apogee.tick, VehicleCommand.DEPLOY_DRAG_DEVICE)]


def test_evt_15_configuration() -> None:
    avionics = REF01["avionics"]
    assert (avionics["burnout_fallback_specific_force_mps2"], avionics["burnout_fallback_samples"]) == (1.0, 20)
    config = load_avionics_config(REF01)
    assert (config.burnout_fallback_specific_force_mps2, config.burnout_fallback_samples) == (1.0, 20)

    threshold, samples = "avionics.burnout_fallback_specific_force_mps2", "avionics.burnout_fallback_samples"
    for path in (threshold, samples):                                 # required: no default
        fault = rejected(scenario_with(path, ...))
        assert (fault.reason, fault.field) == (Reason.MISSING_FIELD, path)
    for bad, reason in ((0.0, Reason.OUT_OF_RANGE), (-1.0, Reason.OUT_OF_RANGE), (math.nan, Reason.INVALID_TYPE),
                        (math.inf, Reason.INVALID_TYPE), ("1.0", Reason.INVALID_TYPE), (True, Reason.INVALID_TYPE),
                        (None, Reason.INVALID_TYPE)):
        fault = rejected(scenario_with(threshold, bad))
        assert (fault.reason, fault.field) == (reason, threshold)
    for bad, reason in ((0, Reason.OUT_OF_RANGE), (-20, Reason.OUT_OF_RANGE), (20.0, Reason.INVALID_TYPE),
                        ("20", Reason.INVALID_TYPE), (True, Reason.INVALID_TYPE), (None, Reason.INVALID_TYPE)):
        fault = rejected(scenario_with(samples, bad))
        assert (fault.reason, fault.field) == (reason, samples)

    # The threshold must be below the liftoff threshold (15.0): equal is rejected.
    for too_high in (15.0, 20.0):
        fault = rejected(scenario_with(threshold, too_high))
        assert (fault.reason, fault.field) == (Reason.OUT_OF_RANGE, threshold)
    load_avionics_config(scenario_with(threshold, 14.9))
    # The sample count must be at least debounce_samples (10): 9 is rejected, 10 is accepted.
    fault = rejected(scenario_with(samples, 9))
    assert (fault.reason, fault.field) == (Reason.OUT_OF_RANGE, samples)
    load_avionics_config(scenario_with(samples, 10))


# ---------------------------------------------------------------- EVT-16: touchdown fallback

FALLBACK_SAMPLES = 561            # 2800 ms at one sample per 5 ms, inclusive of both ends


def descending_driver(**overrides) -> Driver:
    """A machine driven into DESCENT and falling fast, so that neither touchdown criterion is close."""
    d = Driver(machine(**overrides))
    d.feed(10, force=30.0)
    d.feed(10, force=-2.0, velocity=200.0)
    d.feed(250, force=0.0, velocity=50.0, altitude=1_000.0)
    d.feed(10, force=0.0, velocity=-1.0, altitude=1_000.0)
    assert d.fsm.state is State.DESCENT
    d.feed(50, force=0.0, velocity=-150.0, altitude=500.0, accel=-9.0)
    return d


def still(d: Driver, count: int, *, baro: float = 0.0, accel: float = 0.0) -> None:
    """On the ground after a hard impact: barometer still, no acceleration, velocity estimate still wrong."""
    d.feed(count, force=9.8, velocity=-150.0, altitude=-40.0, baro=baro, accel=accel)


def test_evt_16_one_sample_short_of_the_window_does_not_fire() -> None:
    d = descending_driver()
    still(d, FALLBACK_SAMPLES - 1)
    assert d.fsm.state is State.DESCENT
    assert Event.TOUCHDOWN not in [e.event for e in d.events]


def test_evt_16_a_complete_qualifying_window_fires() -> None:
    d = descending_driver()
    first_tick = d.tick
    still(d, FALLBACK_SAMPLES)
    touchdown = d.events[-1]
    assert d.fsm.state is State.LANDED
    assert (touchdown.event, touchdown.tick) == (Event.TOUCHDOWN, first_tick + 2_800)   # spans exactly 2800 ms
    assert touchdown.source is EventSource.FALLBACK_CRITERION
    assert touchdown.quality is DataQuality.NOMINAL
    assert touchdown.flight_state is State.LANDED
    assert "3.0 m" in touchdown.reason and "2.0 m/s^2" in touchdown.reason and "2800 ms" in touchdown.reason


def test_evt_16_a_single_large_acceleration_sample_resets_the_window() -> None:
    """The window is emptied; a new one starts at the next qualifying sample and needs the full 2800 ms."""
    d = descending_driver()
    still(d, 400)
    still(d, 1, accel=2.0)                       # exactly at the limit: disqualifies
    restart_tick = d.tick                        # the next sample starts the new window
    still(d, FALLBACK_SAMPLES - 1)
    assert d.fsm.state is State.DESCENT          # 400 earlier samples did not count
    still(d, 1)
    assert d.fsm.state is State.LANDED and d.events[-1].tick == restart_tick + 2_800

    d = descending_driver()                      # the sign of the acceleration does not matter
    still(d, 400)
    still(d, 1, accel=-7.5)
    still(d, FALLBACK_SAMPLES - 1)
    assert d.fsm.state is State.DESCENT


def test_evt_16_a_barometric_altitude_step_resets_the_window() -> None:
    """A step that takes the range to the 3 m limit starts a new window at the step sample itself."""
    d = descending_driver()
    still(d, 400, baro=0.0)
    step_tick = d.tick
    still(d, 1, baro=3.0)                        # range exactly 3.0 m: not within the limit
    still(d, FALLBACK_SAMPLES - 2, baro=3.0)
    assert d.fsm.state is State.DESCENT
    still(d, 1, baro=3.0)                        # the step sample plus 560 more
    assert d.fsm.state is State.LANDED and d.events[-1].tick == step_tick + 2_800

    d = descending_driver()                      # changes inside the limit do not reset
    first_tick = d.tick
    for k in range(FALLBACK_SAMPLES):
        still(d, 1, baro=2.9 * (k % 2))
    assert d.fsm.state is State.LANDED and d.events[-1].tick == first_tick + 2_800


def test_evt_16_steady_descent_never_qualifies() -> None:
    """Falling at 12.7 m/s with no acceleration: the barometer moves 3 m every 0.24 s."""
    d = descending_driver()
    for k in range(4_000):
        altitude = 500.0 - 12.7 * 0.005 * k
        d.feed(1, force=0.0, velocity=-12.7, altitude=altitude, accel=0.0)
    assert d.fsm.state is State.DESCENT


def test_evt_16_primary_criterion_takes_precedence() -> None:
    """Both criteria satisfied from the same tick: the primary needs 2000 ms, the fallback 2800 ms."""
    d = descending_driver()
    first_tick = d.tick
    d.feed(FALLBACK_SAMPLES, force=9.8, velocity=0.0, altitude=0.0, accel=0.0)
    (touchdown,) = [e for e in d.events if e.event is Event.TOUCHDOWN]
    assert touchdown.tick == first_tick + 2_000
    assert (touchdown.source, touchdown.quality, touchdown.reason) == (EventSource.DETECTED, DataQuality.NOMINAL, None)


def test_evt_16_primary_wins_when_both_complete_on_the_same_tick() -> None:
    d = descending_driver()
    still(d, 160)                                                    # fallback window 160 samples ahead
    d.feed(401, force=9.8, velocity=0.0, altitude=0.0, baro=0.0, accel=0.0)   # both complete on the last one
    (touchdown,) = [e for e in d.events if e.event is Event.TOUCHDOWN]
    assert touchdown.source is EventSource.DETECTED


def no_drag_device_scenario() -> dict:
    scenario = copy.deepcopy(REF01)
    scenario["vehicle"]["drag_device"]["enabled"] = False
    return scenario


def assert_hard_impact_touchdown(record: FlightRecord, label: str) -> None:
    truth = record.truth_events
    assert truth.impact_velocity_mps < -150.0, f"{label}: expected a hard impact"
    (touchdown,) = record.detected(Event.TOUCHDOWN)
    error_s = touchdown.t_sim_s - truth.touchdown_time_s
    low, high = TOLERANCE_S[Event.TOUCHDOWN]
    assert low <= error_s <= high, f"{label}: TOUCHDOWN detected {error_s:+.3f} s from truth"
    assert touchdown.source is EventSource.FALLBACK_CRITERION and touchdown.quality is DataQuality.NOMINAL
    assert record.last_estimate.flight_state is State.LANDED
    print(f"\nEVT-16 {label}: impact {truth.impact_velocity_mps:.1f} m/s, TOUCHDOWN {error_s:+.3f} s after contact")


@pytest.mark.parametrize("seed", range(3))
def test_evt_16_drag_device_disabled(seed: int) -> None:
    """No drag device anywhere: ballistic descent and a hard impact."""
    record = run_flight(seed, scenario=no_drag_device_scenario())
    assert record.commands == [] and record.drag_area_change_tick is None
    assert_hard_impact_touchdown(record, f"drag device disabled, seed {seed}")


@pytest.mark.parametrize("seed", range(3))
def test_evt_16_deployment_commanded_but_ignored_by_the_simulator(seed: int) -> None:
    """Failed deployment: the avionics issues the command, the vehicle does not respond."""
    record = run_flight(seed, scenario=no_drag_device_scenario(), avionics=ref01_avionics_config())
    assert [command for _, command in record.commands] == [VehicleCommand.DEPLOY_DRAG_DEVICE]
    assert record.drag_area_change_tick is None                      # the simulator ignored it
    assert_hard_impact_touchdown(record, f"deployment failed, seed {seed}")


def test_evt_16_ref01_touchdown_stays_primary_and_fallback_never_fires_early() -> None:
    """Eleven nominal flights: touchdown is found by the primary criterion, after true contact."""
    for label, record in [("ideal", flight(0, True)), *((seed, flight(seed)) for seed in range(10))]:
        (touchdown,) = record.detected(Event.TOUCHDOWN)
        assert touchdown.tick == REF01_PINNED_EVENT_TICKS[label][4]
        assert (touchdown.source, touchdown.reason) == (EventSource.DETECTED, None), label
        assert touchdown.t_sim_s > record.truth_events.touchdown_time_s, label


def test_evt_16_configuration() -> None:
    avionics = REF01["avionics"]
    assert (avionics["touchdown_fallback_window_ms"], avionics["touchdown_fallback_acceleration_mps2"]) == (2_800, 2.0)
    assert avionics["touchdown_altitude_range_m"] == 3.0             # reused by the fallback
    config = load_avionics_config(REF01)
    assert (config.touchdown_fallback_window_ms, config.touchdown_fallback_acceleration_mps2) == (2_800, 2.0)

    window, limit = "avionics.touchdown_fallback_window_ms", "avionics.touchdown_fallback_acceleration_mps2"
    for path in (window, limit):                                     # required: no default
        fault = rejected(scenario_with(path, ...))
        assert (fault.reason, fault.field) == (Reason.MISSING_FIELD, path)
    for bad, reason in ((0, Reason.OUT_OF_RANGE), (-2800, Reason.OUT_OF_RANGE), (2800.0, Reason.INVALID_TYPE),
                        ("2800", Reason.INVALID_TYPE), (True, Reason.INVALID_TYPE), (None, Reason.INVALID_TYPE)):
        fault = rejected(scenario_with(window, bad))
        assert (fault.reason, fault.field) == (reason, window)
    for bad, reason in ((0.0, Reason.OUT_OF_RANGE), (-2.0, Reason.OUT_OF_RANGE), (math.nan, Reason.INVALID_TYPE),
                        (math.inf, Reason.INVALID_TYPE), ("2.0", Reason.INVALID_TYPE), (True, Reason.INVALID_TYPE),
                        (None, Reason.INVALID_TYPE)):
        fault = rejected(scenario_with(limit, bad))
        assert (fault.reason, fault.field) == (reason, limit)
    # The fallback window may not be shorter than the primary window (2000 ms): equal is accepted.
    fault = rejected(scenario_with(window, 1_999))
    assert (fault.reason, fault.field) == (Reason.OUT_OF_RANGE, window)
    load_avionics_config(scenario_with(window, 2_000))


# ---------------------------------------------------------------- EVT-17: sample rejection and data quality

UNITS = {"accel": "m/s^2", "gyro": "rad/s", "baro": "Pa", "temp": "K"}


def readings(tick: int, *, accel: float = G0_MPS2, baro: float = P0_PA, temp: float = T0_K, gyro: float = 0.0,
             saturated: tuple[str, ...] = (), drop: tuple[str, ...] = ()) -> list[SensorSample]:
    """The samples due at `tick`: accelerometer and gyroscope every 5 ms, barometer every 20, temperature every 100.

    `saturated` names sources whose sample carries the saturation flag; `drop` names sources that deliver nothing.
    """
    due = [("accel", "z", accel, True), ("gyro", "z", gyro, True),
           ("baro", "pressure", baro, tick % 20 == 0), ("temp", "temperature", temp, tick % 100 == 0)]
    return [
        SensorSample(tick, tick / 1000, sensor, channel, value, UNITS[sensor], f"{sensor}.{channel}" in saturated)
        for sensor, channel, value, is_due in due
        if is_due and f"{sensor}.{channel}" not in drop
    ]


def computer_at(tick_end: int, **kwargs) -> FlightComputer:
    """A flight computer fed clean pad readings for ticks 0 to `tick_end` (exclusive)."""
    computer = FlightComputer(ref01_avionics_config())
    for tick in range(0, tick_end, 5):
        computer.tick(tick, readings(tick, **kwargs))
    return computer


@pytest.mark.parametrize(
    ("source", "field", "clipped", "estimator_input"),
    [
        ("accel.z", "accel", 160.0, True),
        ("baro.pressure", "baro", 110_000.0, True),
        ("temp.temperature", "temp", 330.0, False),
        ("gyro.z", "gyro", 35.0, False),
    ],
)
def test_evt_17_one_saturated_sample_is_rejected_recorded_and_never_used(
    source: str, field: str, clipped: float, estimator_input: bool
) -> None:
    computer, twin = computer_at(2_000), computer_at(2_000)
    output = computer.tick(2_000, readings(2_000, **{field: clipped}, saturated=(source,)))
    reference = twin.tick(2_000, readings(2_000, drop=(source,)))      # the same tick with no sample at all

    # Recorded: which source, when, what it read and why.
    assert output.rejected == (RejectedSample(2_000, 2.0, source, clipped, RejectionReason.SATURATED),)
    # Not used: the estimate is exactly what it would be had the sample never arrived.
    assert output.estimate.altitude_agl_m == reference.estimate.altitude_agl_m
    assert output.estimate.velocity_mps == reference.estimate.velocity_mps
    assert output.estimate.acceleration_mps2 == reference.estimate.acceleration_mps2
    assert abs(output.estimate.altitude_agl_m) < 1.0
    # Marked: the frame lists the source and is DEGRADED. The estimate is DEGRADED only if it uses the source.
    assert output.frame.invalid_sources == (source,)
    assert output.frame.data_quality is DataQuality.DEGRADED
    assert output.estimate.quality is (DataQuality.DEGRADED if estimator_input else DataQuality.NOMINAL)
    # Telemetry carries the raw clipped reading, not a stale or invented one.
    carried = {"baro.pressure": output.frame.pressure_pa, "temp.temperature": output.frame.temperature_k,
               "gyro.z": output.frame.roll_rate_radps}
    if source in carried:
        assert carried[source] == clipped

    # A valid replacement removes the source at once; DEGRADED persists for 1000 ms after the rejection.
    frames = {}
    for tick in range(2_005, 3_101, 5):
        result = computer.tick(tick, readings(tick))
        assert result.rejected == ()
        if result.frame is not None:
            frames[tick] = result.frame
    next_valid_tick = {"accel.z": 2_005, "gyro.z": 2_005, "baro.pressure": 2_020, "temp.temperature": 2_100}[source]
    for tick, frame in frames.items():
        assert frame.invalid_sources == ((source,) if tick < next_valid_tick else ()), tick
        assert frame.data_quality is (DataQuality.DEGRADED if tick < 3_000 else DataQuality.NOMINAL), tick


def test_evt_17_acceleration_is_held_for_at_most_100_ms_then_velocity_only() -> None:
    """Accelerometer saturated from tick 2000 on. Last valid sample: tick 1995, 2.0 m/s^2 above gravity."""
    computer = computer_at(1_000)
    for tick in range(1_000, 2_000, 5):
        computer.tick(tick, readings(tick, accel=G0_MPS2 + 2.0))
    outputs = {}
    for tick in range(2_000, 2_300, 5):
        outputs[tick] = computer.tick(tick, readings(tick, accel=160.0, saturated=("accel.z",)))
        assert outputs[tick].rejected[0].source == "accel.z"
        assert outputs[tick].events == ()                           # a saturated reading never counts for LIFTOFF

    def velocity_step(tick: int) -> float:                           # on ticks with no barometer correction
        return outputs[tick].estimate.velocity_mps - outputs[tick - 5].estimate.velocity_mps

    # Held: age of the last valid sample is at most 100 ms up to tick 2095. The prediction still uses 2.0 m/s^2.
    for tick in (2_005, 2_050, 2_085, 2_090, 2_095):
        assert outputs[tick].estimate.quality is DataQuality.DEGRADED, tick
    for tick in (2_085, 2_090, 2_095):
        assert velocity_step(tick) == pytest.approx(2.0 * 0.005, abs=1e-12)
    # Expired at tick 2100 (age 105 ms): velocity-only prediction and an UNRELIABLE estimate.
    for tick in (2_100, 2_105, 2_200, 2_295):
        assert outputs[tick].estimate.quality is DataQuality.UNRELIABLE, tick
    for tick in (2_105, 2_110, 2_115, 2_205):
        assert velocity_step(tick) == 0.0
        step_m = outputs[tick].estimate.altitude_agl_m - outputs[tick - 5].estimate.altitude_agl_m
        assert step_m == pytest.approx(outputs[tick - 5].estimate.velocity_mps * 0.005, abs=1e-12)
    assert outputs[2_250].frame.data_quality is DataQuality.UNRELIABLE
    assert outputs[2_250].frame.invalid_sources == ("accel.z",)

    # Recovery: before F1 the estimate went DEGRADED, then NOMINAL 1000 ms after the last rejection. Since
    # F1 (docs/F1_DECISIONS.md 6e.1, 6e.2) an expired hold makes the dead reckoning invalid, which raises the
    # latched UNCHECKABLE disagreement flag at tick 2100: the estimate stays UNRELIABLE for the rest of the flight.
    assert computer.disagreement_flag.tick == 2_100 and computer.disagreement_flag.reason.value == "UNCHECKABLE"
    qualities = {}
    for tick in range(2_300, 3_305, 5):
        qualities[tick] = computer.tick(tick, readings(tick, accel=G0_MPS2 + 2.0)).estimate.quality
    assert set(qualities.values()) == {DataQuality.UNRELIABLE}


@pytest.mark.parametrize(("last_saturated_tick", "expect_unreliable"), [(2_960, False), (3_200, True)])
def test_evt_17_barometer_saturation_skips_corrections_and_becomes_unreliable_after_1000_ms(
    last_saturated_tick: int, expect_unreliable: bool
) -> None:
    """Last valid barometer sample at tick 1980. UNRELIABLE needs more than 1000 ms without one: tick 2985."""
    computer = computer_at(2_000)
    unreliable_ticks = []
    for tick in range(2_000, 3_400, 5):
        saturated = tick <= last_saturated_tick
        output = computer.tick(tick, readings(tick, baro=110_000.0 if saturated else P0_PA,
                                              saturated=("baro.pressure",) if saturated else ()))
        assert abs(output.estimate.altitude_agl_m) < 1.0             # 110 000 Pa would read about -700 m
        if output.estimate.quality is DataQuality.UNRELIABLE:
            unreliable_ticks.append(tick)
        if output.frame is not None and saturated:
            assert output.frame.pressure_pa == 110_000.0 and "baro.pressure" in output.frame.invalid_sources
            assert output.frame.data_quality is not DataQuality.NOMINAL
    if expect_unreliable:
        assert unreliable_ticks[0] == 2_985                          # 1980 + 1000 is still allowed; 2985 is not
        assert unreliable_ticks[-1] == 3_215                         # the valid sample at 3220 ends it
    else:
        assert unreliable_ticks == []


def test_evt_17_saturated_accelerometer_samples_do_not_count_toward_liftoff() -> None:
    computer = computer_at(1_000)
    tick = 1_000

    def feed(count: int, accel: float, saturated: bool = False) -> list:
        nonlocal tick
        events = []
        for _ in range(count):
            events += computer.tick(tick, readings(tick, accel=accel, saturated=("accel.z",) if saturated else ())).events
            tick += 5
        return events

    assert feed(40, 160.0, saturated=True) == [] and computer.flight_state is State.PAD_IDLE
    assert feed(9, 40.0) == []                                       # nine valid samples above the 15 m/s^2 threshold
    assert feed(1, 160.0, saturated=True) == []                      # a rejected sample resets the run
    assert feed(9, 40.0) == [] and computer.flight_state is State.PAD_IDLE
    (liftoff,) = feed(1, 40.0)                                       # the tenth valid sample in a row
    assert liftoff.event is Event.LIFTOFF and liftoff.quality is DataQuality.NOMINAL
    assert computer.flight_state is State.BOOST


def test_evt_17_calibration_excludes_rejected_samples() -> None:
    computer = FlightComputer(ref01_avionics_config())
    rejected = []
    for tick in range(0, 1_005, 5):
        bad = tick in (200, 400, 600)
        rejected += computer.tick(tick, readings(tick, baro=110_000.0 if bad else P0_PA, accel=G0_MPS2 + 0.2,
                                                 saturated=("baro.pressure",) if bad else ())).rejected
    assert [(r.tick, r.source, r.reason) for r in rejected] == [
        (200, "baro.pressure", RejectionReason.SATURATED), (400, "baro.pressure", RejectionReason.SATURATED),
        (600, "baro.pressure", RejectionReason.SATURATED),
    ]
    assert computer.calibration.reference_pressure_pa == P0_PA       # the three 110 000 Pa samples are not in the mean
    assert computer.calibration.accel_bias_mps2 == pytest.approx(0.2, abs=1e-9)


@pytest.mark.parametrize("source", ["accel.z", "baro.pressure", "temp.temperature"])
def test_evt_17_calibration_with_no_valid_sample_is_a_configuration_fault(source: str) -> None:
    computer = FlightComputer(ref01_avionics_config())
    for tick in range(0, 1_000, 5):
        computer.tick(tick, readings(tick, saturated=(source,)))
    with pytest.raises(ConfigurationFault) as raised:
        computer.tick(1_000, readings(1_000, saturated=(source,)))
    assert raised.value.reason is Reason.NO_VALID_CALIBRATION_SAMPLES
    assert source in str(raised.value)


def test_evt_17_non_finite_samples_are_rejected_and_telemetry_keeps_the_last_finite_value() -> None:
    computer = computer_at(2_000, gyro=0.25)
    output = computer.tick(2_000, readings(2_000, gyro=math.nan, baro=math.inf))
    assert [(r.source, r.reason) for r in output.rejected] == [
        ("gyro.z", RejectionReason.NON_FINITE), ("baro.pressure", RejectionReason.NON_FINITE),
    ]
    assert math.isnan(output.rejected[0].value) and math.isinf(output.rejected[1].value)
    assert output.frame.roll_rate_radps == 0.25                       # the last finite reading
    assert output.frame.pressure_pa == P0_PA
    assert output.frame.invalid_sources == ("baro.pressure", "gyro.z")   # explicitly marked invalid
    assert output.frame.data_quality is DataQuality.DEGRADED
    assert math.isfinite(output.estimate.altitude_agl_m) and abs(output.estimate.altitude_agl_m) < 1.0
    for value in (output.frame.altitude_m, output.frame.velocity_mps, output.frame.acceleration_mps2,
                  output.frame.pressure_pa, output.frame.temperature_k, output.frame.roll_rate_radps):
        assert math.isfinite(value)


def test_evt_17_source_that_never_delivered_a_finite_reading_is_sent_as_zero_and_marked_invalid() -> None:
    computer = FlightComputer(ref01_avionics_config())
    output = computer.tick(0, readings(0, temp=math.nan))
    assert output.frame.temperature_k == 0.0
    assert output.frame.invalid_sources == ("temp.temperature",)
    assert output.frame.data_quality is DataQuality.DEGRADED
    assert output.estimate.quality is DataQuality.NOMINAL             # temperature does not feed the estimator


def test_evt_17_invalid_sources_keep_the_documented_order() -> None:
    computer = computer_at(2_000)
    output = computer.tick(2_000, readings(2_000, saturated=("gyro.z", "temp.temperature", "baro.pressure", "accel.z")))
    assert output.frame.invalid_sources == ("accel.z", "baro.pressure", "temp.temperature", "gyro.z")
    assert len(output.rejected) == 4


def test_evt_17_ref01_has_no_rejections_and_is_nominal_throughout() -> None:
    for record in (flight(0, True), *(flight(seed) for seed in range(10))):
        assert record.rejected_samples == 0
        assert record.non_nominal_ticks == 0
        assert all(e.quality is DataQuality.NOMINAL for e in record.events)


def test_evt_17_rejected_samples_reset_the_state_machine_counters() -> None:
    d = boosting_driver()                                            # primary burnout: 10 samples below zero
    d.feed(9, force=-2.0, velocity=200.0)
    d.feed(1, accel_rejected=True, velocity=200.0)
    d.feed(9, force=-2.0, velocity=200.0)
    assert d.fsm.state is State.BOOST
    d.feed(1, force=-2.0, velocity=200.0)
    assert d.fsm.state is State.COAST and d.events[-1].source is EventSource.DETECTED

    d = boosting_driver()                                            # fallback burnout: 20 samples below 1.0
    d.feed(19, force=0.5, velocity=200.0)
    d.feed(1, accel_rejected=True, velocity=200.0)
    d.feed(19, force=0.5, velocity=200.0)
    assert d.fsm.state is State.BOOST
    d.feed(1, force=0.5, velocity=200.0)
    assert d.fsm.state is State.COAST and d.events[-1].source is EventSource.FALLBACK_CRITERION

    for rejected in ("accel_rejected", "baro_rejected"):             # touchdown fallback: 561 still samples
        d = descending_driver()
        still(d, 400)
        d.feed(1, force=9.8, velocity=-150.0, altitude=-40.0, baro=0.0, accel=0.0, **{rejected: True})
        still(d, FALLBACK_SAMPLES - 1)
        assert d.fsm.state is State.DESCENT, rejected
        still(d, 1)
        assert d.fsm.state is State.LANDED, rejected


# ---------------------------------------------------------------- EVT-19: the three deployment paths

UNRELIABLE, DEGRADED, NOMINAL = DataQuality.UNRELIABLE, DataQuality.DEGRADED, DataQuality.NOMINAL


def apogee_driver(quality: DataQuality, **overrides) -> tuple[Driver, int, object]:
    """Coast to a barometric peak of 2000 m, then record a detected APOGEE of the given estimate quality."""
    d, burnout_tick = coasting_driver(**overrides)
    d.feed(250, force=0.0, velocity=50.0, altitude=2_000.0)
    d.feed(10, force=0.0, velocity=-1.0, altitude=2_000.0, quality=quality)
    apogee = d.events[-1]
    assert (apogee.event, apogee.source, apogee.quality) == (Event.APOGEE, EventSource.DETECTED, quality)
    assert d.fsm.state is State.DESCENT and d.commands == []
    return d, burnout_tick, apogee


def descend(d: Driver, count: int, baro: float, **kwargs) -> None:
    d.feed(count, force=0.0, velocity=-12.0, altitude=baro, quality=kwargs.pop("quality", UNRELIABLE), **kwargs)


def deployed_events(d: Driver) -> list:
    return [e for e in d.events if e.event is Event.DRAG_DEVICE_DEPLOYED]


@pytest.mark.parametrize("quality", [NOMINAL, DEGRADED])
def test_evt_19_reliable_apogee_deploys_by_the_normal_path_and_cancels_the_timer(quality: DataQuality) -> None:
    d, burnout_tick, apogee = apogee_driver(quality)
    descend(d, 199, 1_990.0, quality=quality)
    assert d.commands == []
    descend(d, 1, 1_990.0, quality=quality)
    assert d.commands == [(apogee.tick + 1_000, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    (event,) = deployed_events(d)
    assert (event.source, event.quality, event.reason) == (EventSource.DETECTED, quality, None)
    descend(d, 5_000, 1_500.0, quality=quality)                       # far past burnout + 20 s
    assert d.tick > burnout_tick + 20_000 and len(d.commands) == 1 and len(deployed_events(d)) == 1


def test_evt_19_reliable_apogee_is_latched_even_if_the_estimate_degrades_during_the_delay() -> None:
    d, _, apogee = apogee_driver(NOMINAL)
    descend(d, 200, 1_990.0, quality=UNRELIABLE, baro_rejected=True)
    assert d.commands == [(apogee.tick + 1_000, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert deployed_events(d)[0].source is EventSource.DETECTED


def test_evt_19_unreliable_apogee_does_not_deploy_by_the_normal_path() -> None:
    """The delay elapses with the barometer only 5 m below the peak: nothing may be commanded."""
    d, burnout_tick, apogee = apogee_driver(UNRELIABLE)
    descend(d, 1_000, 1_995.0)
    assert d.tick > apogee.tick + 1_000 and d.tick < burnout_tick + 20_000
    assert d.commands == [] and deployed_events(d) == []


def test_evt_19_barometric_confirmation_deploys_after_five_consecutive_valid_samples() -> None:
    d, _, apogee = apogee_driver(UNRELIABLE)
    descend(d, 300, 1_995.0)                                          # delay elapsed, 5 m below the 2000 m peak
    descend(d, 4, 1_989.0)                                            # four samples 11 m below
    assert d.commands == []
    confirm_tick = d.tick
    descend(d, 1, 1_989.0)                                            # the fifth
    assert d.commands == [(confirm_tick, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    (event,) = deployed_events(d)
    assert (event.source, event.quality) == (EventSource.BAROMETRIC_CONFIRMATION, DEGRADED)
    assert "5 consecutive valid barometer samples" in event.reason and "2000.0 m" in event.reason
    # The original APOGEE is preserved exactly: later confirmation does not rewrite it.
    assert [e for e in d.events if e.event is Event.APOGEE] == [apogee]
    assert (apogee.source, apogee.quality) == (EventSource.DETECTED, UNRELIABLE)
    descend(d, 5_000, 1_500.0)                                        # the timer does not deploy again
    assert len(d.commands) == 1


def test_evt_19_descent_of_exactly_10_m_qualifies_and_less_does_not() -> None:
    d, _, _ = apogee_driver(UNRELIABLE)
    descend(d, 300, 1_990.01)
    assert d.commands == []
    descend(d, 5, 1_990.0)
    assert len(d.commands) == 1


def test_evt_19_barometric_confirmation_waits_for_the_deployment_delay() -> None:
    d, _, apogee = apogee_driver(UNRELIABLE)
    descend(d, 199, 1_980.0)                                          # confirmed long before the delay
    assert d.commands == []
    descend(d, 1, 1_980.0)
    assert d.commands == [(apogee.tick + 1_000, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert deployed_events(d)[0].source is EventSource.BAROMETRIC_CONFIRMATION


@pytest.mark.parametrize("breaker", [{"baro_rejected": True}, {"baro": 1_995.0}])
def test_evt_19_an_invalid_or_non_qualifying_sample_resets_the_confirmation(breaker: dict) -> None:
    d, _, _ = apogee_driver(UNRELIABLE)
    descend(d, 300, 1_995.0)
    descend(d, 4, 1_989.0)
    descend(d, 1, breaker.get("baro", 1_989.0), baro_rejected=breaker.get("baro_rejected", False))
    descend(d, 4, 1_989.0)
    assert d.commands == []                                           # eight qualifying samples, never five in a row
    descend(d, 1, 1_989.0)
    assert len(d.commands) == 1 and deployed_events(d)[0].source is EventSource.BAROMETRIC_CONFIRMATION


def test_evt_19_barometer_invalid_throughout_falls_back_to_the_timer_in_descent() -> None:
    d, burnout_tick, apogee = apogee_driver(UNRELIABLE)
    descend(d, 3_739, 1_500.0, baro_rejected=True)
    assert d.tick - 5 == burnout_tick + 19_995 and d.commands == []
    descend(d, 1, 1_500.0, baro_rejected=True)
    assert d.commands == [(burnout_tick + 20_000, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    (event,) = deployed_events(d)
    assert (event.source, event.quality) == (EventSource.BACKUP_TIMER, UNRELIABLE)
    assert "unreliable" in event.reason and f"tick {burnout_tick}" in event.reason
    assert [e for e in d.events if e.event is Event.APOGEE] == [apogee]   # no second, inferred apogee
    descend(d, 1_000, 1_400.0, baro_rejected=True)
    assert len(d.commands) == 1


def test_evt_19_a_single_pressure_spike_cannot_establish_the_peak() -> None:
    d, _ = coasting_driver()
    d.feed(100, force=0.0, velocity=50.0, altitude=1_000.0)
    d.feed(1, force=0.0, velocity=50.0, altitude=1_000.0, baro=2_500.0)   # one spike: 1500 m too high
    d.feed(150, force=0.0, velocity=50.0, altitude=1_000.0)
    d.feed(10, force=0.0, velocity=-1.0, altitude=1_000.0, quality=UNRELIABLE)
    assert d.fsm._baro_peak_m == 1_000.0
    descend(d, 400, 1_000.0)                                          # level flight after the delay
    assert d.commands == []                                           # a raw maximum would have "confirmed" 1500 m of descent

    d, _ = coasting_driver()                                          # two spikes inside five samples: still no effect
    d.feed(100, force=0.0, velocity=50.0, altitude=1_000.0)
    d.feed(2, force=0.0, velocity=50.0, altitude=1_000.0, baro=2_500.0)
    d.feed(150, force=0.0, velocity=50.0, altitude=1_000.0)
    assert d.fsm._baro_peak_m == 1_000.0


def test_evt_19_known_limit_three_bad_samples_in_five_do_move_the_peak() -> None:
    """Documented limitation of a five-sample median: a sustained error is not a spike."""
    d, _ = coasting_driver()
    d.feed(100, force=0.0, velocity=50.0, altitude=1_000.0)
    d.feed(3, force=0.0, velocity=50.0, altitude=1_000.0, baro=2_500.0)
    assert d.fsm._baro_peak_m == 2_500.0


def test_evt_19_an_invalid_sample_cannot_help_establish_the_peak() -> None:
    d, _ = coasting_driver()
    for _ in range(40):                                               # never five valid samples in a row
        d.feed(4, force=0.0, velocity=50.0, altitude=1_000.0)
        d.feed(1, force=0.0, velocity=50.0, altitude=1_000.0, baro_rejected=True)
    assert d.fsm._baro_peak_m is None or d.fsm._baro_peak_m == 0.0     # only the pre-coast pad value, if any


def test_evt_19_the_peak_keeps_rising_after_a_falsely_early_apogee() -> None:
    """Unreliable apogee recorded at 1000 m while still climbing to 1500 m: confirmation is measured from 1500 m."""
    d, _ = coasting_driver()
    d.feed(250, force=0.0, velocity=50.0, altitude=1_000.0)
    d.feed(10, force=0.0, velocity=-1.0, altitude=1_000.0, quality=UNRELIABLE)
    descend(d, 100, 1_500.0)
    assert d.fsm._baro_peak_m == 1_500.0
    descend(d, 300, 1_495.0)                                          # 495 m above the altitude at "apogee"
    assert d.commands == []
    descend(d, 5, 1_490.0)
    assert len(d.commands) == 1 and deployed_events(d)[0].source is EventSource.BAROMETRIC_CONFIRMATION


def test_evt_19_confirmation_wins_when_it_and_the_timer_complete_on_the_same_tick() -> None:
    def run(**overrides) -> tuple[Driver, int]:
        d, burnout_tick, _ = apogee_driver(UNRELIABLE, **overrides)
        descend(d, 300, 1_995.0)
        descend(d, 5, 1_989.0)
        return d, burnout_tick

    d, burnout_tick = run()
    ((confirm_tick, _),) = d.commands
    d, burnout_tick = run(backup_deployment_ticks=confirm_tick - burnout_tick)   # timer expires on that same tick
    assert d.commands == [(confirm_tick, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert deployed_events(d)[0].source is EventSource.BAROMETRIC_CONFIRMATION
    descend(d, 100, 1_900.0)
    assert len(d.commands) == 1                                       # the timer does not fire afterwards

    d, burnout_tick = run(backup_deployment_ticks=confirm_tick - burnout_tick - 5)   # one tick earlier: the timer wins
    assert d.commands == [(confirm_tick - 5, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert deployed_events(d)[0].source is EventSource.BACKUP_TIMER


def test_evt_19_unreliable_apogee_detected_on_the_tick_the_timer_expires() -> None:
    d, burnout_tick = coasting_driver(backup_deployment_ticks=2_000)
    d.feed(390, force=0.0, velocity=50.0, altitude=900.0)
    d.feed(10, force=0.0, velocity=-1.0, altitude=900.0, quality=UNRELIABLE)
    expiry_tick = burnout_tick + 2_000
    apogee, deployed = d.events[-2], d.events[-1]
    assert (apogee.event, apogee.tick, apogee.source, apogee.quality) == (Event.APOGEE, expiry_tick, EventSource.DETECTED, UNRELIABLE)
    assert (deployed.event, deployed.tick, deployed.source, deployed.quality) == (
        Event.DRAG_DEVICE_DEPLOYED, expiry_tick, EventSource.BACKUP_TIMER, UNRELIABLE)
    assert d.commands == [(expiry_tick, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert [e.event for e in d.events].count(Event.APOGEE) == 1       # detected, not inferred


def test_evt_19_timer_in_descent_commands_nothing_when_the_drag_device_is_disabled() -> None:
    d, burnout_tick, _ = apogee_driver(UNRELIABLE, drag_device_enabled=False)
    descend(d, 4_200, 1_500.0, baro_rejected=True)
    assert d.tick > burnout_tick + 20_000
    assert d.commands == [] and deployed_events(d) == []


def test_evt_19_event_quality_follows_the_evidence_of_each_event() -> None:
    """LIFTOFF and BURNOUT use valid accelerometer samples; APOGEE and primary TOUCHDOWN use the estimate."""
    d = Driver(machine())
    d.feed(10, force=30.0, quality=UNRELIABLE)
    d.feed(10, force=-2.0, velocity=200.0, quality=UNRELIABLE)
    assert [(e.event, e.quality) for e in d.events] == [(Event.LIFTOFF, NOMINAL), (Event.BURNOUT, NOMINAL)]
    d.feed(250, force=0.0, velocity=50.0, altitude=2_000.0)
    d.feed(10, force=0.0, velocity=-1.0, altitude=2_000.0, quality=DEGRADED)
    assert d.events[-1].quality is DEGRADED                            # APOGEE: the estimate's quality at detection
    d.feed(401, force=9.8, velocity=0.0, altitude=0.0, quality=UNRELIABLE)
    touchdown = d.events[-1]
    assert (touchdown.event, touchdown.source, touchdown.quality) == (Event.TOUCHDOWN, EventSource.DETECTED, UNRELIABLE)

    d = descending_driver()                                           # fallback TOUCHDOWN: valid barometer and accelerometer
    d.feed(FALLBACK_SAMPLES, force=9.8, velocity=-150.0, altitude=-40.0, baro=0.0, accel=0.0, quality=UNRELIABLE)
    assert (d.events[-1].source, d.events[-1].quality) == (EventSource.FALLBACK_CRITERION, NOMINAL)


def test_evt_19_closed_loop_barometer_out_of_range_above_1_km() -> None:
    """REF-01, seed 0, with a barometer that saturates below 90 000 Pa (above about 1 km).

    The barometer is invalid from mid-coast until the descent passes 1 km again, so the estimate is
    UNRELIABLE at apogee, barometric confirmation is impossible there, and only the timer can deploy.
    """
    scenario = copy.deepcopy(REF01)
    scenario["sensors"]["baro"]["range_pa"] = [90_000.0, 110_000.0]
    record = run_flight(0, scenario=scenario)
    truth = record.truth_events
    burnout_tick = record.detected(Event.BURNOUT)[0].tick
    (apogee,) = record.detected(Event.APOGEE)
    (deployed,) = record.detected(Event.DRAG_DEVICE_DEPLOYED)
    assert record.rejected_samples > 500 and record.non_nominal_ticks > 0
    assert (apogee.source, apogee.quality) == (EventSource.DETECTED, DataQuality.UNRELIABLE)
    assert (deployed.source, deployed.quality, deployed.tick) == (EventSource.BACKUP_TIMER, DataQuality.UNRELIABLE, burnout_tick + 20_000)
    assert record.commands == [(burnout_tick + 20_000, VehicleCommand.DEPLOY_DRAG_DEVICE)]
    assert deployed.tick > truth.apogee_tick                          # after true apogee, not during ascent
    assert record.last_estimate.flight_state is State.LANDED
    print(f"\nEVT-19 closed loop: apogee detected {apogee.t_sim_s - truth.apogee_tick / 1000:+.3f} s from truth, "
          f"deployed {deployed.t_sim_s - truth.apogee_tick / 1000:+.3f} s after true apogee, "
          f"touchdown {record.detection_time_s(Event.TOUCHDOWN) - truth.touchdown_time_s:+.3f} s after contact, "
          f"{record.rejected_samples} samples rejected")


def test_evt_19_confirmation_configuration() -> None:
    avionics = REF01["avionics"]
    assert (avionics["deploy_confirm_baro_descent_m"], avionics["deploy_confirm_samples"]) == (10.0, 5)
    descent, samples = "avionics.deploy_confirm_baro_descent_m", "avionics.deploy_confirm_samples"
    for path in (descent, samples):                                   # required: no default
        fault = rejected(scenario_with(path, ...))
        assert (fault.reason, fault.field) == (Reason.MISSING_FIELD, path)
    for bad, reason in ((0.0, Reason.OUT_OF_RANGE), (-10.0, Reason.OUT_OF_RANGE), (math.nan, Reason.INVALID_TYPE),
                        (math.inf, Reason.INVALID_TYPE), ("10", Reason.INVALID_TYPE), (True, Reason.INVALID_TYPE)):
        fault = rejected(scenario_with(descent, bad))
        assert (fault.reason, fault.field) == (reason, descent)
    for bad, reason in ((0, Reason.OUT_OF_RANGE), (-5, Reason.OUT_OF_RANGE), (5.0, Reason.INVALID_TYPE),
                        ("5", Reason.INVALID_TYPE), (True, Reason.INVALID_TYPE)):
        fault = rejected(scenario_with(samples, bad))
        assert (fault.reason, fault.field) == (reason, samples)
    load_avionics_config(scenario_with(samples, 1))


# ---------------------------------------------------------------- EVT-07

def test_evt_07_pressure_spike_during_boost_does_not_trigger_apogee() -> None:
    """A single +5000 Pa barometer sample (about 450 m of false descent) at 5.0 s, mid-boost."""
    spiked = run_flight(0, pressure_spike=(5_000, 5_000.0), stop_tick=12_000)
    clean = run_flight(0, stop_tick=12_000)
    assert [e.event for e in spiked.events] == [Event.LIFTOFF, Event.BURNOUT]
    assert spiked.last_estimate.flight_state is State.COAST
    assert [e.tick for e in spiked.events] == [e.tick for e in clean.events]


# ---------------------------------------------------------------- EVT-09

def sample(tick: int, sensor: str, channel: str, value: float, unit: str) -> SensorSample:
    return SensorSample(tick, tick / 1000, sensor, channel, value, unit, False)


def test_evt_09_calibration_recovers_known_injected_values() -> None:
    """Noise and quantization off: p_ref, T_ref and b_a are recovered within 1e-9."""
    computer = FlightComputer(ref01_avionics_config())
    for tick in range(0, 1_001, 5):
        samples = [sample(tick, "accel", "z", G0_MPS2 + 0.137, "m/s^2"), sample(tick, "gyro", "z", 0.0, "rad/s")]
        if tick % 20 == 0:
            samples.append(sample(tick, "baro", "pressure", 101_325.0 + 41.5, "Pa"))
        if tick % 100 == 0:
            samples.append(sample(tick, "temp", "temperature", 288.15 - 0.62, "K"))
        output = computer.tick(tick, samples)
        assert (computer.calibration is None) == (tick < 1_000)
        assert output.flight_state is State.PAD_IDLE
    result = computer.calibration
    assert result.reference_pressure_pa == pytest.approx(101_366.5, abs=1e-9)
    assert result.reference_temperature_k == pytest.approx(287.53, abs=1e-9)
    assert result.accel_bias_mps2 == pytest.approx(0.137, abs=1e-9)


@pytest.mark.parametrize("seed", range(20))
def test_evt_09_calibration_with_default_noise_is_within_4_standard_errors(seed: int) -> None:
    """Standard error = sqrt(sigma^2 + resolution^2 / 12) / sqrt(N), with N = 50, 10 and 200 samples."""
    suite = SensorSuite(suite_config_from_scenario(REF01["sensors"]), master_seed=seed)
    computer = FlightComputer(ref01_avionics_config())
    for tick in range(0, 1_001, 5):
        computer.tick(tick, suite.sample(pad_truth(tick)))
    bias = {(s.sensor_id, c.channel): c.bias for s in suite.sensors for c in s.channels}
    result = computer.calibration

    def standard_error(sigma: float, resolution: float, n: int) -> float:
        return math.sqrt(sigma**2 + resolution**2 / 12) / math.sqrt(n)

    assert abs(result.reference_pressure_pa - (P0_PA + bias[("baro", "pressure")])) <= 4 * standard_error(3.0, 1.0, 50)
    assert abs(result.reference_temperature_k - (T0_K + bias[("temp", "temperature")])) <= 4 * standard_error(0.05, 0.01, 10)
    assert abs(result.accel_bias_mps2 - bias[("accel", "z")]) <= 4 * standard_error(0.05, 0.005, 200)


def test_calibration_fails_explicitly_when_a_required_sensor_gives_no_samples() -> None:
    calibration = PadCalibration()
    calibration.add([sample(0, "accel", "z", 9.8, "m/s^2"), sample(0, "temp", "temperature", 288.0, "K")])
    with pytest.raises(ValueError, match=r"baro\.pressure"):
        calibration.result()


# ---------------------------------------------------------------- EVT-10, EVT-11

def pooled_rms(records: list[FlightRecord]) -> tuple[float, float]:
    errors = [pair for record in records for pair in record.ascent_errors]
    return (math.sqrt(sum(h * h for h, _ in errors) / len(errors)), math.sqrt(sum(v * v for _, v in errors) / len(errors)))


def test_evt_10_estimator_accuracy_seeds_0_to_9() -> None:
    altitude_rms, velocity_rms = pooled_rms([flight(seed) for seed in range(10)])
    assert altitude_rms <= 5.0 and velocity_rms <= 3.0


@pytest.mark.slow
def test_evt_10_estimator_accuracy_seeds_0_to_99() -> None:
    """Liftoff to apogee, all samples of all 100 seeds pooled: altitude RMS <= 5 m, velocity RMS <= 3 m/s."""
    records = [flight(seed) for seed in range(100)]
    altitude_rms, velocity_rms = pooled_rms(records)
    per_seed = [pooled_rms([record]) for record in records]
    print(f"\nEVT-10 pooled RMS over seeds 0-99: altitude {altitude_rms:.3f} m, velocity {velocity_rms:.3f} m/s")
    print(f"  per-seed altitude RMS: min {min(h for h, _ in per_seed):.3f}, median "
          f"{sorted(h for h, _ in per_seed)[50]:.3f}, max {max(h for h, _ in per_seed):.3f} m;"
          f" seeds above 5 m: {sum(h > 5.0 for h, _ in per_seed)}")
    print(f"  per-seed velocity RMS: max {max(v for _, v in per_seed):.3f} m/s")
    assert altitude_rms <= 5.0 and velocity_rms <= 3.0


@pytest.mark.parametrize("altitude_m", [0.0, 500.0, 2_000.0])
def test_evt_11_barometric_altitude_formula(altitude_m: float) -> None:
    """Ideal pressure from the atmosphere model, pad at sea level: h returned within 1.0 m.

    The expected difference, about h^2/r0 = 0.63 m at 2000 m, is the geopotential approximation.
    """
    pressure_pa = StandardAtmosphere().air(altitude_m).pressure_pa
    estimate_m = baro_altitude_agl_m(pressure_pa, P0_PA, T0_K)
    assert abs(estimate_m - altitude_m) <= 1.0
    if altitude_m == 2_000.0:
        assert altitude_m - estimate_m == pytest.approx(2_000.0**2 / 6_356_766, abs=0.01)


# ---------------------------------------------------------------- estimator, frames, configuration

def test_complementary_filter_follows_the_documented_equations() -> None:
    f = ComplementaryFilter(gain_h=0.2, gain_v_hz=0.5, period_s=0.005)
    f.altitude_agl_m, f.velocity_mps = 10.0, 4.0
    f.predict(2.0)                       # h += v*T + a*T^2/2 ; v += a*T
    assert f.altitude_agl_m == pytest.approx(10.0 + 4.0 * 0.005 + 0.5 * 2.0 * 0.005**2, abs=1e-15)
    assert f.velocity_mps == pytest.approx(4.01, abs=1e-15)
    h, v = f.altitude_agl_m, f.velocity_mps
    f.correct(h + 3.0)                   # r = 3 m: h += K_h*r ; v += K_v*r
    assert f.altitude_agl_m == pytest.approx(h + 0.6, abs=1e-12)
    assert f.velocity_mps == pytest.approx(v + 1.5, abs=1e-12)


def test_telemetry_frames_every_50_ms_and_zero_estimates_until_calibrated(ideal_flight: FlightRecord) -> None:
    assert ideal_flight.frame_ticks == list(range(0, ideal_flight.final_tick + 1, 50))   # FR-AVI-07
    suite = SensorSuite(suite_config_from_scenario(REF01["sensors"]), master_seed=3)
    computer = FlightComputer(ref01_avionics_config())
    for tick in range(0, 1_000, 5):
        output = computer.tick(tick, suite.sample(pad_truth(tick)))
        estimate = output.estimate
        assert (estimate.altitude_agl_m, estimate.velocity_mps, estimate.acceleration_mps2) == (0.0, 0.0, 0.0)
        if output.frame is not None:
            assert (output.frame.altitude_m, output.frame.velocity_mps, output.frame.acceleration_mps2) == (0.0, 0.0, 0.0)
            assert output.frame.pressure_pa > 0.0 and output.frame.temperature_k > 0.0


def test_avionics_configuration_is_read_strictly() -> None:
    config = ref01_avionics_config()
    assert (config.rate_hz, config.telemetry_rate_hz, config.debounce_samples) == (200, 20, 10)
    assert (config.drag_device_enabled, config.deploy_delay_after_apogee_ms) == (True, 1000)
    section = dict(REF01["avionics"])
    del section["estimator_gain_h"]
    with pytest.raises(ValueError, match="estimator_gain_h"):
        avionics_config_from_scenario(section, REF01["vehicle"]["drag_device"])
    with pytest.raises(ValueError, match="unknown"):
        avionics_config_from_scenario(dict(REF01["avionics"], kalman=True), REF01["vehicle"]["drag_device"])
    with pytest.raises(ValueError):
        FlightComputer(ref01_avionics_config(telemetry_rate_hz=125))
    with pytest.raises(ValueError):
        FlightComputer(ref01_avionics_config()).tick(3, [])


# ---------------------------------------------------------------- EVT-14: configuration validation

def scenario_with(path: str, value: object) -> dict:
    """A copy of REF-01 with one dotted field replaced. `...` as the value deletes the field."""
    scenario = copy.deepcopy(REF01)
    node = scenario
    *parents, leaf = path.split(".")
    for key in parents:
        node = node[key]
    if value is ...:
        del node[leaf]
    else:
        node[leaf] = value
    return scenario


def rejected(scenario: dict) -> ConfigurationFault:
    with pytest.raises(ConfigurationFault) as raised:
        load_avionics_config(scenario)
    assert raised.value.code is FaultCode.INVALID_CONFIGURATION
    assert isinstance(raised.value, ValueError)
    assert raised.value.field in str(raised.value) and raised.value.reason.value in str(raised.value)
    return raised.value


def test_evt_14_ref01_is_accepted_unchanged() -> None:
    config = load_avionics_config(REF01)
    # The loader adds only the derived F1-D6 liftoff timer: burn duration 3200 ms + backup delay 20000 ms.
    assert config == dataclasses.replace(
        avionics_config_from_scenario(REF01["avionics"], REF01["vehicle"]["drag_device"]), liftoff_backup_delay_ms=23_200)
    assert (config.estimator_gain_h, config.estimator_gain_v_hz) == (0.24, 0.72)
    assert (config.calibration_duration_ms, config.drag_device_enabled, config.deploy_delay_after_apogee_ms) == (1000, True, 1000)
    FlightComputer(config)


@pytest.mark.parametrize("sensor", ["baro", "accel", "gyro", "temp"])
def test_evt_14_a_disabled_required_sensor_is_rejected(sensor: str) -> None:
    assert sensor in REQUIRED_SENSORS and len(REQUIRED_SENSORS) == 4
    fault = rejected(scenario_with(f"sensors.{sensor}.enabled", False))
    assert (fault.reason, fault.field) == (Reason.REQUIRED_SENSOR_DISABLED, f"sensors.{sensor}.enabled")
    assert sensor in fault.detail


@pytest.mark.parametrize("sensor", ["baro", "accel", "gyro", "temp"])
def test_evt_14_sensor_enabled_true_or_omitted_is_accepted_and_other_values_are_not(sensor: str) -> None:
    assert "enabled" not in REF01["sensors"][sensor]                      # REF-01 omits it: enabled
    load_avionics_config(scenario_with(f"sensors.{sensor}.enabled", True))
    for bad in ("true", 1, 0, None):
        fault = rejected(scenario_with(f"sensors.{sensor}.enabled", bad))
        assert (fault.reason, fault.field) == (Reason.INVALID_TYPE, f"sensors.{sensor}.enabled")
    fault = rejected(scenario_with(f"sensors.{sensor}", ...))
    assert (fault.reason, fault.field) == (Reason.MISSING_FIELD, f"sensors.{sensor}")


AVIONICS_FIELDS = sorted(REF01["avionics"])


@pytest.mark.parametrize("name", AVIONICS_FIELDS)
def test_evt_14_every_avionics_field_is_required(name: str) -> None:
    fault = rejected(scenario_with(f"avionics.{name}", ...))
    assert (fault.reason, fault.field) == (Reason.MISSING_FIELD, f"avionics.{name}")


@pytest.mark.parametrize(
    ("path", "reason", "field"),
    [
        ("vehicle.drag_device.enabled", Reason.MISSING_FIELD, "vehicle.drag_device.enabled"),
        ("vehicle.drag_device.deploy_delay_after_apogee_ms", Reason.MISSING_FIELD, "vehicle.drag_device.deploy_delay_after_apogee_ms"),
        ("vehicle.drag_device", Reason.MISSING_FIELD, "vehicle.drag_device"),
        ("avionics", Reason.MISSING_FIELD, "avionics"),
        ("sensors", Reason.MISSING_FIELD, "sensors"),
        ("timing", Reason.MISSING_FIELD, "timing"),
        ("timing.t_ignition_ms", Reason.MISSING_FIELD, "timing.t_ignition_ms"),
        ("sensors.baro.rate_hz", Reason.MISSING_FIELD, "sensors.baro.rate_hz"),
    ],
)
def test_evt_14_missing_sections_and_fields_are_rejected(path: str, reason: Reason, field: str) -> None:
    fault = rejected(scenario_with(path, ...))
    assert (fault.reason, fault.field) == (reason, field)


def test_evt_14_unknown_avionics_field_is_rejected() -> None:
    fault = rejected(scenario_with("avionics.kalman", True))
    assert (fault.reason, fault.field) == (Reason.UNKNOWN_FIELD, "avionics.kalman")


def test_evt_14_unknown_drag_device_field_is_rejected() -> None:
    assert set(REF01["vehicle"]["drag_device"]) == {"enabled", "cd_area_m2", "deploy_delay_after_apogee_ms"}
    fault = rejected(scenario_with("vehicle.drag_device.deploy_altitude_m", 300.0))
    assert (fault.reason, fault.field) == (Reason.UNKNOWN_FIELD, "vehicle.drag_device.deploy_altitude_m")


@pytest.mark.parametrize(
    ("path", "value", "reason"),
    [
        ("avionics.rate_hz", 0, Reason.OUT_OF_RANGE),
        ("avionics.rate_hz", -200, Reason.OUT_OF_RANGE),
        ("avionics.rate_hz", 200.0, Reason.INVALID_TYPE),
        ("avionics.rate_hz", True, Reason.INVALID_TYPE),
        ("avionics.rate_hz", 300, Reason.INCONSISTENT_RATES),          # does not divide 1000 Hz
        ("avionics.telemetry_rate_hz", 0, Reason.OUT_OF_RANGE),
        ("avionics.telemetry_rate_hz", 30, Reason.INCONSISTENT_RATES),
        ("avionics.telemetry_rate_hz", 125, Reason.INCONSISTENT_RATES),  # 8 ms is not a multiple of 5 ms
        ("avionics.calibration_duration_ms", 0, Reason.OUT_OF_RANGE),
        ("avionics.calibration_duration_ms", 1000.5, Reason.INVALID_TYPE),
        ("avionics.debounce_samples", 0, Reason.OUT_OF_RANGE),
        ("avionics.debounce_samples", 2.5, Reason.INVALID_TYPE),
        ("avionics.apogee_lockout_after_burnout_ms", -1, Reason.OUT_OF_RANGE),
        ("avionics.touchdown_window_ms", 0, Reason.OUT_OF_RANGE),
        ("avionics.touchdown_window_ms", 4, Reason.OUT_OF_RANGE),       # shorter than one 5 ms avionics period
        ("avionics.liftoff_specific_force_mps2", 0.0, Reason.OUT_OF_RANGE),
        ("avionics.liftoff_specific_force_mps2", math.nan, Reason.INVALID_TYPE),
        ("avionics.liftoff_specific_force_mps2", "15", Reason.INVALID_TYPE),
        ("avionics.touchdown_altitude_range_m", -3.0, Reason.OUT_OF_RANGE),
        ("avionics.touchdown_speed_mps", math.inf, Reason.INVALID_TYPE),
        ("avionics.estimator_gain_h", 0.0, Reason.OUT_OF_RANGE),
        ("avionics.estimator_gain_h", -0.24, Reason.OUT_OF_RANGE),
        ("avionics.estimator_gain_h", None, Reason.INVALID_TYPE),
        ("avionics.estimator_gain_v_hz", 0.0, Reason.OUT_OF_RANGE),
        ("avionics.estimator_gain_v_hz", -0.72, Reason.OUT_OF_RANGE),
        ("avionics.estimator_gain_v_hz", math.nan, Reason.INVALID_TYPE),
        ("vehicle.drag_device.enabled", "yes", Reason.INVALID_TYPE),
        ("vehicle.drag_device.deploy_delay_after_apogee_ms", -1, Reason.OUT_OF_RANGE),
        ("vehicle.drag_device.deploy_delay_after_apogee_ms", 1000.0, Reason.INVALID_TYPE),
        ("timing.t_ignition_ms", 3000.0, Reason.INVALID_TYPE),
        ("sensors.baro.rate_hz", 0, Reason.OUT_OF_RANGE),
        ("sensors.baro.rate_hz", -50, Reason.OUT_OF_RANGE),
        ("sensors.baro.rate_hz", 60, Reason.INCONSISTENT_RATES),       # does not divide the 1000 Hz physics clock
        ("sensors.baro.rate_hz", 50.0, Reason.INVALID_TYPE),
        ("sensors.baro.rate_hz", "50", Reason.INVALID_TYPE),
        ("sensors.baro.rate_hz", True, Reason.INVALID_TYPE),
        ("sensors.baro.rate_hz", None, Reason.INVALID_TYPE),
    ],
)
def test_evt_14_invalid_values_are_rejected_with_the_field_named(path: str, value: object, reason: Reason) -> None:
    fault = rejected(scenario_with(path, value))
    assert (fault.reason, fault.field) == (reason, path)


@pytest.mark.parametrize(
    ("path", "value"),
    [
        ("avionics.debounce_samples", 1),
        ("avionics.apogee_lockout_after_burnout_ms", 0),
        ("avionics.touchdown_window_ms", 5),                 # exactly one avionics period
        ("vehicle.drag_device.deploy_delay_after_apogee_ms", 0),
        ("avionics.calibration_duration_ms", 1),
        ("avionics.calibration_duration_ms", 3000),          # exactly the ignition time
        ("avionics.telemetry_rate_hz", 200),                 # equal to the avionics rate
        ("avionics.rate_hz", 1000),                          # every tick; 50 ms telemetry is a multiple
    ],
)
def test_evt_14_boundary_values_are_accepted(path: str, value: object) -> None:
    load_avionics_config(scenario_with(path, value))


def test_evt_14_calibration_must_finish_by_ignition() -> None:
    fault = rejected(scenario_with("avionics.calibration_duration_ms", 3001))
    assert (fault.reason, fault.field) == (Reason.CALIBRATION_EXCEEDS_PRE_IGNITION, "avionics.calibration_duration_ms")
    fault = rejected(scenario_with("timing.t_ignition_ms", 999))
    assert fault.reason is Reason.CALIBRATION_EXCEEDS_PRE_IGNITION


@pytest.mark.parametrize(
    ("gain_h", "gain_v_hz", "stable"),
    [
        (0.24, 0.72, True),          # REF-01
        (1.99, 0.72, True),          # 0.72 * 0.02 = 0.0144 < 4 - 3.98 = 0.02
        (1.999, 0.72, False),        # 0.0144 > 0.002
        (2.0, 0.72, False),
        (2.5, 0.72, False),
        (0.24, 175.0, True),         # 175 * 0.02 = 3.50 < 4 - 0.48 = 3.52
        (0.24, 177.0, False),        # 3.54 > 3.52
        (1.0, 99.0, True),           # 1.98 < 2
        (1.0, 101.0, False),         # 2.02 > 2
    ],
)
def test_evt_14_estimator_gains_must_be_stable(gain_h: float, gain_v_hz: float, stable: bool) -> None:
    """Oracle: the spectral radius of the error-update matrix, computed here independently."""
    period_s = 1 / REF01["sensors"]["baro"]["rate_hz"]
    a, b = 1 - gain_h, (1 - gain_h) * period_s
    c, d = -gain_v_hz, 1 - gain_v_hz * period_s
    trace, det = a + d, a * d - b * c
    disc = trace * trace - 4 * det
    radius = max(abs((trace + math.sqrt(disc)) / 2), abs((trace - math.sqrt(disc)) / 2)) if disc >= 0 else math.sqrt(det)
    assert (radius < 1) == stable                            # the table itself agrees with the matrix

    scenario = scenario_with("avionics.estimator_gain_h", gain_h)
    scenario["avionics"]["estimator_gain_v_hz"] = gain_v_hz
    if stable:
        load_avionics_config(scenario)
        check_estimator_gains(gain_h, gain_v_hz, period_s)
    else:
        assert rejected(scenario).reason is Reason.UNSTABLE_ESTIMATOR_GAINS


def test_evt_14_gain_stability_depends_on_the_barometer_rate() -> None:
    scenario = scenario_with("avionics.estimator_gain_v_hz", 300.0)   # stable at 100 Hz (3.0 < 3.52), not at 50 Hz (6.0)
    assert rejected(scenario).reason is Reason.UNSTABLE_ESTIMATOR_GAINS
    scenario["sensors"]["baro"]["rate_hz"] = 100
    load_avionics_config(scenario)


def test_evt_14_invalid_configuration_fails_before_any_tick_is_simulated(monkeypatch: pytest.MonkeyPatch) -> None:
    """Loading fails without a simulator, a sensor suite or a flight computer ever being built."""
    import aerosat.avionics.flight_computer as module
    from aerosat.sim import simulator

    def forbidden(*args, **kwargs):
        raise AssertionError("a simulation component was constructed for an invalid configuration")

    monkeypatch.setattr(simulator.Simulator, "__init__", forbidden)
    monkeypatch.setattr(simulator.Simulator, "step", forbidden)
    monkeypatch.setattr(SensorSuite, "__init__", forbidden)
    monkeypatch.setattr(module.FlightComputer, "__init__", forbidden)
    monkeypatch.setattr(module.FlightComputer, "tick", forbidden)
    for scenario in (
        scenario_with("sensors.gyro.enabled", False),
        scenario_with("avionics.estimator_gain_h", -1.0),
        scenario_with("avionics.calibration_duration_ms", 5000),
        scenario_with("avionics.touchdown_speed_mps", ...),
    ):
        rejected(scenario)
