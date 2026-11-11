"""EVT-01 to EVT-12 (TEST_PLAN section 5): flight-event detection and state estimation.

Oracles: hand-built sample sequences (EVT-01 to EVT-03), the truth events recorded by the
simulator (EVT-05, EVT-06, EVT-08, EVT-12), known injected values (EVT-09), and the atmosphere
model (EVT-11). Detection tolerances are the ones fixed in TEST_PLAN section 5.
"""
import math
import random

import pytest

from aerosat.avionics.calibration import PadCalibration
from aerosat.avionics.estimator import ComplementaryFilter, baro_altitude_agl_m
from aerosat.avionics.flight_computer import FlightComputer, avionics_config_from_scenario
from aerosat.avionics.state_machine import FlightStateMachine, StateMachineConfig
from aerosat.core.constants import G0_MPS2, P0_PA, T0_K
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
        period_ticks=5, drag_device_enabled=True, deploy_delay_ticks=1000,
    )
    values.update(overrides)
    return FlightStateMachine(StateMachineConfig(**values))


class Driver:
    """Feeds a state machine one avionics tick at a time and collects what it emits."""

    def __init__(self, fsm: FlightStateMachine) -> None:
        self.fsm, self.tick, self.events, self.commands = fsm, 0, [], []

    def feed(self, count: int, *, force: float | None = 9.8, altitude: float = 0.0, velocity: float = 0.0) -> None:
        for _ in range(count):
            events, commands = self.fsm.update(self.tick, specific_force_mps2=force, altitude_m=altitude, velocity_mps=velocity)
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
