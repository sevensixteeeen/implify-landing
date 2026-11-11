"""SEN-01 to SEN-11 and SEN-13 (TEST_PLAN section 6). SEN-12 is in tests/unit/core/test_core_rng.py.

Expected values come from hand-built truth states, the measurement model written out in
ARCHITECTURE section 5.1, or statistical limits fixed in TEST_PLAN. Seeds are fixed.
"""
import copy
import json
import math
import statistics
from dataclasses import replace
from pathlib import Path

import pytest

from aerosat.core.types import SensorSample
from aerosat.sensors.accelerometer import Accelerometer
from aerosat.sensors.barometer import Barometer
from aerosat.sensors.gyroscope import Gyroscope
from aerosat.sensors.pipeline import Channel, ErrorModel, SensorConfig
from aerosat.sensors.suite import SensorSuite, SensorSuiteConfig, suite_config_from_scenario
from aerosat.sensors.thermometer import Thermometer
from aerosat.sim.atmosphere import VacuumAtmosphere
from aerosat.sim.gravity import inverse_square_gravity_mps2
from aerosat.sim.simulator import Simulator, SimulatorConfig
from aerosat.sim.vehicle import RollProfile, ThrustCurve, Vehicle

from .conftest import truth

REF01 = json.loads((Path(__file__).resolve().parents[4] / "scenarios" / "ref-01.json").read_text(encoding="utf-8"))


def ref01_simulator(**overrides) -> Simulator:
    vehicle = REF01["vehicle"]
    config = SimulatorConfig(
        vehicle=Vehicle(vehicle["dry_mass_kg"], vehicle["propellant_mass_kg"],
                        vehicle["drag_coefficient"] * vehicle["reference_area_m2"],
                        ThrustCurve([(p["t_ms"], p["thrust_n"]) for p in vehicle["thrust_curve"]])),
        t_ignition_ms=REF01["timing"]["t_ignition_ms"],
        roll_profile=RollProfile([(p["t_ms"], p["roll_rate_radps"]) for p in vehicle["roll_profile"]]),
    )
    return Simulator(replace(config, **overrides))


def by_channel(samples: list[SensorSample]) -> dict[tuple[str, str], list[SensorSample]]:
    grouped: dict[tuple[str, str], list[SensorSample]] = {}
    for sample in samples:
        grouped.setdefault((sample.sensor_id, sample.channel), []).append(sample)
    return grouped


def channel(error: ErrorModel, *, seed: int = 42, prefix: str = "test", period_s: float = 0.1) -> Channel:
    return Channel("test", "value", "unit", error, master_seed=seed, stream_prefix=prefix, sample_period_s=period_s)


# ---------------------------------------------------------------- SEN-01, SEN-02

def test_sen_01_ideal_sensors_equal_the_truth_mapping_exactly(default_config: SensorSuiteConfig) -> None:
    """Ideal mode over the first 8 s of REF-01 (pad, boost, coast, with roll)."""
    suite = SensorSuite(replace(default_config, ideal=True), master_seed=42)
    sim = ref01_simulator()
    counts = {"baro": 0, "accel": 0, "gyro": 0, "temp": 0}
    state = sim.state
    for _ in range(8_001):
        expected = {
            ("baro", "pressure"): state.pressure_pa,
            ("accel", "x"): 0.0, ("accel", "y"): 0.0, ("accel", "z"): state.specific_force_mps2,
            ("gyro", "x"): 0.0, ("gyro", "y"): 0.0, ("gyro", "z"): state.roll_rate_radps,
            ("temp", "temperature"): state.temperature_k,
        }
        for sample in suite.sample(state):
            assert sample.value == expected[(sample.sensor_id, sample.channel)]
            assert sample.saturated is False
            assert sample.tick == state.tick and sample.t_sim_s == state.t_sim_s
            counts[sample.sensor_id] += 1
        state = sim.step()
    assert all(count > 0 for count in counts.values())
    assert state.roll_rate_radps > 0.0 and state.altitude_agl_m > 100.0  # the run covered real flight


def test_sen_02_accelerometer_reads_specific_force_not_acceleration() -> None:
    accel = Accelerometer(SensorConfig(rate_hz=200), master_seed=42, ideal=True)

    def z(state) -> float:
        return accel.sample(state)[2].value

    # On the pad (500 m above sea level): +g(h_pad), although the kinematic acceleration is 0.
    pad = ref01_simulator(pad_altitude_msl_m=500.0)
    for _ in range(1_000):
        state = pad.step()
    assert state.acceleration_mps2 == 0.0
    assert z(state) == pytest.approx(inverse_square_gravity_mps2(500.0), abs=1e-12)

    # Powered flight: (F_T - F_D) / m.
    powered = ref01_simulator()
    for _ in range(4_500):
        state = powered.step()
    assert state.thrust_n == 1500.0 and state.drag_n > 0.0
    assert z(state) == pytest.approx((state.thrust_n - state.drag_n) / state.mass_kg, abs=1e-12)

    # Drag-free coast in a vacuum: 0, although the vehicle is accelerating downward at g.
    coast = Simulator(SimulatorConfig(vehicle=Vehicle(15.0, 0.0, 0.005), atmosphere=VacuumAtmosphere(),
                                      initial_velocity_mps=100.0))
    for _ in range(2_000):
        state = coast.step()
    assert state.acceleration_mps2 < -9.7
    assert z(state) == pytest.approx(0.0, abs=1e-12)


# ---------------------------------------------------------------- SEN-03, SEN-04

N = 20_000


@pytest.mark.parametrize(
    ("sensor_class", "name", "true_value"),
    [(Barometer, "baro", 90_000.0), (Accelerometer, "accel", 9.80665), (Gyroscope, "gyro", 0.5),
     (Thermometer, "temp", 280.0)],
)
def test_sen_03_noise_statistics(sensor_class, name: str, true_value: float, default_config: SensorSuiteConfig) -> None:
    """Constant truth, quantization off, N = 20 000 samples per channel, seed 42."""
    config: SensorConfig = getattr(default_config, name)
    error = replace(config.error, resolution=0.0)
    sensor = sensor_class(replace(config, error=error), master_seed=42)
    state = truth(pressure_pa=true_value, specific_force_mps2=true_value, roll_rate_radps=true_value,
                  temperature_k=true_value)
    series = [[] for _ in sensor.channels]
    for _ in range(N):
        for values, sample in zip(series, sensor.sample(state)):
            assert not sample.saturated
            values.append(sample.value)

    sigma = error.noise_sigma
    for ch, (_, read), values in zip(sensor.channels, sensor.truth_sources, series):
        errors = [value - read(state) for value in values]
        mean = statistics.fmean(errors)
        assert abs(mean - ch.bias) <= 4 * sigma / math.sqrt(N)
        assert abs(statistics.pstdev(errors) - sigma) <= 0.03 * sigma
        centred = [e - mean for e in errors]
        lag1 = sum(a * b for a, b in zip(centred, centred[1:])) / sum(c * c for c in centred)
        assert abs(lag1) < 4 / math.sqrt(N)


def test_sen_04_bias_is_constant_within_a_run() -> None:
    ch = channel(ErrorModel(bias_sigma=30.0))
    outputs = {ch.measure(tick, 90_000.0).value for tick in range(0, 2_000, 20)}
    assert outputs == {90_000.0 + ch.bias}
    assert ch.bias != 0.0


def test_sen_04_bias_spread_over_seeds_0_to_999() -> None:
    biases = [channel(ErrorModel(bias_sigma=30.0), seed=seed, prefix="baro").bias for seed in range(1_000)]
    assert abs(statistics.pstdev(biases) - 30.0) <= 0.10 * 30.0
    assert len(set(biases)) == 1_000


# ---------------------------------------------------------------- SEN-05, SEN-06, SEN-13

def test_sen_05_outputs_are_multiples_of_the_resolution(default_config: SensorSuiteConfig) -> None:
    suite = SensorSuite(default_config, master_seed=42)
    resolution = {"baro": 1.0, "accel": 0.005, "gyro": 0.001, "temp": 0.01}  # ARCHITECTURE 5.2
    checked = 0
    for tick in range(0, 3_000):
        state = truth(tick, pressure_pa=95_432.1 - tick * 0.37, specific_force_mps2=31.4159 + tick * 1e-3,
                      roll_rate_radps=0.2718, temperature_k=281.2345)
        for sample in suite.sample(state):
            steps = sample.value / resolution[sample.sensor_id]
            assert abs(steps - round(steps)) < 1e-6
            checked += 1
    assert checked > 3_000


@pytest.mark.parametrize(
    ("field", "sensor", "channel_name", "beyond", "limit"),
    [
        ("pressure_pa", "baro", "pressure", 150_000.0, 110_000.0),
        ("pressure_pa", "baro", "pressure", 200.0, 1_000.0),
        ("specific_force_mps2", "accel", "z", 400.0, 160.0),
        ("specific_force_mps2", "accel", "z", -400.0, -160.0),
        ("roll_rate_radps", "gyro", "z", 80.0, 35.0),
        ("roll_rate_radps", "gyro", "z", -80.0, -35.0),
        ("temperature_k", "temp", "temperature", 400.0, 330.0),
        ("temperature_k", "temp", "temperature", 100.0, 200.0),
    ],
)
def test_sen_06_saturation_gives_the_range_limit_and_sets_the_flag(
    field: str, sensor: str, channel_name: str, beyond: float, limit: float, default_config: SensorSuiteConfig
) -> None:
    suite = SensorSuite(default_config, master_seed=42)
    samples = by_channel(suite.sample(truth(0, **{field: beyond})))[(sensor, channel_name)]
    assert samples[0].value == pytest.approx(limit, abs=1e-9)
    assert samples[0].saturated is True
    inside = by_channel(SensorSuite(default_config, master_seed=42).sample(truth(0)))[(sensor, channel_name)]
    assert inside[0].saturated is False


def test_sen_13_error_stages_are_applied_in_the_specified_order() -> None:
    """bias -> noise -> saturate -> quantize, on a case where the order changes the answer.

    Range 0 to 10.3, resolution 1.0, true value 9.0, and a bias large enough to exceed the range.
    """
    seed = next(s for s in range(100) if channel(ErrorModel(bias_sigma=5.0), seed=s).bias > 3.0)
    quiet = channel(ErrorModel(bias_sigma=5.0, resolution=1.0, range_min=0.0, range_max=10.3), seed=seed)
    bias = quiet.bias
    sample = quiet.measure(0, 9.0)
    assert 9.0 < 10.3 < 9.0 + bias              # in range before the bias, beyond it after
    assert sample.saturated is True             # so the bias was applied before saturation
    assert sample.value == 10.0                 # Q(sat(x + b)) = Q(10.3); the wrong order gives sat(Q(.)) = 10.3

    # Noise is applied before saturation: far beyond the range, every output is the same value.
    noisy = channel(ErrorModel(bias_sigma=5.0, noise_sigma=0.5, resolution=1.0, range_min=0.0, range_max=10.3), seed=seed)
    assert {noisy.measure(tick, 30.0).value for tick in range(200)} == {10.0}

    # Quantization is last: an in-range value is rounded after bias and noise are added.
    fine = channel(ErrorModel(bias_sigma=5.0, resolution=0.25, range_min=-100.0, range_max=100.0), seed=seed)
    assert fine.measure(0, 9.0).value == round((9.0 + fine.bias) / 0.25) * 0.25


# ---------------------------------------------------------------- SEN-07, SEN-08

def test_sen_07_sample_timing(default_config: SensorSuiteConfig) -> None:
    """10 s of ticks: count = floor(duration * rate) + 1; every sample tick is a multiple of the period."""
    suite = SensorSuite(default_config, master_seed=42)
    samples = [sample for tick in range(0, 10_001) for sample in suite.sample(truth(tick))]
    grouped = by_channel(samples)
    rate_hz = {"baro": 50, "accel": 200, "gyro": 200, "temp": 10}
    period = {"baro": 20, "accel": 5, "gyro": 5, "temp": 100}  # ARCHITECTURE 5.2, ticks per sample
    assert len(grouped) == 8
    for (sensor, _), channel_samples in grouped.items():
        assert len(channel_samples) == math.floor(10 * rate_hz[sensor]) + 1
        assert all(sample.tick % period[sensor] == 0 for sample in channel_samples)
        assert [s.tick for s in channel_samples] == sorted({s.tick for s in channel_samples})
        assert all(s.t_sim_s == s.tick / 1000 for s in channel_samples)


def test_sen_08_temperature_lag_reaches_63_percent_at_tau() -> None:
    """Step from 280 K to 290 K, noise off, tau = 2.0 s, 10 Hz."""
    thermometer = Thermometer(SensorConfig(rate_hz=10, error=ErrorModel(lag_tau_s=2.0)), master_seed=42)
    assert thermometer.sample(truth(0, temperature_k=280.0))[0].value == 280.0  # starts settled
    target = 1 - math.exp(-1)                                                   # 63.2 %
    fractions = []
    for k in range(1, 61):
        value = thermometer.sample(truth(100 * k, temperature_k=290.0))[0].value
        fractions.append((value - 280.0) / 10.0)
    assert fractions == sorted(fractions) and fractions[-1] < 1.0               # monotonic, no overshoot
    crossing_s = next(k for k, fraction in enumerate(fractions, start=1) if fraction >= target - 1e-12) * 0.1
    assert abs(crossing_s - 2.0) <= 0.1                                         # within one sample period
    assert fractions[19] == pytest.approx(target, abs=1e-12)                    # 20 samples = 2.0 s


# ---------------------------------------------------------------- SEN-09, SEN-10, SEN-11

def accel_sequence(config: SensorSuiteConfig, seed: int = 42) -> list[float]:
    suite = SensorSuite(config, master_seed=seed)
    return [s.value for tick in range(0, 2_000) for s in suite.sample(truth(tick)) if s.sensor_id == "accel"]


def test_sen_09_streams_are_independent(default_config: SensorSuiteConfig) -> None:
    baseline = accel_sequence(default_config)
    louder_baro = replace(default_config, baro=replace(
        default_config.baro, error=replace(default_config.baro.error, noise_sigma=50.0)))
    assert accel_sequence(louder_baro) == baseline
    assert accel_sequence(replace(default_config, gyro=replace(default_config.gyro, enabled=False))) == baseline
    assert len(baseline) == 3 * 400 and len(set(baseline)) > 100


def test_sen_08_lag_starts_at_the_first_true_value_with_no_startup_transient() -> None:
    for first_k in (216.65, 255.0, 288.15):
        thermometer = Thermometer(SensorConfig(rate_hz=10, error=ErrorModel(lag_tau_s=2.0)), master_seed=42)
        assert thermometer.sample(truth(0, temperature_k=first_k))[0].value == first_k   # first sample: exact
        for k in range(1, 50):                                                          # constant truth: no drift
            assert thermometer.sample(truth(100 * k, temperature_k=first_k))[0].value == first_k


# ---------------------------------------------------------------- SEN-14: enabled flag

def test_sen_14_sensors_are_enabled_by_default(ref01_sensors: dict, default_config: SensorSuiteConfig) -> None:
    assert all("enabled" not in ref01_sensors[name] for name in ("baro", "accel", "gyro", "temp"))
    assert all(getattr(default_config, name).enabled is True for name in ("baro", "accel", "gyro", "temp"))
    suite = SensorSuite(default_config, master_seed=42)
    assert suite.sensor_status() == {"baro": True, "accel": True, "gyro": True, "temp": True}
    assert {s.sensor_id for s in suite.sample(truth(0))} == {"baro", "accel", "gyro", "temp"}


@pytest.mark.parametrize("disabled", ["baro", "accel", "gyro", "temp"])
def test_sen_14_a_disabled_sensor_generates_no_samples(disabled: str, ref01_sensors: dict) -> None:
    section = copy.deepcopy(ref01_sensors)
    section[disabled]["enabled"] = False
    section["accel" if disabled != "accel" else "baro"]["enabled"] = True   # an explicit true is accepted
    suite = SensorSuite(suite_config_from_scenario(section), master_seed=42)
    seen = {s.sensor_id for tick in range(0, 1_000) for s in suite.sample(truth(tick))}
    assert seen == {"baro", "accel", "gyro", "temp"} - {disabled}
    assert suite.sensor_status() == {name: name != disabled for name in ("baro", "accel", "gyro", "temp")}


def test_sen_14_all_sensors_disabled_gives_no_samples(ref01_sensors: dict) -> None:
    section = copy.deepcopy(ref01_sensors)
    for name in ("baro", "accel", "gyro", "temp"):
        section[name]["enabled"] = False
    suite = SensorSuite(suite_config_from_scenario(section), master_seed=42)
    assert [s for tick in range(0, 500) for s in suite.sample(truth(tick))] == []
    assert suite.sensor_status() == {"baro": False, "accel": False, "gyro": False, "temp": False}


@pytest.mark.parametrize("bad", ["true", "false", 1, 0, None, 1.0, [], {}])
def test_sen_14_invalid_enabled_values_are_rejected(bad: object, ref01_sensors: dict) -> None:
    section = copy.deepcopy(ref01_sensors)
    section["gyro"]["enabled"] = bad
    with pytest.raises(ValueError, match=r"sensors\.gyro\.enabled"):
        suite_config_from_scenario(section)
    with pytest.raises(ValueError):
        SensorConfig(rate_hz=200, enabled=bad)  # type: ignore[arg-type]


def test_sen_14_a_missing_sensor_section_is_rejected(ref01_sensors: dict) -> None:
    section = copy.deepcopy(ref01_sensors)
    del section["temp"]
    with pytest.raises(ValueError, match=r"sensors\.temp"):
        suite_config_from_scenario(section)


def test_sen_10_same_seed_reproduces_and_different_seed_differs(default_config: SensorSuiteConfig) -> None:
    def run(seed: int) -> list[SensorSample]:
        suite = SensorSuite(default_config, master_seed=seed)
        return [sample for tick in range(0, 1_000) for sample in suite.sample(truth(tick))]

    assert run(42) == run(42)
    assert run(42) != run(43)


def test_sen_11_every_sample_carries_valid_metadata(default_config: SensorSuiteConfig) -> None:
    allowed = {  # ARCHITECTURE 5.2: sensor ID -> (unit, channels)
        "baro": ("Pa", {"pressure"}),
        "accel": ("m/s^2", {"x", "y", "z"}),
        "gyro": ("rad/s", {"x", "y", "z"}),
        "temp": ("K", {"temperature"}),
    }
    suite = SensorSuite(default_config, master_seed=42)
    seen = set()
    for tick in range(0, 500):
        for sample in suite.sample(truth(tick)):
            unit, channels = allowed[sample.sensor_id]
            assert sample.unit == unit and sample.channel in channels
            assert isinstance(sample.saturated, bool) and math.isfinite(sample.value)
            assert sample.tick == tick and sample.t_sim_s == tick / 1000
            seen.add((sample.sensor_id, sample.channel))
    assert seen == {(sensor, ch) for sensor, (_, channels) in allowed.items() for ch in channels}


# ---------------------------------------------------------------- configuration

def test_ref01_sensor_section_is_read_as_architecture_5_2(default_config: SensorSuiteConfig) -> None:
    assert default_config.ideal is False
    assert default_config.baro == SensorConfig(50, ErrorModel(3.0, 30.0, 1.0, 1_000.0, 110_000.0))
    assert default_config.accel == SensorConfig(200, ErrorModel(0.05, 0.10, 0.005, -160.0, 160.0))
    assert default_config.gyro == SensorConfig(200, ErrorModel(0.002, 0.005, 0.001, -35.0, 35.0))
    assert default_config.temp == SensorConfig(10, ErrorModel(0.05, 0.5, 0.01, 200.0, 330.0, lag_tau_s=2.0))


@pytest.mark.parametrize(
    ("mutate", "named"),
    [
        (lambda s: s["baro"].pop("noise_sigma_pa"), "sensors.baro"),
        (lambda s: s["accel"].update(noise_sigma_mps2=-1.0), "sensors.accel"),
        (lambda s: s["gyro"].update(range_radps=[35.0, -35.0]), "sensors.gyro"),
        (lambda s: s["temp"].update(lag_tau_s=0.0), "sensors.temp"),
        (lambda s: s["baro"].update(noise_sigma_k=1.0), "sensors.baro"),
        (lambda s: s.update(magnetometer={}), "sensors"),
        (lambda s: s.update(ideal="no"), "sensors.ideal"),
    ],
)
def test_invalid_sensor_configuration_is_rejected_with_the_field_named(mutate, named: str, ref01_sensors: dict) -> None:
    section = copy.deepcopy(ref01_sensors)
    mutate(section)
    with pytest.raises(ValueError, match=named.replace(".", r"\.")):
        suite_config_from_scenario(section)


def test_rate_must_divide_the_physics_rate(default_config: SensorSuiteConfig) -> None:
    with pytest.raises(ValueError):
        SensorSuite(replace(default_config, baro=replace(default_config.baro, rate_hz=30)), master_seed=42)
