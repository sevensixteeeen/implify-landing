"""Ties the above together; frames and commands.

ARCHITECTURE section 6. Runs at the avionics rate (200 Hz in REF-01). Consumes only sensor samples
and its own tick count (FR-AVI-01). It knows g0, the standard-atmosphere constants and its own
configuration, and nothing else about the vehicle or the truth model.
"""
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from aerosat.avionics.calibration import CalibrationResult, PadCalibration
from aerosat.avionics.estimator import ComplementaryFilter, baro_altitude_agl_m, estimated_acceleration_mps2
from aerosat.avionics.state_machine import FlightStateMachine, StateMachineConfig
from aerosat.core.clock import TICKS_PER_SECOND, ms_to_ticks, tick_to_seconds, ticks_per_sample
from aerosat.core.faults import ConfigurationFault
from aerosat.core.faults import ConfigurationReason as Reason
from aerosat.core.types import (
    AvionicsEstimate,
    FlightEvent,
    FlightState,
    SensorSample,
    TelemetryFrame,
    VehicleCommand,
)

_AVIONICS_KEYS = {
    "rate_hz", "telemetry_rate_hz", "calibration_duration_ms", "liftoff_specific_force_mps2",
    "debounce_samples", "apogee_lockout_after_burnout_ms", "touchdown_window_ms",
    "touchdown_altitude_range_m", "touchdown_speed_mps", "estimator_gain_h", "estimator_gain_v_hz",
    "backup_deployment_delay_ms", "expected_coast_duration_ms",
    "burnout_fallback_specific_force_mps2", "burnout_fallback_samples",
}


@dataclass(frozen=True)
class AvionicsConfig:
    """The avionics computer's own configuration (scenario `avionics` section, plus the drag device)."""

    rate_hz: int
    telemetry_rate_hz: int
    calibration_duration_ms: int
    liftoff_specific_force_mps2: float
    debounce_samples: int
    apogee_lockout_after_burnout_ms: int
    touchdown_window_ms: int
    touchdown_altitude_range_m: float
    touchdown_speed_mps: float
    estimator_gain_h: float
    """Complementary-filter altitude gain K_h. Dimensionless."""
    estimator_gain_v_hz: float
    """Complementary-filter velocity gain K_v, 1/s."""
    backup_deployment_delay_ms: int
    """Backup timer: time after detected BURNOUT at which apogee is inferred and the drag device deployed."""
    expected_coast_duration_ms: int
    """Expected time from burnout to apogee for this scenario. The backup timer must be longer."""
    burnout_fallback_specific_force_mps2: float
    """Fallback BURNOUT criterion: specific force below this value, m/s^2."""
    burnout_fallback_samples: int
    """Fallback BURNOUT criterion: consecutive accelerometer samples required."""
    drag_device_enabled: bool
    deploy_delay_after_apogee_ms: int

    def __post_init__(self) -> None:
        """Validate every value on its own. Raises `ConfigurationFault` naming the field."""
        for name in ("rate_hz", "telemetry_rate_hz", "calibration_duration_ms", "debounce_samples",
                     "backup_deployment_delay_ms", "expected_coast_duration_ms", "burnout_fallback_samples"):
            _require_int(self, name, minimum=1)
        for name in ("apogee_lockout_after_burnout_ms", "deploy_delay_after_apogee_ms"):
            _require_int(self, name, minimum=0)
        _require_int(self, "touchdown_window_ms", minimum=1)
        for name in ("liftoff_specific_force_mps2", "touchdown_altitude_range_m", "touchdown_speed_mps",
                     "estimator_gain_h", "estimator_gain_v_hz", "burnout_fallback_specific_force_mps2"):
            _require_positive_number(self, name)
        if not isinstance(self.drag_device_enabled, bool):
            raise ConfigurationFault(Reason.INVALID_TYPE, "vehicle.drag_device.enabled", "must be true or false")

        for name in ("rate_hz", "telemetry_rate_hz"):
            if TICKS_PER_SECOND % getattr(self, name) != 0:
                raise ConfigurationFault(
                    Reason.INCONSISTENT_RATES, _field(name),
                    f"{getattr(self, name)} Hz does not divide the physics rate of {TICKS_PER_SECOND} Hz exactly",
                )
        period_ms = TICKS_PER_SECOND // self.rate_hz
        if (TICKS_PER_SECOND // self.telemetry_rate_hz) % period_ms != 0:
            raise ConfigurationFault(
                Reason.INCONSISTENT_RATES, "avionics.telemetry_rate_hz",
                "the telemetry period must be a whole multiple of the avionics period",
            )
        if self.touchdown_window_ms < period_ms:
            raise ConfigurationFault(
                Reason.OUT_OF_RANGE, "avionics.touchdown_window_ms", "must span at least one avionics period"
            )
        if self.burnout_fallback_specific_force_mps2 >= self.liftoff_specific_force_mps2:
            raise ConfigurationFault(
                Reason.OUT_OF_RANGE, "avionics.burnout_fallback_specific_force_mps2",
                f"must be below the liftoff threshold of {self.liftoff_specific_force_mps2} m/s^2, "
                f"got {self.burnout_fallback_specific_force_mps2}",
            )
        if self.burnout_fallback_samples < self.debounce_samples:
            raise ConfigurationFault(
                Reason.OUT_OF_RANGE, "avionics.burnout_fallback_samples",
                f"must be at least debounce_samples ({self.debounce_samples}), got {self.burnout_fallback_samples}: "
                "the primary criterion must be able to fire first",
            )
        if self.backup_deployment_delay_ms <= self.expected_coast_duration_ms:
            raise ConfigurationFault(
                Reason.OUT_OF_RANGE, "avionics.backup_deployment_delay_ms",
                f"must exceed the expected coast duration of {self.expected_coast_duration_ms} ms, "
                f"got {self.backup_deployment_delay_ms} ms: the backup timer would fire during ascent",
            )
        if self.estimator_gain_h >= 2.0:
            raise ConfigurationFault(
                Reason.UNSTABLE_ESTIMATOR_GAINS, "avionics.estimator_gain_h",
                f"K_h = {self.estimator_gain_h} is not below 2: the altitude correction would diverge",
            )


_DRAG_DEVICE_FIELDS = {"drag_device_enabled", "deploy_delay_after_apogee_ms"}
_DRAG_DEVICE_SCENARIO_KEYS = {"enabled", "cd_area_m2", "deploy_delay_after_apogee_ms"}
"""Fields of `vehicle.drag_device` (PROJECT_STRUCTURE section 5). `cd_area_m2` belongs to the simulator."""
REQUIRED_SENSORS = ("baro", "accel", "gyro", "temp")
"""Sensors the current avionics cannot work without: calibration, estimation and every telemetry frame use them."""


def _field(name: str) -> str:
    if name == "drag_device_enabled":
        return "vehicle.drag_device.enabled"
    return f"vehicle.drag_device.{name}" if name in _DRAG_DEVICE_FIELDS else f"avionics.{name}"


def _require_int(config: "AvionicsConfig", name: str, *, minimum: int) -> None:
    value = getattr(config, name)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigurationFault(Reason.INVALID_TYPE, _field(name), f"must be an integer, got {value!r}")
    if value < minimum:
        raise ConfigurationFault(Reason.OUT_OF_RANGE, _field(name), f"must be at least {minimum}, got {value}")


def _require_positive_number(config: "AvionicsConfig", name: str) -> None:
    value = getattr(config, name)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ConfigurationFault(Reason.INVALID_TYPE, _field(name), f"must be a finite number, got {value!r}")
    if value <= 0:
        raise ConfigurationFault(Reason.OUT_OF_RANGE, _field(name), f"must be greater than zero, got {value}")


def avionics_config_from_scenario(avionics: Mapping[str, Any], drag_device: Mapping[str, Any]) -> AvionicsConfig:
    """Read the `avionics` and `vehicle.drag_device` sections of a scenario.

    Every field is required: nothing is defaulted. Raises `ConfigurationFault` naming the field.
    """
    for name in sorted(_AVIONICS_KEYS - set(avionics)):
        raise ConfigurationFault(Reason.MISSING_FIELD, f"avionics.{name}", "required field is missing")
    for name in sorted(set(avionics) - _AVIONICS_KEYS):
        raise ConfigurationFault(Reason.UNKNOWN_FIELD, f"avionics.{name}", "unknown field")
    for name in ("enabled", "deploy_delay_after_apogee_ms"):
        if name not in drag_device:
            raise ConfigurationFault(Reason.MISSING_FIELD, f"vehicle.drag_device.{name}", "required field is missing")
    for name in sorted(set(drag_device) - _DRAG_DEVICE_SCENARIO_KEYS):
        raise ConfigurationFault(Reason.UNKNOWN_FIELD, f"vehicle.drag_device.{name}", "unknown field")
    return AvionicsConfig(
        **{name: avionics[name] for name in _AVIONICS_KEYS},
        drag_device_enabled=drag_device["enabled"],
        deploy_delay_after_apogee_ms=drag_device["deploy_delay_after_apogee_ms"],
    )


def _section(scenario: Mapping[str, Any], path: str) -> Mapping[str, Any]:
    node: Any = scenario
    for depth, key in enumerate(path.split("."), start=1):
        if not isinstance(node, Mapping) or key not in node:
            raise ConfigurationFault(
                Reason.MISSING_FIELD, ".".join(path.split(".")[:depth]), "required section is missing"
            )
        node = node[key]
    if not isinstance(node, Mapping):
        raise ConfigurationFault(Reason.INVALID_TYPE, path, "must be an object")
    return node


def load_avionics_config(scenario: Mapping[str, Any]) -> AvionicsConfig:
    """Read and fully validate the avionics configuration of a scenario, before any simulation tick.

    On top of the per-value checks it verifies, against the rest of the scenario:

    - all four required sensors are present and enabled;
    - pad calibration finishes no later than ignition;
    - the estimator gains are stable for the barometer's sample period.

    Raises `ConfigurationFault` naming the offending field.
    """
    config = avionics_config_from_scenario(_section(scenario, "avionics"), _section(scenario, "vehicle.drag_device"))

    for name in REQUIRED_SENSORS:
        section = _section(scenario, f"sensors.{name}")
        enabled = section.get("enabled", True)  # optional in the scenario format; true when omitted
        if not isinstance(enabled, bool):
            raise ConfigurationFault(
                Reason.INVALID_TYPE, f"sensors.{name}.enabled", f"must be true or false, got {enabled!r}"
            )
        if not enabled:
            raise ConfigurationFault(
                Reason.REQUIRED_SENSOR_DISABLED, f"sensors.{name}.enabled",
                f"the {name} sensor is required by the avionics and cannot be disabled",
            )

    timing = _section(scenario, "timing")
    if "t_ignition_ms" not in timing:
        raise ConfigurationFault(Reason.MISSING_FIELD, "timing.t_ignition_ms", "required field is missing")
    t_ignition_ms = timing["t_ignition_ms"]
    if isinstance(t_ignition_ms, bool) or not isinstance(t_ignition_ms, int):
        raise ConfigurationFault(Reason.INVALID_TYPE, "timing.t_ignition_ms", f"must be an integer, got {t_ignition_ms!r}")
    if config.calibration_duration_ms > t_ignition_ms:
        raise ConfigurationFault(
            Reason.CALIBRATION_EXCEEDS_PRE_IGNITION, "avionics.calibration_duration_ms",
            f"calibration lasts {config.calibration_duration_ms} ms but ignition is at {t_ignition_ms} ms",
        )

    baro = scenario["sensors"]["baro"]
    if "rate_hz" not in baro:
        raise ConfigurationFault(Reason.MISSING_FIELD, "sensors.baro.rate_hz", "required field is missing")
    baro_rate_hz = baro["rate_hz"]
    if isinstance(baro_rate_hz, bool) or not isinstance(baro_rate_hz, int):
        raise ConfigurationFault(
            Reason.INVALID_TYPE, "sensors.baro.rate_hz", f"must be an integer, got {baro_rate_hz!r}"
        )
    if baro_rate_hz < 1:
        raise ConfigurationFault(
            Reason.OUT_OF_RANGE, "sensors.baro.rate_hz", f"must be at least 1, got {baro_rate_hz}"
        )
    if TICKS_PER_SECOND % baro_rate_hz != 0:
        raise ConfigurationFault(
            Reason.INCONSISTENT_RATES, "sensors.baro.rate_hz",
            f"{baro_rate_hz} Hz does not divide the physics rate of {TICKS_PER_SECOND} Hz exactly",
        )
    check_estimator_gains(config.estimator_gain_h, config.estimator_gain_v_hz, 1.0 / baro_rate_hz)
    return config


def check_estimator_gains(gain_h: float, gain_v_hz: float, correction_period_s: float) -> None:
    """Reject gains for which the complementary filter's error does not decay.

    Between barometer corrections (period T_b) the error obeys
        e_h' = (1 - K_h)(e_h + T_b*e_v),   e_v' = e_v - K_v(e_h + T_b*e_v),
    whose characteristic polynomial has trace 2 - K_h - K_v*T_b and determinant 1 - K_h. Both roots
    lie inside the unit circle exactly when K_h > 0, K_v > 0, K_h < 2 and K_v*T_b < 4 - 2*K_h.
    """
    if not (gain_h > 0 and gain_v_hz > 0 and gain_h < 2.0 and gain_v_hz * correction_period_s < 4.0 - 2.0 * gain_h):
        raise ConfigurationFault(
            Reason.UNSTABLE_ESTIMATOR_GAINS, "avionics.estimator_gain_v_hz",
            f"K_h = {gain_h}, K_v = {gain_v_hz} 1/s are unstable for a correction period of "
            f"{correction_period_s} s (need K_h < 2 and K_v*T_b < 4 - 2*K_h)",
        )


@dataclass(frozen=True, slots=True)
class AvionicsOutput:
    """Everything the avionics produces on one tick."""

    estimate: AvionicsEstimate
    flight_state: FlightState
    events: tuple[FlightEvent, ...]
    commands: tuple[VehicleCommand, ...]
    frame: TelemetryFrame | None
    """A telemetry frame every 50 ms of sim time, otherwise None."""


class FlightComputer:
    """Calibration, state estimation, flight state machine, telemetry frames and commands."""

    def __init__(self, config: AvionicsConfig) -> None:
        self._period_ticks = ticks_per_sample(config.rate_hz)
        self._telemetry_ticks = ticks_per_sample(config.telemetry_rate_hz)
        self._calibration_ticks = ms_to_ticks(config.calibration_duration_ms)
        self._calibration = PadCalibration()
        self.calibration: CalibrationResult | None = None
        self._filter = ComplementaryFilter(
            config.estimator_gain_h, config.estimator_gain_v_hz, tick_to_seconds(self._period_ticks)
        )
        self._fsm = FlightStateMachine(StateMachineConfig(
            liftoff_specific_force_mps2=config.liftoff_specific_force_mps2,
            debounce_samples=config.debounce_samples,
            apogee_lockout_ticks=ms_to_ticks(config.apogee_lockout_after_burnout_ms),
            touchdown_window_ticks=ms_to_ticks(config.touchdown_window_ms),
            touchdown_altitude_range_m=config.touchdown_altitude_range_m,
            touchdown_speed_mps=config.touchdown_speed_mps,
            period_ticks=self._period_ticks,
            drag_device_enabled=config.drag_device_enabled,
            deploy_delay_ticks=ms_to_ticks(config.deploy_delay_after_apogee_ms),
            backup_deployment_ticks=ms_to_ticks(config.backup_deployment_delay_ms),
            burnout_fallback_specific_force_mps2=config.burnout_fallback_specific_force_mps2,
            burnout_fallback_samples=config.burnout_fallback_samples,
        ))
        self._latest: dict[tuple[str, str], float] = {}
        self._acceleration_mps2 = 0.0
        self._baro_altitude_m = 0.0

    @property
    def flight_state(self) -> FlightState:
        return self._fsm.state

    def tick(self, tick: int, samples: Sequence[SensorSample]) -> AvionicsOutput:
        """Process one avionics tick with the sensor samples that arrived since the previous one."""
        if tick % self._period_ticks != 0:
            raise ValueError(f"tick {tick} is not an avionics tick (period {self._period_ticks} ticks)")

        new_accel = new_baro = False
        for sample in samples:
            self._latest[(sample.sensor_id, sample.channel)] = sample.value
            new_accel = new_accel or (sample.sensor_id == "accel" and sample.channel == "z")
            new_baro = new_baro or sample.sensor_id == "baro"

        events: list[FlightEvent] = []
        commands: list[VehicleCommand] = []
        just_calibrated = False
        if self.calibration is None:
            if tick < self._calibration_ticks:
                self._calibration.add(samples)
            else:
                self.calibration = self._calibration.result()
                just_calibrated = True

        if self.calibration is not None:
            calibration = self.calibration
            specific_force = self._latest[("accel", "z")] - calibration.accel_bias_mps2
            self._acceleration_mps2 = estimated_acceleration_mps2(self._latest[("accel", "z")], calibration.accel_bias_mps2)
            if not just_calibrated:
                self._filter.predict(self._acceleration_mps2)
            if new_baro:
                self._baro_altitude_m = baro_altitude_agl_m(
                    self._latest[("baro", "pressure")], calibration.reference_pressure_pa,
                    calibration.reference_temperature_k,
                )
                self._filter.correct(self._baro_altitude_m)
            events, commands = self._fsm.update(
                tick,
                specific_force_mps2=specific_force if new_accel else None,
                altitude_m=self._filter.altitude_agl_m,
                velocity_mps=self._filter.velocity_mps,
            )

        t_sim_s = tick_to_seconds(tick)
        state = self._fsm.state
        estimate = AvionicsEstimate(
            tick, t_sim_s, self._filter.altitude_agl_m, self._filter.velocity_mps, self._acceleration_mps2,
            self._baro_altitude_m, state,
        )
        frame = None
        if tick % self._telemetry_ticks == 0:
            frame = TelemetryFrame(
                tick, t_sim_s, estimate.altitude_agl_m, estimate.velocity_mps, estimate.acceleration_mps2,
                self._reading("baro", "pressure"), self._reading("temp", "temperature"), state,
                self._reading("gyro", "z"),
            )
        return AvionicsOutput(estimate, state, tuple(events), tuple(commands), frame)

    def _reading(self, sensor_id: str, channel: str) -> float:
        try:
            return self._latest[(sensor_id, channel)]
        except KeyError:
            raise ValueError(
                f"telemetry needs a reading from {sensor_id}.{channel}, but none has arrived; "
                "the sensor is disabled or has not sampled yet"
            ) from None
