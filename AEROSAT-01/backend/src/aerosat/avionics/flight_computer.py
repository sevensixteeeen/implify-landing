"""Ties the above together; frames and commands.

ARCHITECTURE section 6. Runs at the avionics rate (200 Hz in REF-01). Consumes only sensor samples
and its own tick count (FR-AVI-01). It knows g0, the standard-atmosphere constants and its own
configuration, and nothing else about the vehicle or the truth model.
"""
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from typing import Any

from aerosat.avionics.calibration import CalibrationResult, PadCalibration
from aerosat.avionics.estimator import (
    ComplementaryFilter,
    DeadReckoning,
    DisagreementFlag,
    DisagreementMonitor,
    baro_altitude_agl_m,
    estimated_acceleration_mps2,
)
from aerosat.avionics.state_machine import FlightStateMachine, StateMachineConfig
from aerosat.core.clock import TICKS_PER_SECOND, ms_to_ticks, tick_to_seconds, ticks_per_sample
from aerosat.core.faults import ConfigurationFault
from aerosat.core.faults import ConfigurationReason as Reason
from aerosat.core.types import (
    AvionicsEstimate,
    DataQuality,
    RejectedSample,
    RejectionReason,
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
    "touchdown_fallback_window_ms", "touchdown_fallback_acceleration_mps2",
    "deploy_confirm_baro_descent_m", "deploy_confirm_samples",
    "detector_sigma_p_pa", "detector_sigma_a_mps2", "detector_kappa",
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
    touchdown_fallback_window_ms: int
    """Fallback TOUCHDOWN criterion: window over which the barometer must be still."""
    touchdown_fallback_acceleration_mps2: float
    """Fallback TOUCHDOWN criterion: limit on |gravity-compensated estimated vertical acceleration|, m/s^2."""
    deploy_confirm_baro_descent_m: float
    """Deployment after an unreliable apogee: required barometric descent below the filtered peak, m."""
    deploy_confirm_samples: int
    """Deployment after an unreliable apogee: consecutive valid barometer samples required."""
    detector_sigma_p_pa: float
    """Disagreement detector: per-sample barometer noise including quantisation, Pa (F1_DECISIONS 6e.5)."""
    detector_sigma_a_mps2: float
    """Disagreement detector: per-sample accelerometer noise including quantisation, m/s^2."""
    detector_kappa: float
    """Disagreement detector: bound on the relative barometric altitude-scale error. Dimensionless."""
    drag_device_enabled: bool
    deploy_delay_after_apogee_ms: int
    liftoff_backup_delay_ms: int | None = None
    """F1-D6 liftoff-anchored backup timer: burn duration + backup_deployment_delay_ms. Derived by
    `load_avionics_config` from the scenario's thrust curve; None disables the timer."""

    def __post_init__(self) -> None:
        """Validate every value on its own. Raises `ConfigurationFault` naming the field."""
        for name in ("rate_hz", "telemetry_rate_hz", "calibration_duration_ms", "debounce_samples",
                     "backup_deployment_delay_ms", "expected_coast_duration_ms", "burnout_fallback_samples",
                     "touchdown_fallback_window_ms", "deploy_confirm_samples"):
            _require_int(self, name, minimum=1)
        for name in ("apogee_lockout_after_burnout_ms", "deploy_delay_after_apogee_ms"):
            _require_int(self, name, minimum=0)
        _require_int(self, "touchdown_window_ms", minimum=1)
        for name in ("liftoff_specific_force_mps2", "touchdown_altitude_range_m", "touchdown_speed_mps",
                     "estimator_gain_h", "estimator_gain_v_hz", "burnout_fallback_specific_force_mps2",
                     "touchdown_fallback_acceleration_mps2", "deploy_confirm_baro_descent_m",
                     "detector_sigma_p_pa", "detector_sigma_a_mps2", "detector_kappa"):
            _require_positive_number(self, name)
        if self.liftoff_backup_delay_ms is not None:
            _require_int(self, "liftoff_backup_delay_ms", minimum=1)
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
        if self.touchdown_fallback_window_ms < self.touchdown_window_ms:
            raise ConfigurationFault(
                Reason.OUT_OF_RANGE, "avionics.touchdown_fallback_window_ms",
                f"must be at least touchdown_window_ms ({self.touchdown_window_ms}), got "
                f"{self.touchdown_fallback_window_ms}: the fallback must not undercut the primary criterion",
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

    # F1-D6: the liftoff-anchored backup timer = burn duration (end of the thrust table) + backup delay.
    vehicle = _section(scenario, "vehicle")
    curve = vehicle.get("thrust_curve")
    if not isinstance(curve, list) or not curve:
        raise ConfigurationFault(Reason.MISSING_FIELD, "vehicle.thrust_curve", "required to derive the liftoff backup timer")
    burn_ms = curve[-1].get("t_ms") if isinstance(curve[-1], Mapping) else None
    if isinstance(burn_ms, bool) or not isinstance(burn_ms, int) or burn_ms < 0:
        raise ConfigurationFault(Reason.INVALID_TYPE, "vehicle.thrust_curve", f"last t_ms must be a non-negative integer, got {burn_ms!r}")
    return replace(config, liftoff_backup_delay_ms=burn_ms + config.backup_deployment_delay_ms)


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
    rejected: tuple[RejectedSample, ...] = ()
    """Samples refused on this tick: which source, when, what it read and why."""
    disagreement: DisagreementFlag | None = None
    """The latched F1 disagreement flag, once raised (F1_DECISIONS 6e.2)."""


DEGRADED_WINDOW_TICKS = 1000
"""A rejection keeps the data quality at DEGRADED for this long (1000 ms)."""

BARO_MAX_AGE_TICKS = 1000
"""Without a valid barometer sample for longer than this (1000 ms) the estimate is UNRELIABLE."""

ACCEL_MAX_AGE_TICKS = 100
"""The last valid acceleration is held for at most this long (100 ms). After that the estimator
predicts from velocity alone and the estimate is UNRELIABLE."""

ACCEL_Z, BARO, TEMP, GYRO_Z = ("accel", "z"), ("baro", "pressure"), ("temp", "temperature"), ("gyro", "z")
SOURCES = (ACCEL_Z, BARO, TEMP, GYRO_Z)
"""The sensor channels the avionics uses, in the order they are listed in `invalid_sources`."""


def source_name(source: tuple[str, str]) -> str:
    return f"{source[0]}.{source[1]}"


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
            touchdown_fallback_window_ticks=ms_to_ticks(config.touchdown_fallback_window_ms),
            touchdown_fallback_acceleration_mps2=config.touchdown_fallback_acceleration_mps2,
            period_ticks=self._period_ticks,
            drag_device_enabled=config.drag_device_enabled,
            deploy_delay_ticks=ms_to_ticks(config.deploy_delay_after_apogee_ms),
            backup_deployment_ticks=ms_to_ticks(config.backup_deployment_delay_ms),
            burnout_fallback_specific_force_mps2=config.burnout_fallback_specific_force_mps2,
            burnout_fallback_samples=config.burnout_fallback_samples,
            deploy_confirm_baro_descent_m=config.deploy_confirm_baro_descent_m,
            deploy_confirm_samples=config.deploy_confirm_samples,
            liftoff_backup_ticks=(None if config.liftoff_backup_delay_ms is None
                                  else ms_to_ticks(config.liftoff_backup_delay_ms)),
        ))
        self._config = config
        self._dead_reckoning: DeadReckoning | None = None
        self._monitor: DisagreementMonitor | None = None
        self._calibrated_tick: int | None = None
        # Last finite reading per source, for telemetry. 0.0 until a finite reading arrives.
        self._latest: dict[tuple[str, str], float] = {source: 0.0 for source in SOURCES}
        # Sources whose most recent sample was rejected, or that have never delivered a valid one.
        self._invalid: set[tuple[str, str]] = set(SOURCES)
        self._last_valid_tick: dict[tuple[str, str], int | None] = {source: None for source in SOURCES}
        self._last_rejection_tick: dict[tuple[str, str], int | None] = {source: None for source in SOURCES}
        self._valid_accel_z: float | None = None
        self._acceleration_mps2 = 0.0
        self._baro_altitude_m = 0.0

    @property
    def flight_state(self) -> FlightState:
        return self._fsm.state

    @property
    def disagreement_flag(self) -> DisagreementFlag | None:
        """The latched F1 disagreement flag, or None if it has not been raised."""
        return None if self._monitor is None else self._monitor.flag

    def tick(self, tick: int, samples: Sequence[SensorSample]) -> AvionicsOutput:
        """Process one avionics tick with the sensor samples that arrived since the previous one."""
        if tick % self._period_ticks != 0:
            raise ValueError(f"tick {tick} is not an avionics tick (period {self._period_ticks} ticks)")

        t_sim_s = tick_to_seconds(tick)

        # 1. Classify every sample of a used source. A rejected sample is never used below.
        valid: list[SensorSample] = []
        rejected: list[RejectedSample] = []
        arrived: set[tuple[str, str]] = set()
        for sample in samples:
            source = (sample.sensor_id, sample.channel)
            if source not in self._latest:
                continue                                  # a channel the avionics does not use
            arrived.add(source)
            finite = isinstance(sample.value, (int, float)) and math.isfinite(sample.value)
            if finite:
                self._latest[source] = sample.value       # the raw reading, clipped or not, for telemetry
            if finite and not sample.saturated:
                valid.append(sample)
                self._invalid.discard(source)
                self._last_valid_tick[source] = tick
            else:
                reason = RejectionReason.SATURATED if finite else RejectionReason.NON_FINITE
                rejected.append(RejectedSample(tick, t_sim_s, source_name(source), sample.value, reason))
                self._invalid.add(source)
                self._last_rejection_tick[source] = tick
        valid_sources = {(sample.sensor_id, sample.channel) for sample in valid}
        new_accel, new_baro = ACCEL_Z in valid_sources, BARO in valid_sources
        accel_rejected = ACCEL_Z in arrived and not new_accel
        baro_rejected = BARO in arrived and not new_baro
        if new_accel:
            self._valid_accel_z = self._latest[ACCEL_Z]

        events: list[FlightEvent] = []
        commands: list[VehicleCommand] = []
        just_calibrated = False
        if self.calibration is None:
            if tick < self._calibration_ticks:
                self._calibration.add(valid)
            else:
                self.calibration = self._calibration.result()
                just_calibrated = True

        if self.calibration is not None:
            calibration = self.calibration
            if just_calibrated:
                period_s = tick_to_seconds(self._period_ticks)
                self._calibrated_tick = tick
                self._dead_reckoning = DeadReckoning(period_s)
                self._monitor = DisagreementMonitor(
                    self._config.detector_sigma_p_pa, self._config.detector_sigma_a_mps2,
                    self._config.detector_kappa, calibration, period_s,
                )
            # 2. Prediction. The last valid acceleration is held for at most ACCEL_MAX_AGE_TICKS;
            #    after that the estimator predicts from velocity alone (zero acceleration).
            accel_age = tick - self._last_valid_tick[ACCEL_Z]
            specific_force = self._valid_accel_z - calibration.accel_bias_mps2
            if accel_age <= ACCEL_MAX_AGE_TICKS:
                self._acceleration_mps2 = estimated_acceleration_mps2(self._valid_accel_z, calibration.accel_bias_mps2)
                predicted_with = self._acceleration_mps2
            else:
                predicted_with = 0.0
            if not just_calibrated:
                self._filter.predict(predicted_with)
            # 3. Correction, only from a valid barometer sample.
            baro_sample_m = None
            if new_baro:
                self._baro_altitude_m = baro_sample_m = baro_altitude_agl_m(
                    self._latest[BARO], calibration.reference_pressure_pa, calibration.reference_temperature_k,
                )
                self._filter.correct(self._baro_altitude_m)
            # 4. Dead reckoning and the disagreement detector (F1_DECISIONS 6e.3, steps 3 and 4).
            self._update_disagreement(tick, just_calibrated, accel_age, baro_sample_m, baro_rejected)
            # 5. Estimate quality, including a raised disagreement flag.
            estimate_quality = self._estimate_quality(tick)
            events, commands = self._fsm.update(
                tick,
                specific_force_mps2=specific_force if new_accel else None,
                altitude_m=self._filter.altitude_agl_m,
                velocity_mps=self._filter.velocity_mps,
                baro_altitude_m=self._baro_altitude_m,
                acceleration_mps2=self._acceleration_mps2,
                estimate_quality=estimate_quality,
                accel_rejected=accel_rejected,
                baro_sample_m=baro_sample_m,
                baro_rejected=baro_rejected,
                disagreement=self.disagreement_flag,
            )
        else:
            estimate_quality = self._estimate_quality(tick)

        state = self._fsm.state
        estimate = AvionicsEstimate(
            tick, t_sim_s, self._filter.altitude_agl_m, self._filter.velocity_mps, self._acceleration_mps2,
            self._baro_altitude_m, state, estimate_quality,
        )
        frame = None
        if tick % self._telemetry_ticks == 0:
            frame = TelemetryFrame(
                tick, t_sim_s, estimate.altitude_agl_m, estimate.velocity_mps, estimate.acceleration_mps2,
                self._latest[BARO], self._latest[TEMP], state, self._latest[GYRO_Z],
                data_quality=self._frame_quality(tick, estimate_quality),
                invalid_sources=tuple(source_name(source) for source in SOURCES if source in self._invalid),
            )
        return AvionicsOutput(estimate, state, tuple(events), tuple(commands), frame, tuple(rejected),
                              self.disagreement_flag)

    def _update_disagreement(self, tick: int, just_calibrated: bool, accel_age: int, baro_sample_m: float | None,
                             baro_rejected: bool) -> None:
        """Advance the accelerometer-only dead reckoning and feed the disagreement detector. The detector
        runs from calibration until the deployment decision (F1_DECISIONS 6e.1); a raised flag stays."""
        if self._fsm.deployment_decided:
            return
        dead_reckoning, monitor = self._dead_reckoning, self._monitor
        if accel_age > ACCEL_MAX_AGE_TICKS:
            dead_reckoning.invalidate()
            monitor.uncheckable(tick)
        elif not just_calibrated:
            dead_reckoning.step(self._acceleration_mps2)
        if baro_rejected:
            monitor.barometer_rejected()
        if baro_sample_m is not None and dead_reckoning.valid:
            monitor.barometer_sample(
                tick, baro_sample_m, self._latest[BARO], dead_reckoning.altitude_m,
                tick_to_seconds(tick - self._calibrated_tick), self._filter.velocity_mps,
            )

    def _too_old(self, source: tuple[str, str], tick: int, max_age_ticks: int) -> bool:
        last = self._last_valid_tick[source]
        return last is None or tick - last > max_age_ticks

    def _recently_rejected(self, sources: Sequence[tuple[str, str]], tick: int) -> bool:
        return any(
            source in self._invalid
            or (self._last_rejection_tick[source] is not None
                and tick - self._last_rejection_tick[source] < DEGRADED_WINDOW_TICKS)
            for source in sources
        )

    def _estimate_quality(self, tick: int) -> DataQuality:
        """Reliability of the estimate. Only the estimator's inputs count: accel.z and baro.pressure.
        A raised disagreement flag makes it UNRELIABLE for the rest of the flight (F1-D4, 6e.2)."""
        if self.disagreement_flag is not None:
            return DataQuality.UNRELIABLE
        if self._too_old(ACCEL_Z, tick, ACCEL_MAX_AGE_TICKS) or self._too_old(BARO, tick, BARO_MAX_AGE_TICKS):
            return DataQuality.UNRELIABLE
        if self._recently_rejected((ACCEL_Z, BARO), tick):
            return DataQuality.DEGRADED
        return DataQuality.NOMINAL

    def _frame_quality(self, tick: int, estimate_quality: DataQuality) -> DataQuality:
        """Quality of a telemetry frame: UNRELIABLE estimates first, then any rejection on any source."""
        if estimate_quality is DataQuality.UNRELIABLE:
            return DataQuality.UNRELIABLE
        return DataQuality.DEGRADED if self._recently_rejected(SOURCES, tick) else DataQuality.NOMINAL
