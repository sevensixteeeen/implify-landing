"""Ties the above together; frames and commands.

ARCHITECTURE section 6. Runs at the avionics rate (200 Hz in REF-01). Consumes only sensor samples
and its own tick count (FR-AVI-01). It knows g0, the standard-atmosphere constants and its own
configuration, and nothing else about the vehicle or the truth model.
"""
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from aerosat.avionics.calibration import CalibrationResult, PadCalibration
from aerosat.avionics.estimator import ComplementaryFilter, baro_altitude_agl_m, estimated_acceleration_mps2
from aerosat.avionics.state_machine import FlightStateMachine, StateMachineConfig
from aerosat.core.clock import ms_to_ticks, tick_to_seconds, ticks_per_sample
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
    drag_device_enabled: bool = False
    deploy_delay_after_apogee_ms: int = 0


def avionics_config_from_scenario(avionics: Mapping[str, Any], drag_device: Mapping[str, Any]) -> AvionicsConfig:
    """Read the `avionics` and `vehicle.drag_device` sections of a scenario. Names the offending field."""
    missing, extra = _AVIONICS_KEYS - set(avionics), set(avionics) - _AVIONICS_KEYS
    if missing:
        raise ValueError(f"avionics: missing field(s) {sorted(missing)}")
    if extra:
        raise ValueError(f"avionics: unknown field(s) {sorted(extra)}")
    for name in ("enabled", "deploy_delay_after_apogee_ms"):
        if name not in drag_device:
            raise ValueError(f"vehicle.drag_device.{name}: missing")
    if not isinstance(drag_device["enabled"], bool):
        raise ValueError("vehicle.drag_device.enabled: must be true or false")
    return AvionicsConfig(
        **{name: avionics[name] for name in _AVIONICS_KEYS},
        drag_device_enabled=drag_device["enabled"],
        deploy_delay_after_apogee_ms=drag_device["deploy_delay_after_apogee_ms"],
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
        if self._telemetry_ticks % self._period_ticks != 0:
            raise ValueError("avionics.telemetry_rate_hz: the telemetry period must be a multiple of the avionics period")
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
