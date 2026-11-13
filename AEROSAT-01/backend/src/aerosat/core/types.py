"""TruthState, SensorSample, AvionicsEstimate, FlightEvent, VehicleCommand, DownlinkMessage.

Distinct types per data category, so a truth value cannot be passed where a measurement is
expected without a visible conversion (ARCHITECTURE section 3).

Implemented so far: TruthState and VehicleCommand (build step 1), SensorSample (build step 2),
FlightState, FlightEvent, AvionicsEstimate and TelemetryFrame (build step 3). DownlinkMessage
(a sequenced telemetry frame or event) is added with the telemetry subsystem.
"""
from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import StrEnum


class FlightState(StrEnum):
    """Flight states of the avionics computer. Transitions are one-way, in this order."""

    PAD_IDLE = "PAD_IDLE"
    BOOST = "BOOST"
    COAST = "COAST"
    DESCENT = "DESCENT"
    LANDED = "LANDED"


class FlightEventType(StrEnum):
    """Flight events detected by the avionics computer."""

    LIFTOFF = "LIFTOFF"
    BURNOUT = "BURNOUT"
    APOGEE = "APOGEE"
    DRAG_DEVICE_DEPLOYED = "DRAG_DEVICE_DEPLOYED"
    TOUCHDOWN = "TOUCHDOWN"


class EventSource(StrEnum):
    """How the avionics arrived at a flight event."""

    DETECTED = "DETECTED"
    """Directly detected from estimates or measurements by the event's primary criterion."""
    FALLBACK_CRITERION = "FALLBACK_CRITERION"
    """Directly detected from measurements, but by the event's fallback criterion."""
    BACKUP_TIMER = "BACKUP_TIMER"
    """Inferred: the backup deployment timer expired before the event was detected."""
    BAROMETRIC_CONFIRMATION = "BAROMETRIC_CONFIRMATION"
    """Drag-device deployment confirmed by barometric descent after an unreliable apogee."""


class DataQuality(StrEnum):
    """How far an estimate, a telemetry frame or an event can be trusted.

    For an event it describes the evidence used for that particular event. It is not copied
    from the telemetry frame, and it is never rewritten after the event is recorded.
    """

    NOMINAL = "NOMINAL"
    DEGRADED = "DEGRADED"
    UNRELIABLE = "UNRELIABLE"


class RejectionReason(StrEnum):
    """Why the avionics refused to use a sensor sample."""

    SATURATED = "SATURATED"
    """The sensor clipped the reading at its range limit."""
    NON_FINITE = "NON_FINITE"
    """The reading is NaN or infinite."""


@dataclass(frozen=True, slots=True)
class RejectedSample:
    """A sensor sample the avionics refused to use: which source, when, what it read and why."""

    tick: int
    t_sim_s: float
    source: str
    """Sensor and channel, for example "baro.pressure"."""
    value: float
    reason: RejectionReason


@dataclass(frozen=True, slots=True)
class FlightEvent:
    """A flight event recorded by the avionics. `t_sim_s` is the time it was recorded, not the true event time.

    `source` separates a directly detected event from an inferred one. An inferred event carries
    quality UNRELIABLE and a `reason` saying why it was inferred.
    """

    event: FlightEventType
    tick: int
    t_sim_s: float
    flight_state: FlightState
    """State after the event."""
    detail: Mapping[str, float] = field(default_factory=dict)
    """Event-specific estimates, for example {"altitude_m": ...} for APOGEE. May be empty."""
    source: EventSource = EventSource.DETECTED
    quality: DataQuality = DataQuality.NOMINAL
    reason: str | None = None
    """Why the event was inferred. None for a directly detected event."""


@dataclass(frozen=True, slots=True)
class AvionicsEstimate:
    """ESTIMATE. The avionics computer's belief about the flight, computed only from measurements.

    Positive is up. Altitudes are above the pad (AGL).
    """

    tick: int
    t_sim_s: float
    altitude_agl_m: float
    velocity_mps: float
    acceleration_mps2: float
    baro_altitude_agl_m: float
    flight_state: FlightState
    quality: DataQuality = DataQuality.NOMINAL
    """Reliability of the estimate. Depends only on the estimator's inputs: accel.z and baro.pressure."""


@dataclass(frozen=True, slots=True)
class TelemetryFrame:
    """TELEMETRY content produced by the avionics every 50 ms: estimates and raw measurements.

    `altitude_m` (AGL), `velocity_mps` and `acceleration_mps2` are estimates. `pressure_pa`,
    `temperature_k` and `roll_rate_radps` are the most recent sensor readings. None is truth.

    A reading whose source is listed in `invalid_sources` is not a valid measurement: it is the raw
    clipped value, or the last finite value, and must never be presented as valid
    (TELEMETRY_PROTOCOL section 3.4).
    """

    tick: int
    t_sim_s: float
    altitude_m: float
    velocity_mps: float
    acceleration_mps2: float
    pressure_pa: float
    temperature_k: float
    flight_state: FlightState
    roll_rate_radps: float
    data_quality: DataQuality = DataQuality.NOMINAL
    invalid_sources: tuple[str, ...] = ()
    """Sources whose most recent sample was rejected, in the order accel.z, baro.pressure, temp.temperature, gyro.z."""


class VehicleCommand(StrEnum):
    """Commands the avionics computer may send to the vehicle."""

    DEPLOY_DRAG_DEVICE = "DEPLOY_DRAG_DEVICE"


@dataclass(frozen=True, slots=True)
class SensorSample:
    """MEASUREMENT. One reading of one sensor channel (FR-SEN-05).

    Simulated data: truth plus lag, bias, noise, saturation and quantization. `value` is in `unit`.
    """

    tick: int
    t_sim_s: float
    sensor_id: str
    channel: str
    value: float
    unit: str
    saturated: bool


@dataclass(frozen=True, slots=True)
class TruthState:
    """TRUTH. The simulator's exact state at one tick (FR-SIM-08). No real system ever sees this.

    Positive is up. `mach` is dimensionless. `on_ground` is true on the pad and after touchdown.
    """

    tick: int
    t_sim_s: float
    altitude_msl_m: float
    altitude_agl_m: float
    velocity_mps: float
    acceleration_mps2: float
    specific_force_mps2: float
    mass_kg: float
    thrust_n: float
    drag_n: float
    pressure_pa: float
    temperature_k: float
    density_kgpm3: float
    mach: float
    roll_rate_radps: float
    roll_angle_rad: float
    on_ground: bool
