"""MODEL_RANGE, VALIDITY, NON_FINITE_VALUE, ...

Fault codes shared by all subsystems. A fault stops a run; a warning is recorded and the run continues.
"""
from dataclasses import dataclass
from enum import StrEnum


class FaultCode(StrEnum):
    MODEL_RANGE = "MODEL_RANGE"
    VALIDITY = "VALIDITY"
    NON_FINITE_VALUE = "NON_FINITE_VALUE"
    RECORDER_FAILURE = "RECORDER_FAILURE"
    UNKNOWN_MISSION = "UNKNOWN_MISSION"
    INTERNAL = "INTERNAL"
    INVALID_CONFIGURATION = "INVALID_CONFIGURATION"


class ConfigurationReason(StrEnum):
    """Why a configuration was rejected. Carried by `ConfigurationFault`."""

    MISSING_FIELD = "MISSING_FIELD"
    """A required field is absent. Required fields have no silent default."""
    UNKNOWN_FIELD = "UNKNOWN_FIELD"
    """A field is present that the configuration does not define."""
    INVALID_TYPE = "INVALID_TYPE"
    """A value has the wrong type, or is not a finite number."""
    OUT_OF_RANGE = "OUT_OF_RANGE"
    """A value has the right type but is outside its allowed range."""
    INCONSISTENT_RATES = "INCONSISTENT_RATES"
    """A rate does not divide the physics rate, or two periods are not multiples of each other."""
    REQUIRED_SENSOR_DISABLED = "REQUIRED_SENSOR_DISABLED"
    """A sensor the avionics needs is disabled."""
    UNSTABLE_ESTIMATOR_GAINS = "UNSTABLE_ESTIMATOR_GAINS"
    """The estimator gains would make the complementary filter diverge."""
    CALIBRATION_EXCEEDS_PRE_IGNITION = "CALIBRATION_EXCEEDS_PRE_IGNITION"
    """Pad calibration would still be running at ignition."""
    NO_VALID_CALIBRATION_SAMPLES = "NO_VALID_CALIBRATION_SAMPLES"
    """A required sensor delivered no valid sample during pad calibration."""


class AerosatFault(Exception):
    """A fault that stops the run. Carries a `FaultCode`."""

    code: FaultCode = FaultCode.INTERNAL

    def __init__(self, message: str) -> None:
        super().__init__(message)
        self.message = message


class ModelRangeFault(AerosatFault):
    """The state left the range in which the model is valid (0-20 km geopotential altitude)."""

    code = FaultCode.MODEL_RANGE


class NonFiniteValueFault(AerosatFault):
    """A NaN or infinite value was produced where a finite number is required."""

    code = FaultCode.NON_FINITE_VALUE


class ConfigurationFault(AerosatFault, ValueError):
    """A scenario or configuration value is invalid. Raised before the first simulation tick.

    `field` is the dotted path of the offending value (for example "sensors.gyro.enabled") and
    `reason` says what is wrong with it. Also a `ValueError`, so generic callers can catch it.
    """

    code = FaultCode.INVALID_CONFIGURATION

    def __init__(self, reason: ConfigurationReason, field: str, detail: str) -> None:
        super().__init__(f"{field}: {detail} [{reason.value}]")
        self.reason = reason
        self.field = field
        self.detail = detail


@dataclass(frozen=True, slots=True)
class ValidityWarning:
    """A recorded warning that the model is being used where it is no longer credible."""

    tick: int
    t_sim_s: float
    message: str
    code: FaultCode = FaultCode.VALIDITY
