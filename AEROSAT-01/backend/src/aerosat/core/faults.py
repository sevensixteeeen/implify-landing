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


@dataclass(frozen=True, slots=True)
class ValidityWarning:
    """A recorded warning that the model is being used where it is no longer credible."""

    tick: int
    t_sim_s: float
    message: str
    code: FaultCode = FaultCode.VALIDITY
