"""TruthState, SensorSample, AvionicsEstimate, FlightEvent, VehicleCommand, DownlinkMessage.

Distinct types per data category, so a truth value cannot be passed where a measurement is
expected without a visible conversion (ARCHITECTURE section 3).

Implemented so far: TruthState and VehicleCommand (build step 1). SensorSample,
AvionicsEstimate, FlightEvent and DownlinkMessage are added with the subsystems that produce them.
"""
from dataclasses import dataclass
from enum import StrEnum


class VehicleCommand(StrEnum):
    """Commands the avionics computer may send to the vehicle."""

    DEPLOY_DRAG_DEVICE = "DEPLOY_DRAG_DEVICE"


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
