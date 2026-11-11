"""Derivative function a(t, h, v); specific force.

ARCHITECTURE sections 4.3, 4.6 and 4.8. One axis, positive up, SI units.
"""
from collections.abc import Callable
from typing import Protocol

from aerosat.sim.atmosphere import AirState
from aerosat.sim.vehicle import Vehicle

State = tuple[float, ...]
Derivative = Callable[[float, State], State]


class Atmosphere(Protocol):
    def air(self, altitude_msl_m: float, *, clamp: bool = False) -> AirState: ...


def drag_force_n(density_kgpm3: float, cd_area_m2: float, velocity_mps: float) -> float:
    """Drag, N, with the sign of the velocity, so that subtracting it always opposes motion."""
    return 0.5 * density_kgpm3 * cd_area_m2 * velocity_mps * abs(velocity_mps)


def specific_force_mps2(acceleration_mps2: float, gravity_mps2: float) -> float:
    """What an accelerometer senses: f = a + g(h). +g at rest on the ground, about 0 in drag-free fall."""
    return acceleration_mps2 + gravity_mps2


class FlightModel:
    """The equations of motion dh/dt = v, dv/dt = (F_T - F_D)/m - g(h) for one vehicle and environment."""

    def __init__(
        self,
        vehicle: Vehicle,
        *,
        t_ignition_s: float,
        atmosphere: Atmosphere,
        gravity_mps2: Callable[[float], float],
        pad_altitude_msl_m: float,
    ) -> None:
        self.vehicle = vehicle
        self.t_ignition_s = t_ignition_s
        self.atmosphere = atmosphere
        self.gravity_mps2 = gravity_mps2
        self.pad_altitude_msl_m = pad_altitude_msl_m

    def derivative(self, cd_area_m2: float, *, hold_on_pad: bool = False) -> Derivative:
        """f(t, [h, v]) -> [v, a] for the integrators.

        Intermediate stage evaluations that dip below the pad use the pad-altitude atmosphere, and
        the range check is left to the accepted state at the end of the step. With `hold_on_pad`
        (the vehicle has never left the pad) a negative acceleration is replaced by zero.
        """
        thrust_n = self.vehicle.thrust_n
        mass_kg = self.vehicle.mass_kg
        air_at = self.atmosphere.air
        gravity_mps2 = self.gravity_mps2
        t_ignition_s = self.t_ignition_s
        pad_m = self.pad_altitude_msl_m

        def f(t_s: float, y: State) -> State:
            altitude_m, velocity_mps = y
            since_ignition_s = t_s - t_ignition_s
            air = air_at(altitude_m if altitude_m > pad_m else pad_m, clamp=True)
            drag_n = 0.5 * air.density_kgpm3 * cd_area_m2 * velocity_mps * abs(velocity_mps)
            acceleration = (thrust_n(since_ignition_s) - drag_n) / mass_kg(since_ignition_s) - gravity_mps2(altitude_m)
            if hold_on_pad and acceleration < 0.0:
                acceleration = 0.0
            return (velocity_mps, acceleration)

        return f
