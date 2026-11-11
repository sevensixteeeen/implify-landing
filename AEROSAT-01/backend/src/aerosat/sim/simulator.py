"""Stepping, pad hold, touchdown, commands, truth events.

ARCHITECTURE sections 4.8 and 7. Produces TRUTH only. No I/O, no clock, no knowledge of sensors.
"""
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field, replace

from aerosat.core.clock import TICK_S, ms_to_ticks, tick_to_seconds
from aerosat.core.faults import ValidityWarning
from aerosat.core.types import TruthState, VehicleCommand
from aerosat.sim.atmosphere import StandardAtmosphere
from aerosat.sim.dynamics import Atmosphere, FlightModel, drag_force_n, specific_force_mps2
from aerosat.sim.gravity import inverse_square_gravity_mps2
from aerosat.sim.integrators import INTEGRATORS
from aerosat.sim.vehicle import RollProfile, Vehicle

MACH_VALIDITY_LIMIT = 0.8
"""Above this Mach number a constant drag coefficient is no longer credible. Dimensionless."""


@dataclass(frozen=True)
class DragDevice:
    """Descent drag device. `cd_area_m2` replaces the vehicle's drag area once deployed."""

    enabled: bool = False
    cd_area_m2: float = 0.0


@dataclass(frozen=True)
class SimulatorConfig:
    """Everything the simulator needs. Built from a scenario by the mission layer.

    `initial_altitude_agl_m` and `initial_velocity_mps` start the run already in flight. They exist
    for verification against closed-form solutions and are not scenario parameters.
    """

    vehicle: Vehicle
    t_ignition_ms: int = 0
    pad_altitude_msl_m: float = 0.0
    atmosphere: Atmosphere = field(default_factory=StandardAtmosphere)
    gravity_mps2: Callable[[float], float] = inverse_square_gravity_mps2
    integrator: str = "rk4"
    drag_device: DragDevice = field(default_factory=DragDevice)
    roll_profile: RollProfile = field(default_factory=lambda: RollProfile(()))
    post_touchdown_hold_ms: int = 5000
    max_duration_ms: int = 600_000
    initial_altitude_agl_m: float = 0.0
    initial_velocity_mps: float = 0.0

    def __post_init__(self) -> None:
        if self.integrator not in INTEGRATORS:
            raise ValueError(f"unknown integrator {self.integrator!r}; expected one of {sorted(INTEGRATORS)}")
        if self.initial_altitude_agl_m < 0.0:
            raise ValueError("initial_altitude_agl_m must not be negative")


@dataclass(frozen=True, slots=True)
class TruthEvents:
    """TRUTH. True event times, for comparison with what the avionics detects.

    liftoff = first tick with h > h_pad; burnout = end of the thrust table; apogee = tick of
    maximum h; touchdown = ground-contact time interpolated inside the step.
    """

    liftoff_tick: int | None = None
    burnout_tick: int | None = None
    apogee_tick: int | None = None
    apogee_altitude_agl_m: float | None = None
    touchdown_tick: int | None = None
    touchdown_time_s: float | None = None
    impact_velocity_mps: float | None = None


class Simulator:
    """Advances truth one tick (1 ms) at a time.

    `state` is the truth at the current tick; tick 0 is the initial state, before any step.
    `step(commands)` applies the commands issued on the previous tick, then advances one tick.
    """

    def __init__(self, config: SimulatorConfig) -> None:
        self._config = config
        self._model = FlightModel(
            config.vehicle,
            t_ignition_s=tick_to_seconds(ms_to_ticks(config.t_ignition_ms)),
            atmosphere=config.atmosphere,
            gravity_mps2=config.gravity_mps2,
            pad_altitude_msl_m=config.pad_altitude_msl_m,
        )
        self._integrate = INTEGRATORS[config.integrator]
        self._hold_ticks = ms_to_ticks(config.post_touchdown_hold_ms)
        self._max_ticks = ms_to_ticks(config.max_duration_ms)

        self._tick = 0
        self._altitude_m = config.pad_altitude_msl_m + config.initial_altitude_agl_m
        self._velocity_mps = config.initial_velocity_mps
        self._left_pad = config.initial_altitude_agl_m > 0.0 or config.initial_velocity_mps != 0.0
        self._landed = False
        self._cd_area_m2 = config.vehicle.cd_area_m2
        self._deployed = False
        self._derivative = self._model.derivative(self._cd_area_m2, hold_on_pad=not self._left_pad)

        curve = config.vehicle.thrust_curve
        self._burnout_tick = (
            ms_to_ticks(config.t_ignition_ms) + round(curve.burn_duration_s * 1000) if curve.times_s else None
        )
        self._events = TruthEvents()
        self._max_altitude_m = self._altitude_m
        self._above_mach_limit = False
        self._warnings: list[ValidityWarning] = []
        self._state = self._build_state()

    # ------------------------------------------------------------------ read-only views

    @property
    def state(self) -> TruthState:
        return self._state

    @property
    def tick(self) -> int:
        return self._tick

    @property
    def cd_area_m2(self) -> float:
        """Drag area currently in effect, m^2."""
        return self._cd_area_m2

    @property
    def truth_events(self) -> TruthEvents:
        return self._events

    @property
    def warnings(self) -> tuple[ValidityWarning, ...]:
        return tuple(self._warnings)

    @property
    def finished(self) -> bool:
        """True once the post-touchdown hold has elapsed or the maximum duration is reached."""
        if self._tick >= self._max_ticks:
            return True
        touchdown_tick = self._events.touchdown_tick
        return touchdown_tick is not None and self._tick >= touchdown_tick + self._hold_ticks

    # ------------------------------------------------------------------ stepping

    def step(self, commands: Sequence[VehicleCommand] = ()) -> TruthState:
        """Apply commands issued on the previous tick, advance one tick, and return the new truth.

        Raises `ModelRangeFault` if the new state is outside the 0-20 km model range.
        """
        for command in commands:
            self._apply(command)

        t_s = tick_to_seconds(self._tick)
        self._tick += 1
        if not self._landed:
            altitude_m, velocity_mps = self._integrate(
                self._derivative, t_s, (self._altitude_m, self._velocity_mps), TICK_S
            )
            pad_m = self._config.pad_altitude_msl_m
            if not self._left_pad:
                # The pad-hold derivative never lets the acceleration go negative, so the vehicle
                # cannot sink below the pad or gain downward speed here. Explicit Euler gains
                # speed one tick before it gains height, so speed alone is not yet liftoff.
                if altitude_m > pad_m:
                    self._left_pad = True
                    self._events = replace(self._events, liftoff_tick=self._tick)
                    self._derivative = self._model.derivative(self._cd_area_m2, hold_on_pad=False)
            elif altitude_m <= pad_m and velocity_mps < 0.0:
                fallen_m = self._altitude_m - altitude_m
                fraction = (self._altitude_m - pad_m) / fallen_m if fallen_m > 0.0 else 0.0
                self._events = replace(
                    self._events,
                    touchdown_tick=self._tick,
                    touchdown_time_s=t_s + fraction * TICK_S,
                    impact_velocity_mps=self._velocity_mps + fraction * (velocity_mps - self._velocity_mps),
                )
                altitude_m, velocity_mps = pad_m, 0.0
                self._landed = True
            self._altitude_m, self._velocity_mps = altitude_m, velocity_mps

        self._state = self._build_state()
        return self._state

    def _apply(self, command: VehicleCommand) -> None:
        if command is VehicleCommand.DEPLOY_DRAG_DEVICE:
            if self._config.drag_device.enabled and not self._deployed:
                self._deployed = True
                self._cd_area_m2 = self._config.drag_device.cd_area_m2
                self._derivative = self._model.derivative(self._cd_area_m2, hold_on_pad=not self._left_pad)
        else:
            raise ValueError(f"unknown vehicle command {command!r}")

    def _build_state(self) -> TruthState:
        config = self._config
        tick = self._tick
        t_s = tick_to_seconds(tick)
        altitude_m, velocity_mps = self._altitude_m, self._velocity_mps

        air = config.atmosphere.air(altitude_m)  # raises ModelRangeFault outside 0-20 km
        gravity = config.gravity_mps2(altitude_m)
        since_ignition_s = t_s - self._model.t_ignition_s
        thrust_n = config.vehicle.thrust_n(since_ignition_s)
        mass_kg = config.vehicle.mass_kg(since_ignition_s)
        drag_n = drag_force_n(air.density_kgpm3, self._cd_area_m2, velocity_mps)

        if self._landed:
            acceleration = 0.0
        else:
            acceleration = (thrust_n - drag_n) / mass_kg - gravity
            if not self._left_pad and acceleration < 0.0:
                acceleration = 0.0  # pad hold

        if self._left_pad and not self._landed and (
            self._events.apogee_tick is None or altitude_m > self._max_altitude_m
        ):
            self._max_altitude_m = altitude_m
            self._events = replace(
                self._events,
                apogee_tick=tick,
                apogee_altitude_agl_m=altitude_m - config.pad_altitude_msl_m,
            )
        if self._burnout_tick is not None and tick >= self._burnout_tick and self._events.burnout_tick is None:
            self._events = replace(self._events, burnout_tick=self._burnout_tick)

        mach = abs(velocity_mps) / air.speed_of_sound_mps
        above = mach > MACH_VALIDITY_LIMIT
        if above and not self._above_mach_limit:
            self._warnings.append(
                ValidityWarning(tick, t_s, f"Mach {mach:.3f} exceeds {MACH_VALIDITY_LIMIT}: constant drag coefficient is not credible.")
            )
        self._above_mach_limit = above

        return TruthState(
            tick=tick,
            t_sim_s=t_s,
            altitude_msl_m=altitude_m,
            altitude_agl_m=altitude_m - config.pad_altitude_msl_m,
            velocity_mps=velocity_mps,
            acceleration_mps2=acceleration,
            specific_force_mps2=specific_force_mps2(acceleration, gravity),
            mass_kg=mass_kg,
            thrust_n=thrust_n,
            drag_n=drag_n,
            pressure_pa=air.pressure_pa,
            temperature_k=air.temperature_k,
            density_kgpm3=air.density_kgpm3,
            mach=mach,
            roll_rate_radps=config.roll_profile.roll_rate_radps(t_s),
            roll_angle_rad=config.roll_profile.roll_angle_rad(t_s),
            on_ground=self._landed or not self._left_pad,
        )
