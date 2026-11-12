"""Flight states, debounced event detection.

ARCHITECTURE section 6.3. PAD_IDLE -> BOOST -> COAST -> DESCENT -> LANDED. Transitions are
one-way, each event fires once, and a debounce counter resets when its criterion is interrupted.

Backup deployment timer: it starts at detected BURNOUT. If it expires while still in COAST, apogee
is inferred (source BACKUP_TIMER, quality UNRELIABLE), the state becomes DESCENT, and the drag
device is deployed by the timer itself. A directly detected apogee always takes precedence.
"""
from collections import deque
from dataclasses import dataclass

from aerosat.core.clock import tick_to_seconds
from aerosat.core.types import (
    DataQuality,
    EventSource,
    FlightEvent,
    FlightEventType,
    FlightState,
    VehicleCommand,
)


@dataclass(frozen=True)
class StateMachineConfig:
    """Detection criteria. Durations are in ticks (1 tick = 1 ms)."""

    liftoff_specific_force_mps2: float
    debounce_samples: int
    apogee_lockout_ticks: int
    touchdown_window_ticks: int
    touchdown_altitude_range_m: float
    touchdown_speed_mps: float
    touchdown_fallback_window_ticks: int
    """Touchdown fallback: how long the barometer must be still and the vehicle not accelerating."""
    touchdown_fallback_acceleration_mps2: float
    """Touchdown fallback: limit on |gravity-compensated estimated vertical acceleration|."""
    period_ticks: int
    """Avionics period: the spacing of calls to `update`."""
    drag_device_enabled: bool
    deploy_delay_ticks: int
    burnout_fallback_specific_force_mps2: float
    """Fallback BURNOUT criterion: specific force below this value..."""
    burnout_fallback_samples: int
    """...for this many consecutive accelerometer samples. Does not rely on drag."""
    backup_deployment_ticks: int
    """Backup timer: time after detected BURNOUT at which apogee is inferred if it has not been detected."""

    def __post_init__(self) -> None:
        if self.backup_deployment_ticks < 1:
            raise ValueError("backup_deployment_ticks must be at least 1")
        if self.debounce_samples < 1:
            raise ValueError("debounce_samples must be at least 1")
        if self.period_ticks < 1:
            raise ValueError("period_ticks must be at least 1")
        if self.touchdown_window_ticks < self.period_ticks:
            raise ValueError("touchdown window must span at least one avionics period")


class FlightStateMachine:
    """Debounced, one-way flight-state logic. Sees estimates and bias-corrected readings only."""

    def __init__(self, config: StateMachineConfig) -> None:
        self._config = config
        self.state = FlightState.PAD_IDLE
        self._count = 0
        self._burnout_fallback_count = 0
        self._burnout_tick: int | None = None
        self._apogee_tick: int | None = None
        self._apogee_inferred = False
        self._deployed = False
        self._slow_ticks = 0
        self._window_samples = config.touchdown_window_ticks // config.period_ticks + 1
        self._altitudes: deque[float] = deque(maxlen=self._window_samples)
        self._fallback_window_samples = config.touchdown_fallback_window_ticks // config.period_ticks + 1
        self._fallback_window_count = 0
        self._fallback_window_min_m = 0.0
        self._fallback_window_max_m = 0.0

    def update(
        self,
        tick: int,
        *,
        specific_force_mps2: float | None,
        altitude_m: float,
        velocity_mps: float,
        baro_altitude_m: float,
        acceleration_mps2: float,
    ) -> tuple[list[FlightEvent], list[VehicleCommand]]:
        """One avionics tick.

        `specific_force_mps2` is the bias-corrected z-accelerometer reading if a new sample arrived
        this tick, otherwise None. `altitude_m` (AGL) and `velocity_mps` are the current estimates.
        `baro_altitude_m` is the latest barometric altitude (AGL) and `acceleration_mps2` the
        gravity-compensated estimated vertical acceleration; both are used only by the touchdown
        fallback.
        """
        config = self._config
        events: list[FlightEvent] = []
        commands: list[VehicleCommand] = []

        if self.state is FlightState.PAD_IDLE:
            if specific_force_mps2 is not None:
                self._debounce(specific_force_mps2 > config.liftoff_specific_force_mps2)
                if self._count >= config.debounce_samples:
                    events.append(self._enter(FlightState.BOOST, FlightEventType.LIFTOFF, tick))

        elif self.state is FlightState.BOOST:
            if specific_force_mps2 is not None:
                self._debounce(specific_force_mps2 < 0.0)
                below = specific_force_mps2 < config.burnout_fallback_specific_force_mps2
                self._burnout_fallback_count = self._burnout_fallback_count + 1 if below else 0
                if self._count >= config.debounce_samples:
                    # Primary criterion: drag makes the specific force negative once thrust ends.
                    self._burnout_tick = tick
                    events.append(self._enter(FlightState.COAST, FlightEventType.BURNOUT, tick))
                elif self._burnout_fallback_count >= config.burnout_fallback_samples:
                    # Fallback criterion: thrust has ended even though drag is too small to show it.
                    self._burnout_tick = tick
                    events.append(self._enter(
                        FlightState.COAST, FlightEventType.BURNOUT, tick,
                        source=EventSource.FALLBACK_CRITERION,
                        reason=(
                            f"specific force below {config.burnout_fallback_specific_force_mps2} m/s^2 for "
                            f"{config.burnout_fallback_samples} consecutive samples"
                        ),
                    ))

        elif self.state is FlightState.COAST:
            self._debounce(velocity_mps < 0.0)
            locked_out = tick - self._burnout_tick < config.apogee_lockout_ticks
            if self._count >= config.debounce_samples and not locked_out:
                # Directly detected apogee. It takes precedence over the backup timer, which is now moot.
                self._apogee_tick = tick
                events.append(self._enter(FlightState.DESCENT, FlightEventType.APOGEE, tick, {"altitude_m": altitude_m}))
            elif tick - self._burnout_tick >= config.backup_deployment_ticks:
                # Backup timer: apogee was not detected in time, so it is inferred, not observed.
                self._apogee_tick = tick
                self._apogee_inferred = True
                reason = (
                    f"backup deployment timer expired {config.backup_deployment_ticks} ms after detected "
                    f"BURNOUT (tick {self._burnout_tick}) with no APOGEE detected"
                )
                self.state = FlightState.DESCENT
                self._count = 0
                events.append(FlightEvent(
                    FlightEventType.APOGEE, tick, tick_to_seconds(tick), self.state, {"altitude_m": altitude_m},
                    source=EventSource.BACKUP_TIMER, quality=DataQuality.UNRELIABLE, reason=reason,
                ))
                # The timer is its own deployment path. It does not go through the normal
                # apogee-plus-delay path, which an inferred apogee is never allowed to trigger.
                if config.drag_device_enabled and not self._deployed:
                    self._deployed = True
                    commands.append(VehicleCommand.DEPLOY_DRAG_DEVICE)
                    events.append(FlightEvent(
                        FlightEventType.DRAG_DEVICE_DEPLOYED, tick, tick_to_seconds(tick), self.state,
                        source=EventSource.BACKUP_TIMER, quality=DataQuality.UNRELIABLE, reason=reason,
                    ))

        elif self.state is FlightState.DESCENT:
            if (
                config.drag_device_enabled
                and not self._deployed
                and not self._apogee_inferred  # normal deployment needs a directly detected apogee
                and tick - self._apogee_tick >= config.deploy_delay_ticks
            ):
                self._deployed = True
                commands.append(VehicleCommand.DEPLOY_DRAG_DEVICE)
                events.append(FlightEvent(FlightEventType.DRAG_DEVICE_DEPLOYED, tick, tick_to_seconds(tick), self.state))
            self._altitudes.append(altitude_m)
            self._slow_ticks = self._slow_ticks + 1 if abs(velocity_mps) < config.touchdown_speed_mps else 0
            self._advance_touchdown_fallback(baro_altitude_m, acceleration_mps2)
            if (
                self._slow_ticks >= self._window_samples
                and len(self._altitudes) == self._window_samples
                and max(self._altitudes) - min(self._altitudes) < config.touchdown_altitude_range_m
            ):
                # Primary criterion: the altitude and velocity estimates have settled.
                events.append(self._enter(FlightState.LANDED, FlightEventType.TOUCHDOWN, tick))
            elif self._fallback_window_count >= self._fallback_window_samples:
                # Fallback criterion: the barometer is still and the vehicle is not accelerating,
                # whatever the velocity estimate says (it is left badly wrong by a hard impact).
                events.append(self._enter(
                    FlightState.LANDED, FlightEventType.TOUCHDOWN, tick,
                    source=EventSource.FALLBACK_CRITERION,
                    reason=(
                        f"barometric altitude within {config.touchdown_altitude_range_m} m and estimated "
                        f"acceleration below {config.touchdown_fallback_acceleration_mps2} m/s^2 for "
                        f"{config.touchdown_fallback_window_ticks} ms"
                    ),
                ))

        return events, commands

    def _advance_touchdown_fallback(self, baro_altitude_m: float, acceleration_mps2: float) -> None:
        """Advance the touchdown fallback window by one avionics tick.

        The window is the run of consecutive ticks, ending now, over which the barometric altitude
        stayed within `touchdown_altitude_range_m` and |estimated acceleration| stayed below
        `touchdown_fallback_acceleration_mps2`. It qualifies once it spans the fallback window.

        Reset behaviour:
        - A tick whose acceleration is at or above the limit empties the window. The next
          qualifying tick starts a new one.
        - A barometric altitude that takes the window's range to or beyond the limit starts a new
          window at that tick: the new altitude becomes the window's only sample.
        """
        config = self._config
        if abs(acceleration_mps2) >= config.touchdown_fallback_acceleration_mps2:
            self._fallback_window_count = 0
            return
        if self._fallback_window_count > 0:
            lowest = min(self._fallback_window_min_m, baro_altitude_m)
            highest = max(self._fallback_window_max_m, baro_altitude_m)
            if highest - lowest < config.touchdown_altitude_range_m:
                self._fallback_window_min_m, self._fallback_window_max_m = lowest, highest
                self._fallback_window_count += 1
                return
        self._fallback_window_min_m = self._fallback_window_max_m = baro_altitude_m
        self._fallback_window_count = 1

    def _debounce(self, criterion: bool) -> None:
        self._count = self._count + 1 if criterion else 0

    def _enter(self, state: FlightState, event: FlightEventType, tick: int,
               detail: dict[str, float] | None = None, *,
               source: EventSource = EventSource.DETECTED, reason: str | None = None) -> FlightEvent:
        self.state = state
        self._count = 0
        self._burnout_fallback_count = 0
        return FlightEvent(event, tick, tick_to_seconds(tick), state, detail or {}, source=source, reason=reason)
