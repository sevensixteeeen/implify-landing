"""Flight states, debounced event detection.

ARCHITECTURE section 6.3. PAD_IDLE -> BOOST -> COAST -> DESCENT -> LANDED. Transitions are
one-way, each event fires once, and a debounce counter resets when its criterion is interrupted.

Event detection and drag-device deployment are separate. An APOGEE is always recorded when its
criterion is met, with the quality of the estimate it was detected from. Deployment then follows
exactly one of three paths, evaluated in this order on every tick:

1. Normal: a reliable (NOMINAL or DEGRADED) detected apogee, then the configured delay.
2. Barometric confirmation: an unreliable detected apogee, the same delay, and the configured
   number of consecutive valid barometer samples, each at least the configured descent below a
   spike-resistant peak (the highest median of five consecutive valid barometer samples).
3. Backup timer: the configured time after detected BURNOUT. It runs in COAST and DESCENT and is
   cancelled only by a reliable detected apogee. If it expires with no apogee recorded, apogee
   is inferred (source BACKUP_TIMER, quality UNRELIABLE).

If BURNOUT is never detected the timer never starts and none of these paths can deploy. That
is an unresolved safety requirement (a liftoff-anchored emergency timeout), not handled here.
"""
import statistics
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

PEAK_MEDIAN_SAMPLES = 5
"""Barometric peak filter: the median of this many consecutive valid barometer samples."""


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
    """Backup timer: time after detected BURNOUT at which the drag device is deployed regardless."""
    deploy_confirm_baro_descent_m: float
    """Barometric confirmation: required descent below the filtered barometric peak."""
    deploy_confirm_samples: int
    """Barometric confirmation: consecutive valid barometer samples required."""

    def __post_init__(self) -> None:
        if self.backup_deployment_ticks < 1:
            raise ValueError("backup_deployment_ticks must be at least 1")
        if self.debounce_samples < 1:
            raise ValueError("debounce_samples must be at least 1")
        if self.period_ticks < 1:
            raise ValueError("period_ticks must be at least 1")
        if self.touchdown_window_ticks < self.period_ticks:
            raise ValueError("touchdown window must span at least one avionics period")
        if self.deploy_confirm_samples < 1:
            raise ValueError("deploy_confirm_samples must be at least 1")
        if not self.deploy_confirm_baro_descent_m > 0:
            raise ValueError("deploy_confirm_baro_descent_m must be greater than zero")


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
        self._apogee_quality = DataQuality.NOMINAL
        self._apogee_reliable = False
        self._timer_fired = False
        self._deployed = False
        self._slow_ticks = 0
        self._window_samples = config.touchdown_window_ticks // config.period_ticks + 1
        self._altitudes: deque[float] = deque(maxlen=self._window_samples)
        self._fallback_window_samples = config.touchdown_fallback_window_ticks // config.period_ticks + 1
        self._fallback_window_count = 0
        self._fallback_window_min_m = 0.0
        self._fallback_window_max_m = 0.0
        self._baro_recent: deque[float] = deque(maxlen=PEAK_MEDIAN_SAMPLES)
        self._baro_peak_m: float | None = None
        self._descent_confirm_count = 0

    def update(
        self,
        tick: int,
        *,
        specific_force_mps2: float | None,
        altitude_m: float,
        velocity_mps: float,
        baro_altitude_m: float,
        acceleration_mps2: float,
        estimate_quality: DataQuality,
        accel_rejected: bool,
        baro_sample_m: float | None,
        baro_rejected: bool,
    ) -> tuple[list[FlightEvent], list[VehicleCommand]]:
        """One avionics tick.

        `specific_force_mps2` is the bias-corrected z-accelerometer reading if a new VALID sample
        arrived this tick, otherwise None. `altitude_m` (AGL) and `velocity_mps` are the current
        estimates and `estimate_quality` their reliability. `baro_altitude_m` is the latest valid
        barometric altitude (AGL) and `acceleration_mps2` the gravity-compensated estimated vertical
        acceleration; both feed the touchdown fallback. `baro_sample_m` is the barometric altitude
        of a new VALID barometer sample this tick, otherwise None. `accel_rejected` and
        `baro_rejected` say that a sample from that sensor arrived this tick and was rejected.
        """
        config = self._config
        events: list[FlightEvent] = []
        commands: list[VehicleCommand] = []

        if accel_rejected:
            # A rejected accelerometer sample breaks every run that counts accelerometer samples.
            if self.state in (FlightState.PAD_IDLE, FlightState.BOOST):
                self._count = 0
            self._burnout_fallback_count = 0
            self._fallback_window_count = 0
        if baro_rejected:
            self._fallback_window_count = 0      # the touchdown fallback needs a live barometer
        if self._burnout_tick is not None:
            self._track_barometric_descent(baro_sample_m, baro_rejected)

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
                # Directly detected apogee. Its quality is the reliability of the velocity estimate
                # it was detected from, fixed here and never rewritten. Only a reliable one
                # cancels the backup timer and permits normal deployment.
                self._apogee_tick = tick
                self._apogee_quality = estimate_quality
                self._apogee_reliable = estimate_quality is not DataQuality.UNRELIABLE
                events.append(self._enter(
                    FlightState.DESCENT, FlightEventType.APOGEE, tick, {"altitude_m": altitude_m},
                    quality=estimate_quality,
                ))
            elif self._timer_expired(tick):
                # Backup timer with no apogee recorded: apogee is inferred, not observed.
                self._apogee_tick = tick
                self._apogee_inferred = True
                self._apogee_quality = DataQuality.UNRELIABLE
                self._apogee_reliable = False
                events.append(self._enter(
                    FlightState.DESCENT, FlightEventType.APOGEE, tick, {"altitude_m": altitude_m},
                    source=EventSource.BACKUP_TIMER, quality=DataQuality.UNRELIABLE,
                    reason=self._timer_reason(),
                ))
            if self.state is FlightState.DESCENT:
                self._decide_deployment(tick, events, commands)

        elif self.state is FlightState.DESCENT:
            self._decide_deployment(tick, events, commands)
            self._altitudes.append(altitude_m)
            self._slow_ticks = self._slow_ticks + 1 if abs(velocity_mps) < config.touchdown_speed_mps else 0
            if not (accel_rejected or baro_rejected):
                self._advance_touchdown_fallback(baro_altitude_m, acceleration_mps2)
            if (
                self._slow_ticks >= self._window_samples
                and len(self._altitudes) == self._window_samples
                and max(self._altitudes) - min(self._altitudes) < config.touchdown_altitude_range_m
            ):
                # Primary criterion: the altitude and velocity estimates have settled. Its
                # evidence is the estimate, so it takes the estimate's quality.
                events.append(self._enter(
                    FlightState.LANDED, FlightEventType.TOUCHDOWN, tick, quality=estimate_quality,
                ))
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

    # ------------------------------------------------------------------ deployment

    def _timer_expired(self, tick: int) -> bool:
        """The backup timer runs from detected BURNOUT and fires at most once."""
        return (
            not self._timer_fired
            and self._burnout_tick is not None
            and tick - self._burnout_tick >= self._config.backup_deployment_ticks
        )

    def _timer_reason(self) -> str:
        return (
            f"backup deployment timer expired {self._config.backup_deployment_ticks} ms after detected "
            f"BURNOUT (tick {self._burnout_tick}) with no APOGEE detected"
            if self._apogee_inferred else
            f"backup deployment timer expired {self._config.backup_deployment_ticks} ms after detected "
            f"BURNOUT (tick {self._burnout_tick}); the detected APOGEE was unreliable and was not confirmed"
        )

    def _decide_deployment(self, tick: int, events: list[FlightEvent], commands: list[VehicleCommand]) -> None:
        """Deploy the drag device by exactly one path. Called on every tick in DESCENT."""
        config = self._config
        delay_elapsed = tick - self._apogee_tick >= config.deploy_delay_ticks
        if self._apogee_reliable:
            # 1. Normal path. A reliable apogee has cancelled the backup timer.
            if delay_elapsed:
                self._deploy(tick, events, commands, EventSource.DETECTED, self._apogee_quality, None)
            return
        if (
            not self._apogee_inferred
            and delay_elapsed
            and self._descent_confirm_count >= config.deploy_confirm_samples
        ):
            # 2. Barometric confirmation of an unreliable detected apogee.
            self._deploy(
                tick, events, commands, EventSource.BAROMETRIC_CONFIRMATION, DataQuality.DEGRADED,
                f"{config.deploy_confirm_samples} consecutive valid barometer samples at least "
                f"{config.deploy_confirm_baro_descent_m} m below the filtered peak of {self._baro_peak_m:.1f} m, "
                "after an unreliable APOGEE",
            )
        elif self._timer_expired(tick):
            # 3. Backup timer. It fires once, whether or not a device is fitted.
            self._timer_fired = True
            self._deploy(tick, events, commands, EventSource.BACKUP_TIMER, DataQuality.UNRELIABLE, self._timer_reason())

    def _deploy(self, tick: int, events: list[FlightEvent], commands: list[VehicleCommand],
                source: EventSource, quality: DataQuality, reason: str | None) -> None:
        if not self._config.drag_device_enabled or self._deployed:
            return
        self._deployed = True
        commands.append(VehicleCommand.DEPLOY_DRAG_DEVICE)
        events.append(FlightEvent(
            FlightEventType.DRAG_DEVICE_DEPLOYED, tick, tick_to_seconds(tick), self.state,
            source=source, quality=quality, reason=reason,
        ))

    def _track_barometric_descent(self, baro_sample_m: float | None, baro_rejected: bool) -> None:
        """Maintain the spike-resistant barometric peak and the descent confirmation count.

        Runs from BURNOUT onward, in COAST and DESCENT, from valid barometer samples only, so it
        does not depend on the estimator. A rejected sample empties the median buffer and resets
        the count: it can neither establish the peak nor count toward confirmation.
        """
        if baro_rejected:
            self._baro_recent.clear()
            self._descent_confirm_count = 0
            return
        if baro_sample_m is None:
            return
        self._baro_recent.append(baro_sample_m)
        if len(self._baro_recent) == PEAK_MEDIAN_SAMPLES:
            filtered_m = statistics.median(self._baro_recent)
            if self._baro_peak_m is None or filtered_m > self._baro_peak_m:
                self._baro_peak_m = filtered_m
        descended = (
            self._baro_peak_m is not None
            and self._baro_peak_m - baro_sample_m >= self._config.deploy_confirm_baro_descent_m
        )
        self._descent_confirm_count = self._descent_confirm_count + 1 if descended else 0

    # ------------------------------------------------------------------ touchdown fallback

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
        - A rejected accelerometer or barometer sample empties the window (handled by the caller).
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
               source: EventSource = EventSource.DETECTED, quality: DataQuality = DataQuality.NOMINAL,
               reason: str | None = None) -> FlightEvent:
        self.state = state
        self._count = 0
        self._burnout_fallback_count = 0
        return FlightEvent(
            event, tick, tick_to_seconds(tick), state, detail or {}, source=source, quality=quality, reason=reason,
        )
