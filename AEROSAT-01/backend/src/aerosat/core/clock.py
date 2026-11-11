"""Integer tick clock; tick <-> seconds.

Sim time is always derived from the integer tick counter (`tick / 1000`). It is never
accumulated by repeated floating-point addition (ARCHITECTURE section 4.1).
"""

TICKS_PER_SECOND = 1000
"""Physics rate in Hz. One tick is one millisecond of sim time."""

TICK_S = 1 / TICKS_PER_SECOND
"""Duration of one tick, s."""


def tick_to_seconds(tick: int) -> float:
    """Sim time in seconds for an integer tick count."""
    return tick / TICKS_PER_SECOND


def ms_to_ticks(t_ms: int) -> int:
    """Number of ticks in a whole number of milliseconds. Rejects non-integers."""
    if isinstance(t_ms, bool) or not isinstance(t_ms, int):
        raise TypeError(f"milliseconds must be an integer, got {t_ms!r}")
    return t_ms * TICKS_PER_SECOND // 1000


def ticks_per_sample(rate_hz: int) -> int:
    """Tick period of something that runs at `rate_hz`. The rate must divide the physics rate exactly."""
    if isinstance(rate_hz, bool) or not isinstance(rate_hz, int) or rate_hz <= 0:
        raise ValueError(f"rate must be a positive integer number of Hz, got {rate_hz!r}")
    if TICKS_PER_SECOND % rate_hz != 0:
        raise ValueError(f"{rate_hz} Hz does not divide the physics rate of {TICKS_PER_SECOND} Hz exactly")
    return TICKS_PER_SECOND // rate_hz


class TickClock:
    """Counts ticks from zero and reports sim time."""

    def __init__(self) -> None:
        self._tick = 0

    @property
    def tick(self) -> int:
        return self._tick

    @property
    def t_sim_s(self) -> float:
        """Sim time since scenario start, s."""
        return tick_to_seconds(self._tick)

    def advance(self) -> int:
        """Move forward one tick and return the new tick."""
        self._tick += 1
        return self._tick
