"""PHY-14 (TEST_PLAN section 4): the time base. `t == tick / 1000` exactly."""
import pytest

from aerosat.core.clock import TICK_S, TICKS_PER_SECOND, TickClock, ms_to_ticks, tick_to_seconds, ticks_per_sample


def test_phy_14_time_is_tick_over_1000_exactly() -> None:
    assert TICKS_PER_SECOND == 1000
    assert TICK_S == 0.001
    for tick in (0, 1, 5, 50, 999, 1000, 20_550, 123_457, 600_000):
        assert tick_to_seconds(tick) == tick / 1000


def test_phy_14_tick_600_000_is_exactly_600_seconds() -> None:
    assert tick_to_seconds(600_000) == 600.0


def test_phy_14_clock_does_not_accumulate_rounding_error() -> None:
    clock = TickClock()
    accumulated = 0.0
    for _ in range(100_000):
        clock.advance()
        accumulated += 0.001
    assert clock.tick == 100_000
    assert clock.t_sim_s == 100.0          # derived from the integer tick: exact
    assert accumulated != 100.0            # what repeated addition would have produced


def test_ticks_per_sample_matches_architecture_5_2() -> None:
    # Ticks per sample: barometer 20, accelerometer and gyroscope 5, temperature 100.
    assert [ticks_per_sample(rate) for rate in (50, 200, 10, 20, 1000)] == [20, 5, 100, 50, 1]


@pytest.mark.parametrize("bad_rate", [0, -50, 3, 300, 2.5, True])
def test_rates_that_do_not_divide_the_physics_rate_are_rejected(bad_rate: object) -> None:
    with pytest.raises(ValueError):
        ticks_per_sample(bad_rate)  # type: ignore[arg-type]


def test_milliseconds_must_be_integers() -> None:
    assert ms_to_ticks(3000) == 3000
    with pytest.raises(TypeError):
        ms_to_ticks(100.5)  # type: ignore[arg-type]
