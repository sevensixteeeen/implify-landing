"""SEN-12 (TEST_PLAN section 6): the Gaussian generator, plus seed derivation (ARCHITECTURE 5.3)."""
import hashlib
import statistics

import pytest

from aerosat.core.rng import RandomStream, derive_seed

# First 10 Box-Muller values for master seed 42, stream "baro.noise". Stored on 2026-11-11 from
# Python 3.14.10 on Windows x86-64, after the statistical checks below passed. This is a
# cross-platform regression vector, not an oracle: it proves the same numbers come out elsewhere.
STORED_FIRST_10 = [
    -0.6303859107485376, -0.6582329862236198, -1.797161199344445, -0.4000885541929948,
    -0.6912923272963963, 1.2831278905615098, 0.5159889862422969, -1.001415193941438,
    0.17130586271117226, 1.2787847634854272,
]


def test_seed_is_the_first_8_bytes_of_sha256() -> None:
    # 14827543983096070746 was computed with hashlib alone, outside the project code.
    assert derive_seed(42, "baro.noise") == 14827543983096070746
    assert derive_seed(7, "accel.z.noise") == int.from_bytes(hashlib.sha256(b"7:accel.z.noise").digest()[:8], "big")


def test_streams_with_different_names_or_seeds_have_different_seeds() -> None:
    seeds = {derive_seed(42, name) for name in ("baro.noise", "baro.bias", "accel.z.noise", "gyro.z.noise")}
    assert len(seeds) == 4
    assert derive_seed(42, "baro.noise") != derive_seed(43, "baro.noise")


def test_sen_12_box_muller_mean_and_standard_deviation() -> None:
    stream = RandomStream(42, "baro.noise")
    draws = [stream.standard_normal() for _ in range(100_000)]
    assert abs(statistics.fmean(draws)) <= 0.02
    assert abs(statistics.pstdev(draws) - 1.0) <= 0.01


def test_sen_12_first_10_values_match_the_stored_vector() -> None:
    stream = RandomStream(42, "baro.noise")
    assert [stream.standard_normal() for _ in range(10)] == pytest.approx(STORED_FIRST_10, rel=1e-12)


def test_same_seed_and_name_reproduce_the_same_sequence() -> None:
    a, b = RandomStream(5, "temp.noise"), RandomStream(5, "temp.noise")
    assert [a.standard_normal() for _ in range(1_000)] == [b.standard_normal() for _ in range(1_000)]


def test_streams_are_independent_of_each_other() -> None:
    alone = RandomStream(42, "accel.z.noise")
    expected = [alone.standard_normal() for _ in range(100)]
    interleaved, other = RandomStream(42, "accel.z.noise"), RandomStream(42, "baro.noise")
    observed = []
    for _ in range(100):
        other.standard_normal()
        observed.append(interleaved.standard_normal())
    assert observed == expected


def test_uniform_values_are_in_the_unit_interval() -> None:
    stream = RandomStream(1, "any")
    assert all(0.0 <= stream.uniform() < 1.0 for _ in range(10_000))
