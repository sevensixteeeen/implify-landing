"""lag -> bias -> noise -> saturate -> quantize.

ARCHITECTURE section 5.1. For each channel, at each sample tick k:

    x_k   = true quantity
    x'_k  = lag(x_k)                          only where a time constant is defined
    y_k   = Q( sat( x'_k + b + sigma*w_k ) )  w_k ~ N(0, 1), independent

All sensor parameters are illustrative simulation values, not taken from any datasheet.
"""
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from aerosat.core.clock import ticks_per_sample, tick_to_seconds
from aerosat.core.rng import RandomStream
from aerosat.core.types import SensorSample, TruthState


@dataclass(frozen=True)
class ErrorModel:
    """Error terms of one sensor. Every value is in the sensor's own unit, except `lag_tau_s` (s).

    A term is switched off by its neutral value: zero sigma, zero resolution, infinite range,
    no time constant. The default is an ideal sensor.
    """

    noise_sigma: float = 0.0
    bias_sigma: float = 0.0
    resolution: float = 0.0
    range_min: float = -math.inf
    range_max: float = math.inf
    lag_tau_s: float | None = None

    def __post_init__(self) -> None:
        for name in ("noise_sigma", "bias_sigma", "resolution"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be a finite number that is not negative, got {value!r}")
        for name in ("range_min", "range_max"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or math.isnan(value):
                raise ValueError(f"{name} must be a number, got {value!r}")
        if not self.range_min < self.range_max:
            raise ValueError("range_min must be below range_max")
        if self.lag_tau_s is not None and not (math.isfinite(self.lag_tau_s) and self.lag_tau_s > 0):
            raise ValueError(f"lag_tau_s must be finite and positive, got {self.lag_tau_s!r}")


IDEAL = ErrorModel()


@dataclass(frozen=True)
class SensorConfig:
    """Sample rate, error model and enabled flag of one sensor.

    A disabled sensor is not sampled and produces no samples. `enabled` defaults to true.
    """

    rate_hz: int
    error: ErrorModel = IDEAL
    enabled: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise ValueError(f"enabled must be true or false, got {self.enabled!r}")


class Channel:
    """One measured quantity with its own bias and its own noise stream (FR-SEN-04).

    `stream_prefix` names the random streams: "{prefix}.bias" and "{prefix}.noise". A scalar
    sensor uses its sensor ID as the prefix ("baro.noise"); a vector sensor uses the sensor ID and
    the channel ("accel.x.noise", "accel.y.noise", "accel.z.noise").

    A lagged channel starts at the first true value it is given, so there is no artificial
    start-up transient.
    """

    def __init__(self, sensor_id: str, channel: str, unit: str, error: ErrorModel, *,
                 master_seed: int, stream_prefix: str, sample_period_s: float) -> None:
        self.sensor_id = sensor_id
        self.channel = channel
        self.unit = unit
        self._error = error
        self.bias = (
            error.bias_sigma * RandomStream(master_seed, f"{stream_prefix}.bias").standard_normal()
            if error.bias_sigma > 0 else 0.0
        )
        """The bias drawn for this run, constant thereafter. Truth-side value: tests only."""
        self._noise = RandomStream(master_seed, f"{stream_prefix}.noise")
        self._lag_gain = (
            1.0 - math.exp(-sample_period_s / error.lag_tau_s) if error.lag_tau_s is not None else None
        )
        self._lagged: float | None = None

    def measure(self, tick: int, true_value: float) -> SensorSample:
        error = self._error
        value = true_value
        if self._lag_gain is not None:                      # 1. dynamics (first-order lag)
            if self._lagged is None:
                self._lagged = true_value                   # starts settled: no start-up transient
            else:
                self._lagged += self._lag_gain * (true_value - self._lagged)
            value = self._lagged
        value += self.bias                                  # 2. bias
        if error.noise_sigma > 0:                           # 3. noise
            value += error.noise_sigma * self._noise.standard_normal()
        saturated = False                                   # 4. saturation
        if value > error.range_max:
            value, saturated = error.range_max, True
        elif value < error.range_min:
            value, saturated = error.range_min, True
        if error.resolution > 0:                            # 5. quantization
            value = round(value / error.resolution) * error.resolution
        return SensorSample(tick, tick_to_seconds(tick), self.sensor_id, self.channel, value, self.unit, saturated)


class Sensor:
    """A sensor with one or more channels, sampled on exact ticks (FR-SEN-03).

    Subclasses name the sensor, its unit, its channels, and how each channel reads truth. A scalar
    sensor has one channel named after its quantity; a vector sensor has channels x, y and z.
    In ideal mode every error term is off, so outputs equal the truth mapping exactly (FR-SEN-06).
    """

    sensor_id: str
    unit: str
    truth_sources: Sequence[tuple[str, Callable[[TruthState], float]]]
    """(channel name, function reading that channel's true quantity from a TruthState)."""

    def __init__(self, config: SensorConfig, *, master_seed: int, ideal: bool = False) -> None:
        self.rate_hz = config.rate_hz
        self.period_ticks = ticks_per_sample(config.rate_hz)
        error = IDEAL if ideal else config.error
        single = len(self.truth_sources) == 1
        self.channels = tuple(
            Channel(
                self.sensor_id, name, self.unit, error, master_seed=master_seed,
                stream_prefix=self.sensor_id if single else f"{self.sensor_id}.{name}",
                sample_period_s=tick_to_seconds(self.period_ticks),
            )
            for name, _ in self.truth_sources
        )

    def is_due(self, tick: int) -> bool:
        return tick % self.period_ticks == 0

    def sample(self, truth: TruthState) -> list[SensorSample]:
        return [
            channel.measure(truth.tick, read(truth))
            for channel, (_, read) in zip(self.channels, self.truth_sources)
        ]
