"""Builds sensors from a scenario; due-tick scheduling."""
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from aerosat.core.types import SensorSample, TruthState
from aerosat.sensors.accelerometer import Accelerometer
from aerosat.sensors.barometer import Barometer
from aerosat.sensors.gyroscope import Gyroscope
from aerosat.sensors.pipeline import ErrorModel, Sensor, SensorConfig
from aerosat.sensors.thermometer import Thermometer

# Scenario key for each sensor, and the unit suffix its parameters carry (PROJECT_STRUCTURE section 5).
_SCENARIO_SENSORS = (("baro", "pa"), ("accel", "mps2"), ("gyro", "radps"), ("temp", "k"))
_SENSOR_CLASSES = {"baro": Barometer, "accel": Accelerometer, "gyro": Gyroscope, "temp": Thermometer}


@dataclass(frozen=True)
class SensorSuiteConfig:
    """The four sensors. A sensor whose configuration has `enabled = False` produces no samples."""

    ideal: bool
    baro: SensorConfig
    accel: SensorConfig
    gyro: SensorConfig
    temp: SensorConfig


def suite_config_from_scenario(sensors: Mapping[str, Any]) -> SensorSuiteConfig:
    """Read the `sensors` section of a scenario. Raises ValueError naming the offending field."""
    known = {"ideal"} | {name for name, _ in _SCENARIO_SENSORS}
    unknown = set(sensors) - known
    if unknown:
        raise ValueError(f"sensors: unknown field(s) {sorted(unknown)}")
    if not isinstance(sensors.get("ideal"), bool):
        raise ValueError("sensors.ideal: must be true or false")

    configs: dict[str, SensorConfig] = {}
    for name, suffix in _SCENARIO_SENSORS:
        section = sensors.get(name)
        if not isinstance(section, Mapping):
            raise ValueError(f"sensors.{name}: missing sensor section")
        expected = {"rate_hz", f"noise_sigma_{suffix}", f"bias_sigma_{suffix}", f"resolution_{suffix}", f"range_{suffix}"}
        optional = {"enabled", "lag_tau_s"} if name == "temp" else {"enabled"}
        if not isinstance(section.get("enabled", True), bool):
            raise ValueError(f"sensors.{name}.enabled: must be true or false, got {section['enabled']!r}")
        missing, extra = expected - set(section), set(section) - expected - optional
        if missing:
            raise ValueError(f"sensors.{name}: missing field(s) {sorted(missing)}")
        if extra:
            raise ValueError(f"sensors.{name}: unknown field(s) {sorted(extra)}")
        value_range = section[f"range_{suffix}"]
        if not isinstance(value_range, (list, tuple)) or len(value_range) != 2:
            raise ValueError(f"sensors.{name}.range_{suffix}: must be [minimum, maximum]")
        try:
            configs[name] = SensorConfig(
                rate_hz=section["rate_hz"],
                error=ErrorModel(
                    noise_sigma=section[f"noise_sigma_{suffix}"],
                    bias_sigma=section[f"bias_sigma_{suffix}"],
                    resolution=section[f"resolution_{suffix}"],
                    range_min=value_range[0],
                    range_max=value_range[1],
                    lag_tau_s=section.get("lag_tau_s"),
                ),
                enabled=section.get("enabled", True),
            )
        except (ValueError, TypeError) as problem:
            raise ValueError(f"sensors.{name}: {problem}") from None
    return SensorSuiteConfig(ideal=sensors["ideal"], **configs)


class SensorSuite:
    """The enabled sensors. Converts a truth state into the samples due at its tick."""

    def __init__(self, config: SensorSuiteConfig, master_seed: int) -> None:
        self._enabled = {name: getattr(config, name).enabled for name, _ in _SCENARIO_SENSORS}
        self.sensors: tuple[Sensor, ...] = tuple(
            _SENSOR_CLASSES[name](getattr(config, name), master_seed=master_seed, ideal=config.ideal)
            for name, enabled in self._enabled.items()
            if enabled
        )

    def sensor_status(self) -> dict[str, bool]:
        """Sensor ID -> enabled, for all four sensors. Recorded in the mission metadata (FR-FDR-02)."""
        return dict(self._enabled)

    def sample(self, truth: TruthState) -> list[SensorSample]:
        """Samples from every sensor whose period divides the truth state's tick."""
        samples: list[SensorSample] = []
        for sensor in self.sensors:
            if sensor.is_due(truth.tick):
                samples.extend(sensor.sample(truth))
        return samples
