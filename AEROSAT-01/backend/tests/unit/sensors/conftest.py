"""Shared fixtures for the sensor (SEN) tests."""
import json
from pathlib import Path

import pytest

from aerosat.core.types import TruthState
from aerosat.sensors.suite import SensorSuiteConfig, suite_config_from_scenario

ROOT = Path(__file__).resolve().parents[4]


def truth(tick: int = 0, **overrides: float) -> TruthState:
    """A hand-built truth state. Defaults: at rest on a sea-level pad."""
    values = dict(
        tick=tick, t_sim_s=tick / 1000, altitude_msl_m=0.0, altitude_agl_m=0.0, velocity_mps=0.0,
        acceleration_mps2=0.0, specific_force_mps2=9.80665, mass_kg=20.0, thrust_n=0.0, drag_n=0.0,
        pressure_pa=101_325.0, temperature_k=288.15, density_kgpm3=1.225, mach=0.0,
        roll_rate_radps=0.0, roll_angle_rad=0.0, on_ground=True,
    )
    values.update(overrides)
    return TruthState(**values)


@pytest.fixture(scope="session")
def ref01_sensors() -> dict:
    """The `sensors` section of REF-01."""
    return json.loads((ROOT / "scenarios" / "ref-01.json").read_text(encoding="utf-8"))["sensors"]


@pytest.fixture
def default_config(ref01_sensors: dict) -> SensorSuiteConfig:
    """Default (REF-01) sensor configuration, with noise, bias, quantization and ranges."""
    return suite_config_from_scenario(ref01_sensors)
