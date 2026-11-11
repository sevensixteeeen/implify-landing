"""Shared fixtures for the physics (PHY) tests.

The analytic configuration is TEST_PLAN section 4: m = 15 kg, C_d*A = 0.005 m^2,
rho = 1.225 kg/m^3 constant, g = 9.80665 m/s^2 constant, dt = 1 ms.
"""
import json
import math
from pathlib import Path

import pytest

from aerosat.sim.simulator import DragDevice, SimulatorConfig
from aerosat.sim.vehicle import RollProfile, ThrustCurve, Vehicle

ROOT = Path(__file__).resolve().parents[4]

MASS_KG = 15.0
CD_AREA_M2 = 0.005
DENSITY_KGPM3 = 1.225
G_MPS2 = 9.80665


@pytest.fixture(scope="session")
def terminal_velocity_mps() -> float:
    """v_t = sqrt(2*m*g / (rho*C_d*A)). TEST_PLAN section 4 states 219.1633 m/s."""
    v_t = math.sqrt(2 * MASS_KG * G_MPS2 / (DENSITY_KGPM3 * CD_AREA_M2))
    assert v_t == pytest.approx(219.1633, abs=5e-5)
    return v_t


@pytest.fixture
def unpowered_vehicle() -> Vehicle:
    """15 kg, C_d*A = 0.005 m^2, no thrust."""
    return Vehicle(dry_mass_kg=MASS_KG, propellant_mass_kg=0.0, cd_area_m2=CD_AREA_M2)


@pytest.fixture(scope="session")
def ref01() -> dict:
    return json.loads((ROOT / "scenarios" / "ref-01.json").read_text(encoding="utf-8"))


@pytest.fixture
def ref01_config(ref01: dict) -> SimulatorConfig:
    """REF-01 as a simulator configuration (standard atmosphere, inverse-square gravity)."""
    vehicle = ref01["vehicle"]
    return SimulatorConfig(
        vehicle=Vehicle(
            dry_mass_kg=vehicle["dry_mass_kg"],
            propellant_mass_kg=vehicle["propellant_mass_kg"],
            cd_area_m2=vehicle["drag_coefficient"] * vehicle["reference_area_m2"],
            thrust_curve=ThrustCurve([(p["t_ms"], p["thrust_n"]) for p in vehicle["thrust_curve"]]),
        ),
        t_ignition_ms=ref01["timing"]["t_ignition_ms"],
        pad_altitude_msl_m=ref01["environment"]["pad_altitude_msl_m"],
        drag_device=DragDevice(
            enabled=vehicle["drag_device"]["enabled"], cd_area_m2=vehicle["drag_device"]["cd_area_m2"]
        ),
        roll_profile=RollProfile([(p["t_ms"], p["roll_rate_radps"]) for p in vehicle["roll_profile"]]),
        post_touchdown_hold_ms=ref01["timing"]["post_touchdown_hold_ms"],
        max_duration_ms=ref01["timing"]["max_duration_ms"],
    )
