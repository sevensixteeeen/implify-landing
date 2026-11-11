"""SET-01 and SET-02 (TEST_PLAN section 11.1): Phase 1 project layout and reference scenario file.

Expected values are copied by hand from the specification documents, which are the oracle:
docs/PROJECT_STRUCTURE.md section 1 (layout) and section 5 (scenario format),
docs/ARCHITECTURE.md sections 4.11 and 5.2, docs/SRS.md FR-SCN-03, FR-SCN-05 and PR-01.

These tests read files only. They run no simulation code.
"""
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "backend" / "src" / "aerosat"

# PROJECT_STRUCTURE section 1.
MODULES = {
    "core": {"constants", "clock", "rng", "types", "faults"},
    "sim": {"atmosphere", "gravity", "vehicle", "dynamics", "integrators", "simulator"},
    "sensors": {"pipeline", "barometer", "accelerometer", "gyroscope", "thermometer", "suite"},
    "avionics": {"calibration", "estimator", "state_machine", "flight_computer"},
    "telemetry": {"messages", "encoder", "link", "hub"},
    "recorder": {"writer", "manifest", "reader"},
    "playback": {"player"},
    "mission": {"scenario", "runner", "pacing", "lifecycle"},
    "server": {"app", "routes_missions", "routes_flights", "routes_playbacks", "ws"},
}
TEST_FOLDERS = [
    "unit/core", "unit/mission", "unit/sim", "unit/sensors", "unit/avionics", "unit/telemetry",
    "unit/recorder", "unit/playback", "integration", "architecture", "perf", "golden",
]
FRONTEND_FOLDERS = ["protocol", "net", "state", "scene", "components", "__tests__"]
FRONTEND_FILES = ["package.json", "package-lock.json", "tsconfig.json", "vite.config.ts", "index.html",
                  "src/App.tsx", "src/main.tsx"]
DOCUMENTS = ["SRS.md", "ARCHITECTURE.md", "TELEMETRY_PROTOCOL.md", "TEST_PLAN.md", "PROJECT_STRUCTURE.md"]


# ---------------------------------------------------------------- SET-01: layout

@pytest.mark.parametrize("package", sorted(MODULES))
def test_set_01_backend_package_contains_exactly_the_specified_modules(package: str) -> None:
    folder = SRC / package
    assert (folder / "__init__.py").is_file()
    present = {path.stem for path in folder.glob("*.py")} - {"__init__"}
    assert present == MODULES[package]


def test_set_01_backend_has_no_unspecified_packages() -> None:
    packages = {path.name for path in SRC.iterdir() if path.is_dir() and path.name != "__pycache__"}
    assert packages == set(MODULES)
    assert (SRC / "cli.py").is_file()
    assert {path.name for path in SRC.glob("*.py")} == {"__init__.py", "cli.py"}


def test_set_01_backend_configuration_files_exist() -> None:
    for name in ("pyproject.toml", "uv.lock", ".python-version"):
        assert (ROOT / "backend" / name).is_file(), name


@pytest.mark.parametrize("folder", TEST_FOLDERS)
def test_set_01_test_folder_exists(folder: str) -> None:
    assert (ROOT / "backend" / "tests" / folder).is_dir()


def test_set_01_frontend_layout() -> None:
    for folder in FRONTEND_FOLDERS:
        assert (ROOT / "frontend" / "src" / folder).is_dir(), folder
    for name in FRONTEND_FILES:
        assert (ROOT / "frontend" / name).is_file(), name


def test_set_01_documents_and_top_level_layout() -> None:
    for name in DOCUMENTS:
        assert (ROOT / "docs" / name).is_file(), name
    assert (ROOT / "README.md").is_file()
    assert (ROOT / "schemas").is_dir()
    assert (ROOT / "scenarios" / "verification").is_dir()
    assert (ROOT / "scenarios" / "ref-01.json").is_file()


# ---------------------------------------------------------------- SET-02: reference scenario file

@pytest.fixture(scope="module")
def ref01() -> dict:
    return json.loads((ROOT / "scenarios" / "ref-01.json").read_text(encoding="utf-8"))


def test_set_02_identity_and_fictional_flag(ref01: dict) -> None:
    assert ref01["scenario_id"] == "REF-01"
    assert ref01["schema_version"] == "1.0"
    assert ref01["fictional"] is True  # FR-SCN-03
    assert ref01["seed"] == 42  # ARCHITECTURE 4.11


def test_set_02_vehicle_matches_architecture_4_11(ref01: dict) -> None:
    vehicle = ref01["vehicle"]
    assert vehicle["dry_mass_kg"] == 15.0
    assert vehicle["propellant_mass_kg"] == 5.0
    assert vehicle["drag_coefficient"] == 0.5
    assert vehicle["reference_area_m2"] == 0.010
    assert vehicle["drag_coefficient"] * vehicle["reference_area_m2"] == pytest.approx(0.005, rel=1e-12)
    assert [(p["t_ms"], p["thrust_n"]) for p in vehicle["thrust_curve"]] == [
        (0, 0.0), (100, 1500.0), (3000, 1500.0), (3200, 0.0),
    ]
    assert vehicle["drag_device"] == {"enabled": True, "cd_area_m2": 1.5, "deploy_delay_after_apogee_ms": 1000}
    assert ref01["environment"]["pad_altitude_msl_m"] == 0.0
    assert ref01["timing"]["t_ignition_ms"] == 3000


def test_set_02_total_impulse_is_4575_newton_seconds(ref01: dict) -> None:
    """ARCHITECTURE 4.11: 75 + 4350 + 150 = 4575 N*s by trapezoids. Arithmetic on the file only."""
    points = [(p["t_ms"], p["thrust_n"]) for p in ref01["vehicle"]["thrust_curve"]]
    impulse_n_ms = sum((f0 + f1) / 2 * (t1 - t0) for (t0, f0), (t1, f1) in zip(points, points[1:]))
    assert impulse_n_ms == 4575.0 * 1000


def test_set_02_rates_match_pr_01_and_divide_the_physics_rate(ref01: dict) -> None:
    physics = ref01["timing"]["physics_rate_hz"]
    assert physics == 1000
    rates = {
        "baro": ref01["sensors"]["baro"]["rate_hz"],
        "accel": ref01["sensors"]["accel"]["rate_hz"],
        "gyro": ref01["sensors"]["gyro"]["rate_hz"],
        "temp": ref01["sensors"]["temp"]["rate_hz"],
        "avionics": ref01["avionics"]["rate_hz"],
        "telemetry": ref01["avionics"]["telemetry_rate_hz"],
        "truth_log": ref01["timing"]["truth_log_rate_hz"],
    }
    assert rates == {"baro": 50, "accel": 200, "gyro": 200, "temp": 10, "avionics": 200, "telemetry": 20,
                     "truth_log": 200}
    for name, rate in rates.items():  # FR-SCN-05
        assert physics % rate == 0, name


def test_set_02_timing_satisfies_fr_scn_05(ref01: dict) -> None:
    timing = ref01["timing"]
    assert timing["t_ignition_ms"] >= 2000
    assert ref01["avionics"]["calibration_duration_ms"] == 1000
    assert timing["post_touchdown_hold_ms"] == 5000
    assert all(isinstance(p["t_ms"], int) for p in ref01["vehicle"]["thrust_curve"])
    assert ref01["vehicle"]["dry_mass_kg"] > 0 and ref01["vehicle"]["propellant_mass_kg"] > 0


def test_set_02_sensor_defaults_match_architecture_5_2(ref01: dict) -> None:
    sensors = ref01["sensors"]
    assert sensors["ideal"] is False
    assert sensors["baro"] == {"rate_hz": 50, "noise_sigma_pa": 3.0, "bias_sigma_pa": 30.0,
                               "resolution_pa": 1.0, "range_pa": [1000.0, 110000.0]}
    assert sensors["accel"] == {"rate_hz": 200, "noise_sigma_mps2": 0.05, "bias_sigma_mps2": 0.10,
                                "resolution_mps2": 0.005, "range_mps2": [-160.0, 160.0]}
    assert sensors["gyro"] == {"rate_hz": 200, "noise_sigma_radps": 0.002, "bias_sigma_radps": 0.005,
                               "resolution_radps": 0.001, "range_radps": [-35.0, 35.0]}
    assert sensors["temp"] == {"rate_hz": 10, "noise_sigma_k": 0.05, "bias_sigma_k": 0.5,
                               "resolution_k": 0.01, "range_k": [200.0, 330.0], "lag_tau_s": 2.0}


def test_set_02_models_and_no_density_parameter(ref01: dict) -> None:
    """REF-01 uses the standard atmosphere, so it must not carry the constant-density parameter."""
    assert ref01["timing"]["integrator"] == "rk4"
    assert ref01["environment"]["atmosphere_model"] == "standard"
    assert ref01["environment"]["gravity_model"] == "inverse_square"
    assert "air_density_kgpm3" not in ref01["environment"]
    assert ref01["link"] == {"profile": "ideal"}
