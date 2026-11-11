"""ARCH-01 and ARCH-02 (TEST_PLAN section 11): static scan of import statements.

The rules are copied by hand from docs/PROJECT_STRUCTURE.md section 3 ("An arrow means
'may import'. Anything not listed is forbidden."). They are the oracle; nothing here is
derived from the code under test.

No aerosat module is executed: source files are parsed with `ast` only.
"""
import ast
import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[2] / "src" / "aerosat"

# PROJECT_STRUCTURE section 3. package -> (allowed aerosat packages, allowed third-party packages)
RULES: dict[str, tuple[frozenset[str], frozenset[str]]] = {
    "core": (frozenset(), frozenset()),
    "sim": (frozenset({"core"}), frozenset()),
    "sensors": (frozenset({"core"}), frozenset()),
    "avionics": (frozenset({"core"}), frozenset()),
    "telemetry": (frozenset({"core"}), frozenset({"pydantic"})),
    "recorder": (frozenset({"core"}), frozenset()),
    "playback": (frozenset({"core", "recorder", "telemetry"}), frozenset()),
    "mission": (frozenset({"core", "sim", "sensors", "avionics", "telemetry", "recorder"}), frozenset()),
    "server": (frozenset({"core", "mission", "telemetry", "recorder", "playback"}), frozenset({"fastapi"})),
    "cli": (frozenset({"core", "mission", "recorder", "telemetry"}), frozenset()),
}

# ARCH-02: "sim, sensors, avionics import only the standard library and aerosat.core; no time,
# datetime, asyncio, socket, file access, or module-level random functions".
PURE_PACKAGES = ("sim", "sensors", "avionics")
FORBIDDEN_STDLIB = frozenset({"time", "datetime", "asyncio", "socket"})
# TEST_PLAN section 3.6: "file access" is the built-in open() and these standard modules.
FILE_ACCESS_MODULES = frozenset({"io", "os", "pathlib", "shutil", "tempfile"})


def imported_names(tree: ast.AST, module: str, is_package: bool) -> set[str]:
    """Return every absolute module name imported anywhere in `tree` (including inside functions)."""
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                parts = module.split(".")
                package = parts if is_package else parts[:-1]
                package = package[: len(package) - (node.level - 1)]
                prefix = ".".join(package)
                if node.module:
                    names.add(f"{prefix}.{node.module}")
                else:
                    names.update(f"{prefix}.{alias.name}" for alias in node.names)
            else:
                assert node.module is not None
                names.add(node.module)
                if node.module == "aerosat":
                    names.update(f"aerosat.{alias.name}" for alias in node.names)
    return names


def import_rule_violations(package: str, names: set[str]) -> list[str]:
    """ARCH-01: imports of `package` that PROJECT_STRUCTURE section 3 does not list."""
    allowed_internal, allowed_third_party = RULES[package]
    problems = []
    for name in sorted(names):
        parts = name.split(".")
        if parts[0] == "aerosat":
            target = parts[1] if len(parts) > 1 else None
            if target is not None and target != package and target not in allowed_internal:
                problems.append(f"{package} may not import aerosat.{target}")
        elif parts[0] not in sys.stdlib_module_names and parts[0] not in allowed_third_party:
            problems.append(f"{package} may not import third-party package {parts[0]}")
    return problems


def purity_violations(tree: ast.AST, names: set[str]) -> list[str]:
    """ARCH-02: forbidden standard-library use in the pure core."""
    problems = []
    for name in sorted(names):
        top = name.split(".")[0]
        if top in FORBIDDEN_STDLIB:
            problems.append(f"imports {top}")
        if top in FILE_ACCESS_MODULES:
            problems.append(f"imports file-access module {top}")
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "open":
            problems.append("calls open()")
        if isinstance(node, ast.ImportFrom) and node.level == 0 and node.module == "random":
            for alias in node.names:
                if alias.name != "Random":
                    problems.append(f"imports module-level random function {alias.name}")
        if (
            isinstance(node, ast.Attribute)
            and isinstance(node.value, ast.Name)
            and node.value.id == "random"
            and node.attr != "Random"
        ):
            problems.append(f"uses module-level random.{node.attr}")
    return problems


def source_files() -> list[tuple[str, str, bool, Path]]:
    """(package, dotted module name, is_package, path) for every module below src/aerosat."""
    found = []
    for path in sorted(SRC.rglob("*.py")):
        relative = path.relative_to(SRC)
        if relative == Path("__init__.py"):
            continue  # the root package has no rule in PROJECT_STRUCTURE section 3
        package = relative.parts[0].removesuffix(".py")
        dotted = ".".join(("aerosat", *relative.with_suffix("").parts))
        is_package = path.name == "__init__.py"
        if is_package:
            dotted = dotted.removesuffix(".__init__")
        found.append((package, dotted, is_package, path))
    return found


def scan(path: Path, dotted: str, is_package: bool) -> tuple[ast.AST, set[str]]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    return tree, imported_names(tree, dotted, is_package)


# --- The checker itself is checked against hand-built cases, so a pass is not vacuous. ---

@pytest.mark.parametrize(
    ("package", "source", "expected"),
    [
        ("avionics", "from aerosat.sim import atmosphere", ["avionics may not import aerosat.sim"]),
        ("avionics", "import aerosat.sensors.suite", ["avionics may not import aerosat.sensors"]),
        ("avionics", "from aerosat.core import types\nimport math", []),
        ("avionics", "from . import estimator\nfrom .calibration import x", []),
        ("avionics", "from ..sim import gravity", ["avionics may not import aerosat.sim"]),
        ("core", "import pydantic", ["core may not import third-party package pydantic"]),
        ("sim", "import numpy as np", ["sim may not import third-party package numpy"]),
        ("telemetry", "import pydantic\nfrom aerosat.core import types", []),
        ("telemetry", "import fastapi", ["telemetry may not import third-party package fastapi"]),
        ("playback", "from aerosat import mission", ["playback may not import aerosat.mission"]),
        ("playback", "from aerosat.recorder import reader\nfrom aerosat.telemetry import messages", []),
        ("mission", "import aerosat.playback.player", ["mission may not import aerosat.playback"]),
        ("recorder", "from aerosat.telemetry import encoder", ["recorder may not import aerosat.telemetry"]),
        ("server", "import fastapi\nfrom aerosat.core import faults\nfrom aerosat.playback import player", []),
        ("server", "from aerosat.sim import simulator", ["server may not import aerosat.sim"]),
        ("cli", "from aerosat.core import faults\nfrom aerosat.mission import runner", []),
        ("cli", "import fastapi", ["cli may not import third-party package fastapi"]),
    ],
)
def test_arch_01_checker_detects_known_cases(package: str, source: str, expected: list[str]) -> None:
    names = imported_names(ast.parse(source), f"aerosat.{package}.module_under_test", is_package=False)
    assert import_rule_violations(package, names) == expected


@pytest.mark.parametrize(
    ("source", "expected"),
    [
        ("import math\nfrom aerosat.core import constants", []),
        ("import random\nrng = random.Random(1)", []),
        ("from random import Random", []),
        ("import time", ["imports time"]),
        ("from datetime import datetime", ["imports datetime"]),
        ("import asyncio", ["imports asyncio"]),
        ("import socket", ["imports socket"]),
        ("from pathlib import Path", ["imports file-access module pathlib"]),
        ("data = open('x').read()", ["calls open()"]),
        ("import random\nx = random.gauss(0, 1)", ["uses module-level random.gauss"]),
        ("from random import gauss", ["imports module-level random function gauss"]),
    ],
)
def test_arch_02_checker_detects_known_cases(source: str, expected: list[str]) -> None:
    tree = ast.parse(source)
    names = imported_names(tree, "aerosat.sim.module_under_test", is_package=False)
    assert purity_violations(tree, names) == expected


# --- The real source tree. ---

def test_arch_01_every_package_has_a_rule() -> None:
    """A package that PROJECT_STRUCTURE section 3 does not list has no permitted imports at all."""
    packages = {package for package, _, _, _ in source_files()}
    assert packages == set(RULES)


def test_arch_01_import_rules() -> None:
    files = source_files()
    assert files, "no source files found"
    problems = []
    for package, dotted, is_package, path in files:
        _, names = scan(path, dotted, is_package)
        problems += [f"{dotted}: {problem}" for problem in import_rule_violations(package, names)]
    assert problems == []


def test_arch_02_pure_core() -> None:
    files = [entry for entry in source_files() if entry[0] in PURE_PACKAGES]
    assert {entry[0] for entry in files} == set(PURE_PACKAGES)
    problems = []
    for package, dotted, is_package, path in files:
        tree, names = scan(path, dotted, is_package)
        for name in sorted(names):
            parts = name.split(".")
            if parts[0] == "aerosat":
                if len(parts) > 1 and parts[1] not in (package, "core"):
                    problems.append(f"{dotted}: imports {name}")
            elif parts[0] not in sys.stdlib_module_names:
                problems.append(f"{dotted}: imports third-party package {parts[0]}")
        problems += [f"{dotted}: {problem}" for problem in purity_violations(tree, names)]
    assert problems == []
