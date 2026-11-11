"""ARCH-03 (TEST_PLAN section 11): declared dependencies are a subset of the allow-list.

The allow-list is copied by hand from docs/ARCHITECTURE.md section 11. It is the oracle.
"""
import json
import re
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

# ARCHITECTURE section 11.
BACKEND_RUNTIME = {"fastapi", "uvicorn", "pydantic"}
BACKEND_DEV = {"pytest", "pytest-cov", "httpx2"}
FRONTEND_RUNTIME = {"react", "react-dom", "three", "uplot"}
FRONTEND_DEV = {
    "typescript",
    "vite",
    "@vitejs/plugin-react",
    "vitest",
    "@types/react",
    "@types/react-dom",
    "@types/three",
}
# ARCHITECTURE section 11, "Explicitly excluded" (the items that are package names).
EXCLUDED = {
    "numpy", "scipy", "pandas", "redis", "celery", "python-socketio", "socket.io", "socket.io-client",
    "redux", "@reduxjs/toolkit", "zustand", "mobx", "@react-three/fiber", "@react-three/drei",
}


def requirement_name(requirement: str) -> str:
    """'uvicorn[standard]>=0.30 ; python_version > "3.11"' -> 'uvicorn' (normalised)."""
    match = re.match(r"\s*([A-Za-z0-9][A-Za-z0-9._-]*)", requirement)
    assert match, f"cannot parse requirement {requirement!r}"
    return re.sub(r"[-_.]+", "-", match.group(1)).lower()


def backend_declared() -> tuple[set[str], set[str]]:
    data = tomllib.loads((ROOT / "backend" / "pyproject.toml").read_text(encoding="utf-8"))
    runtime = {requirement_name(r) for r in data["project"]["dependencies"]}
    dev: set[str] = set()
    for group in data.get("dependency-groups", {}).values():
        dev |= {requirement_name(r) for r in group}
    for extra in data["project"].get("optional-dependencies", {}).values():
        dev |= {requirement_name(r) for r in extra}
    return runtime, dev


def frontend_declared() -> tuple[set[str], set[str]]:
    data = json.loads((ROOT / "frontend" / "package.json").read_text(encoding="utf-8"))
    return set(data.get("dependencies", {})), set(data.get("devDependencies", {}))


def test_requirement_name_parsing() -> None:
    assert requirement_name("uvicorn[standard]") == "uvicorn"
    assert requirement_name("pytest_cov>=7") == "pytest-cov"
    assert requirement_name('httpx2 ; python_version >= "3.11"') == "httpx2"


def test_arch_03_backend_runtime_dependencies_are_allow_listed() -> None:
    runtime, _ = backend_declared()
    assert runtime, "no backend runtime dependencies declared"
    assert runtime - BACKEND_RUNTIME == set()


def test_arch_03_backend_dev_dependencies_are_allow_listed() -> None:
    _, dev = backend_declared()
    assert dev, "no backend development dependencies declared"
    assert dev - BACKEND_DEV == set()


def test_arch_03_frontend_runtime_dependencies_are_allow_listed() -> None:
    runtime, _ = frontend_declared()
    assert runtime, "no frontend runtime dependencies declared"
    assert runtime - FRONTEND_RUNTIME == set()


def test_arch_03_frontend_dev_dependencies_are_allow_listed() -> None:
    _, dev = frontend_declared()
    assert dev, "no frontend development dependencies declared"
    assert dev - FRONTEND_DEV == set()


def test_arch_03_no_explicitly_excluded_package_is_declared() -> None:
    declared = set().union(*backend_declared(), *frontend_declared())
    assert declared & EXCLUDED == set()
