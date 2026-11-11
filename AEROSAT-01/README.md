# AEROSAT-01

A **software-only** virtual rocket avionics and telemetry laboratory for learning how flight
software, sensors, telemetry and ground systems fit together.

> **All data is simulated.** Nothing in this project is measured from a physical instrument.
> Every vehicle, thrust and sensor parameter is a **fictional simulation parameter** chosen for
> convenient arithmetic. None describes a real rocket, motor or sensor. There is no hardware
> interface of any kind.

## Status

Phase 1 (specification and setup) is approved. Phase 2 build step 1 is implemented: the shared
core and the flight simulator (truth physics), tested against closed-form solutions (PHY-01 to
PHY-16, PHY-18, SEN-12). Sensors, avionics, telemetry, recorder, playback, server and the
dashboard are not implemented yet. Test status is listed at the top of `docs/TEST_PLAN.md`.

## Documents

The specification lives in `docs/`:

- `SRS.md`: requirements
- `ARCHITECTURE.md`: subsystems, mathematical model, interfaces, risks
- `TELEMETRY_PROTOCOL.md`: wire protocol
- `TEST_PLAN.md`: planned verification
- `PROJECT_STRUCTURE.md`: layout, dependency rules, conventions

## Setup

### Python versions

The requirement is Python 3.11 or newer (SRS NFR-08). What has actually been checked, on
2026-11-11, Windows 11 x86-64, with no C compiler installed:

| Python | Dependency install | Result |
|---|---|---|
| 3.11.18 | succeeds | Verified |
| 3.12.16 | succeeds | Verified |
| 3.13.18 | succeeds | Verified |
| 3.14.10 | succeeds | Verified; used for `backend/.venv` |
| 3.15.2 | fails | Not supported on this setup |

"Verified" means: the install succeeds, `pip check` reports no broken requirements, all stub
modules import, the FastAPI test client (on `httpx2`) makes a REST call and a WebSocket
connection with no warnings, and a real Uvicorn server completes a WebSocket handshake on
127.0.0.1. No project test existed when this check was run, so nothing more than that is claimed. The five-version
check was run with `pip` before the project moved to `uv`; the current `uv.lock` environment has
been checked on 3.14.10 only.

Python 3.15.2 fails because `uvicorn[standard]` requires PyYAML, PyYAML 6.0.3 has no prebuilt
wheel for Python 3.15 on Windows, and building it from source needs Microsoft Visual C++.
Every other dependency installs and passes the same checks on 3.15.2; only PyYAML blocks it.

Not checked: macOS, Linux, 32-bit or ARM Windows, and Python 3.15 on a machine with a C compiler.

### Backend

Dependencies are managed with [uv](https://docs.astral.sh/uv/). `backend/pyproject.toml` declares
them, `backend/uv.lock` pins exact versions and is committed, and `backend/.python-version` selects
Python 3.14 (uv downloads it if it is missing).

Install or synchronize the environment (creates `backend/.venv`, installs exactly what the lock
file says, and removes anything else):

```
cd backend
uv sync
```

Run the tests:

```
uv run pytest -m "not slow and not perf"     # fast tier
uv run pytest -m "slow or perf"              # slow tier
uv run pytest --cov                          # with a coverage report
```

The slow tier has no tests yet, so that command exits with code 5 ("no tests ran"). That is an
empty run, not a pass.

Dependency maintenance:

| Purpose | Command |
|---|---|
| Check that `uv.lock` matches `pyproject.toml` | `uv lock --check` |
| Check that the environment matches the lock file | `uv sync --locked --check` |
| Re-lock after editing `pyproject.toml` | `uv lock`, then `uv sync` |
| Upgrade pinned versions deliberately | `uv lock --upgrade`, then `uv sync` |
| Show the dependency tree | `uv tree` |

Commit `uv.lock` together with any change to `pyproject.toml`.

### Frontend

Node.js (checked on 26.13.0).

```
cd frontend
npm install
npm test
npm run dev
```
