# AEROSAT-01 — Project Structure

| | |
|---|---|
| Document | PROJECT_STRUCTURE.md |
| Version | 0.1 — DRAFT FOR REVIEW |
| Date | 2026-11-10 |

> This describes the **planned** layout. Nothing below exists yet except the `docs/` folder.

---

## 1. Repository layout

```
aerosat-01/
├── README.md                      What this is; "software-only, simulated data" notice
├── docs/
│   ├── SRS.md
│   ├── ARCHITECTURE.md
│   ├── TELEMETRY_PROTOCOL.md
│   ├── TEST_PLAN.md
│   └── PROJECT_STRUCTURE.md
│
├── scenarios/                     Reproducible run definitions (all "fictional": true)
│   ├── ref-01.json                Reference flight
│   └── verification/              Vacuum, constant-density and other analytic cases
│
├── schemas/                       Generated from backend message models; committed
│   ├── telemetry.schema.json
│   ├── event.schema.json
│   ├── hello.schema.json
│   ├── status.schema.json
│   ├── heartbeat.schema.json
│   ├── error.schema.json
│   └── scenario.schema.json
│
├── backend/
│   ├── pyproject.toml             Dependencies (allow-list) and tool configuration
│   ├── src/aerosat/
│   │   ├── core/                  Shared foundations (standard library only)
│   │   │   ├── constants.py       g0, r0, T0, p0, L, R, gamma: the only place they appear
│   │   │   ├── clock.py           Integer tick clock; tick <-> seconds
│   │   │   ├── rng.py             Seed derivation, per-stream generators, Box-Muller
│   │   │   ├── types.py           TruthState, SensorSample, AvionicsEstimate,
│   │   │   │                      FlightEvent, VehicleCommand, DownlinkMessage
│   │   │   └── faults.py          MODEL_RANGE, VALIDITY, NON_FINITE_VALUE, ...
│   │   │
│   │   ├── sim/                   1. Flight simulator (truth)
│   │   │   ├── atmosphere.py      Geopotential altitude, T, p, rho, speed of sound
│   │   │   ├── gravity.py         g(h)
│   │   │   ├── vehicle.py         Thrust table, impulse, mass, drag area
│   │   │   ├── dynamics.py        Derivative function a(t, h, v); specific force
│   │   │   ├── integrators.py     RK4, Euler
│   │   │   └── simulator.py       Stepping, pad hold, touchdown, commands, truth events
│   │   │
│   │   ├── sensors/               2. Synthetic sensor subsystem
│   │   │   ├── pipeline.py        lag -> bias -> noise -> saturate -> quantize
│   │   │   ├── barometer.py
│   │   │   ├── accelerometer.py
│   │   │   ├── gyroscope.py
│   │   │   ├── thermometer.py
│   │   │   └── suite.py           Builds sensors from a scenario; due-tick scheduling
│   │   │
│   │   ├── avionics/              3. Virtual avionics computer
│   │   │   ├── calibration.py     Pad reference pressure, temperature, accel bias
│   │   │   ├── estimator.py       Barometric altitude; complementary filter
│   │   │   ├── state_machine.py   Flight states, debounced event detection
│   │   │   └── flight_computer.py Ties the above together; frames and commands
│   │   │
│   │   ├── telemetry/             4. Telemetry infrastructure (transport-independent part)
│   │   │   ├── messages.py        Message models; JSON Schema export
│   │   │   ├── encoder.py         Sequencing, validation, strict JSON serialization
│   │   │   ├── link.py            Optional seeded impairment model
│   │   │   └── hub.py             Subscriber queues, fan-out, slow-client policy
│   │   │
│   │   ├── recorder/              5a. Flight data recorder
│   │   │   ├── writer.py          Append-only CSV / JSON Lines writers
│   │   │   ├── manifest.py        Manifest, checksums, finalize
│   │   │   └── reader.py          Load and verify a flight record
│   │   │
│   │   ├── playback/              5b. Playback
│   │   │   └── player.py          Recorded messages -> paced message stream
│   │   │
│   │   ├── mission/               Composition root
│   │   │   ├── scenario.py        Load and validate scenario files
│   │   │   ├── runner.py          Tick scheduler wiring sim, sensors, avionics, recorder
│   │   │   ├── pacing.py          Batch and paced execution; injectable clock
│   │   │   └── lifecycle.py       Mission status state machine
│   │   │
│   │   ├── server/                4. Telemetry infrastructure (network part)
│   │   │   ├── app.py             FastAPI application factory
│   │   │   ├── routes_missions.py
│   │   │   ├── routes_flights.py
│   │   │   ├── routes_playbacks.py
│   │   │   └── ws.py              WebSocket endpoint
│   │   │
│   │   └── cli.py                 Headless batch run; schema export; golden-file generation
│   │
│   └── tests/
│       ├── unit/
│       │   ├── core/              PHY-14, SEN-12
│       │   ├── mission/           PHY-17
│       │   ├── sim/               PHY-*
│       │   ├── sensors/           SEN-*
│       │   ├── avionics/          EVT-*
│       │   ├── telemetry/         TLM-01...08
│       │   ├── recorder/          REC-*
│       │   └── playback/          PBK-*
│       ├── integration/           Full runs, WebSocket, REST (TLM-09...14)
│       ├── architecture/          ARCH-*
│       ├── perf/                  PERF-* (slow tier)
│       └── golden/                Frozen reference outputs + generation metadata
│
├── frontend/                      6. Ground station
│   ├── package.json
│   ├── tsconfig.json              strict mode
│   ├── vite.config.ts
│   ├── index.html
│   └── src/
│       ├── protocol/              Message types and runtime validators
│       ├── net/                   WebSocket client (reconnect), REST client
│       ├── state/                 Pure store: reducer, selectors, formatting, link stats
│       ├── scene/                 Three.js scene class and pure mapping functions
│       ├── components/            React panels, charts, controls, SIMULATED banner
│       ├── App.tsx
│       ├── main.tsx
│       └── __tests__/             DSH-* (fixtures copied from backend golden files)
│
└── data/                          Not committed
    └── flights/{mission_id}/      One folder per recorded mission
```

## 2. Subsystem-to-folder map

| Subsystem (ARCHITECTURE §2) | Code location |
|---|---|
| 1. Flight simulator | `backend/src/aerosat/sim/` |
| 2. Synthetic sensor subsystem | `backend/src/aerosat/sensors/` |
| 3. Virtual avionics computer | `backend/src/aerosat/avionics/` |
| 4. Telemetry infrastructure | `backend/src/aerosat/telemetry/`, `backend/src/aerosat/server/` |
| 5. Flight data recorder and playback | `backend/src/aerosat/recorder/`, `backend/src/aerosat/playback/` |
| 6. Ground station | `frontend/` |

## 3. Dependency rules

An arrow means "may import". Anything not listed is forbidden. Enforced by test ARCH-01.

```
core        -> (standard library only)
sim         -> core
sensors     -> core
avionics    -> core
telemetry   -> core, pydantic
recorder    -> core
playback    -> core, recorder, telemetry
mission     -> core, sim, sensors, avionics, telemetry, recorder
server      -> core, mission, telemetry, recorder, playback, fastapi
cli         -> core, mission, recorder, telemetry
```

Key consequences:

- `avionics` cannot import `sim` or `sensors`: it can only receive `SensorSample` values handed to it. This is what keeps truth out of the flight computer.
- `sim`, `sensors`, `avionics` have no third-party imports and no I/O, so they run and test anywhere.
- `playback` cannot reach `sim`, `sensors`, `avionics` or `mission`.
- Only `server` knows FastAPI. Only `mission.pacing` and `server` know the wall clock. The recorder never reads a clock: `mission` passes it the wall-clock timestamps for the manifest and triggers its periodic flush.
- `recorder` does not serialize messages: it is handed the line already serialized by `telemetry.encoder`.

Frontend:

```
protocol    -> (nothing)
net         -> protocol
state       -> protocol
scene       -> state (types only), three
components  -> protocol (types only), state, scene, net, react, uplot
App, main   -> components, state, net, react
```

`state` and `scene` never import React. `scene` never imports `net`.

## 4. Naming and unit conventions

| Rule | Example |
|---|---|
| Physical quantities carry a unit suffix everywhere: variables, JSON keys, CSV columns | `altitude_m`, `thrust_n`, `density_kgpm3` |
| Allowed suffixes | `_m`, `_mps`, `_mps2`, `_s`, `_ms`, `_kg`, `_n`, `_ns` (N·s), `_pa`, `_k`, `_kgpm3`, `_m2`, `_rad`, `_radps`, `_hz` |
| Altitude always states its reference | `altitude_msl_m`, `altitude_agl_m` (telemetry `altitude_m` is defined as AGL in the protocol) |
| Truth, measurement and estimate never share a name | `TruthState.velocity_mps` vs `AvionicsEstimate.velocity_mps`: different types, different files |
| Dimensionless values say so in a docstring | `mach`, `drag_coefficient` |
| Enumerations are upper snake case | `PAD_IDLE`, `DEPLOY_DRAG_DEVICE` |
| Integer time in scenarios is milliseconds | `t_ms`, `t_ignition_ms` |
| Python: `snake_case` modules and functions, `PascalCase` classes. TypeScript: `camelCase` values, `PascalCase` types and components. Wire field names stay `snake_case` on both sides. | |

Unit conversions for display (K → °C, m/s² → g, rad/s → deg/s) happen only in `frontend/src/state/` formatting functions.

## 5. Scenario file format

`scenarios/ref-01.json` (all values fictional; see ARCHITECTURE §4.11 and §5.2):

```json
{
  "scenario_id": "REF-01",
  "schema_version": "1.0",
  "fictional": true,
  "description": "Reference vertical flight. Fictional parameters for education only.",
  "seed": 42,
  "timing": {
    "physics_rate_hz": 1000,
    "integrator": "rk4",
    "t_ignition_ms": 3000,
    "post_touchdown_hold_ms": 5000,
    "max_duration_ms": 600000,
    "truth_log_rate_hz": 200
  },
  "environment": {
    "pad_altitude_msl_m": 0.0,
    "atmosphere_model": "standard",
    "gravity_model": "inverse_square"
  },
  "vehicle": {
    "dry_mass_kg": 15.0,
    "propellant_mass_kg": 5.0,
    "drag_coefficient": 0.5,
    "reference_area_m2": 0.01,
    "thrust_curve": [
      { "t_ms": 0,    "thrust_n": 0.0 },
      { "t_ms": 100,  "thrust_n": 1500.0 },
      { "t_ms": 3000, "thrust_n": 1500.0 },
      { "t_ms": 3200, "thrust_n": 0.0 }
    ],
    "drag_device": { "enabled": true, "cd_area_m2": 1.5, "deploy_delay_after_apogee_ms": 1000 },
    "roll_profile": [
      { "t_ms": 0,    "roll_rate_radps": 0.0 },
      { "t_ms": 3000, "roll_rate_radps": 0.0 },
      { "t_ms": 6200, "roll_rate_radps": 0.5 }
    ]
  },
  "sensors": {
    "ideal": false,
    "baro":  { "rate_hz": 50,  "noise_sigma_pa": 3.0,      "bias_sigma_pa": 30.0,     "resolution_pa": 1.0,      "range_pa": [1000.0, 110000.0] },
    "accel": { "rate_hz": 200, "noise_sigma_mps2": 0.05,   "bias_sigma_mps2": 0.10,   "resolution_mps2": 0.005,  "range_mps2": [-160.0, 160.0] },
    "gyro":  { "rate_hz": 200, "noise_sigma_radps": 0.002, "bias_sigma_radps": 0.005, "resolution_radps": 0.001, "range_radps": [-35.0, 35.0] },
    "temp":  { "rate_hz": 10,  "noise_sigma_k": 0.05,      "bias_sigma_k": 0.5,       "resolution_k": 0.01,      "range_k": [200.0, 330.0], "lag_tau_s": 2.0 }
  },
  "avionics": {
    "rate_hz": 200,
    "telemetry_rate_hz": 20,
    "calibration_duration_ms": 1000,
    "liftoff_specific_force_mps2": 15.0,
    "debounce_samples": 10,
    "apogee_lockout_after_burnout_ms": 1000,
    "touchdown_window_ms": 2000,
    "touchdown_altitude_range_m": 3.0,
    "touchdown_speed_mps": 2.0
  },
  "link": { "profile": "ideal" }
}
```

Estimator gains are added to `avionics` in Phase 2 once chosen by analysis. The roll profile is linearly interpolated between points and held after the last one.

Time references: `thrust_curve` `t_ms` is time since ignition. `roll_profile` `t_ms` is time since scenario start (in REF-01, 3000 is ignition and 6200 is the end of the thrust table).

Allowed values:

| Field | Values |
|---|---|
| `timing.integrator` | `"rk4"` (default), `"euler"` (FR-SIM-04) |
| `environment.atmosphere_model` | `"standard"`, `"constant_density"`, `"vacuum"` (FR-SIM-10) |
| `environment.gravity_model` | `"inverse_square"`, `"constant"` (constant = g₀) (FR-SIM-10) |
| `environment.air_density_kgpm3` | Number > 0, in kg/m³. Required when `atmosphere_model` is `"constant_density"`; not allowed otherwise. No default. REF-01 does not contain it. The constant-density verification scenario sets 1.225. |

## 6. Flight record layout

```
data/flights/AS01-7F3A9C2E/
├── manifest.json        metadata, embedded scenario, units, categories, checksums, status
├── truth.csv            TRUTH        tick, t_sim_s, altitude_msl_m, altitude_agl_m, velocity_mps,
│                                     acceleration_mps2, specific_force_mps2, mass_kg, thrust_n,
│                                     drag_n, pressure_pa, temperature_k, density_kgpm3, mach,
│                                     roll_rate_radps, roll_angle_rad, on_ground
├── truth_events.json    TRUTH        true liftoff, burnout, apogee, touchdown times; impact velocity
├── sensors.csv          MEASUREMENT  tick, t_sim_s, sensor_id, channel, value, unit, saturated
├── avionics.csv         ESTIMATE     tick, t_sim_s, altitude_agl_m, velocity_mps, acceleration_mps2,
│                                     baro_altitude_agl_m, flight_state
└── telemetry.jsonl      TELEMETRY    one vehicle-originated message per line, in seq order
```

## 7. Planned commands (to be confirmed in Phase 2)

| Purpose | Command |
|---|---|
| Headless run | `python -m aerosat.cli run scenarios/ref-01.json` |
| Export schemas | `python -m aerosat.cli export-schemas` |
| Backend tests, fast tier | `pytest -m "not slow and not perf"` |
| Backend tests, slow tier | `pytest -m "slow or perf"` |
| Start server | `uvicorn aerosat.server.app:create_app --factory` |
| Frontend dev server | `npm run dev` |
| Frontend tests | `npm test` |

## 8. Proposed Phase 2 build order

Each step is finished when its tests pass. Later steps do not start on top of failing earlier ones.

**Must do**

1. `core` + `sim` → PHY tests against closed-form oracles.
2. `sensors` → SEN tests.
3. `avionics` → EVT tests, including multi-seed.
4. `recorder` + `mission.runner` + CLI → REC tests, determinism, first golden files.
5. `telemetry` + `server` → TLM tests.
6. `playback` → PBK tests.
7. Frontend `protocol` + `state` → DSH tests using backend golden files.
8. Frontend 2D dashboard (readouts, charts, link health, controls).
9. Frontend 3D view.

**Nice to have (only after the above)**

- Link-impairment profiles beyond `ideal` (if D-4 is approved, the model itself is in step 5).
- Mid-flight history backfill for late-joining clients.
- Browser end-to-end smoke test.
- Truth-versus-estimate comparison view, clearly labelled as an instructor/debug tool.
