# AEROSAT-01 — Software Requirements Specification (SRS)

| | |
|---|---|
| Document | SRS.md |
| Version | 0.1 — APPROVED 2026-11-11 (Phase 1) |
| Date | 2026-11-10 |
| Phase | 1 (requirements and architecture) approved 2026-11-11. Phase 2 (implementation) in progress; status is tracked in PROJECT_STRUCTURE.md and TEST_PLAN.md. |
| Related | ARCHITECTURE.md, TELEMETRY_PROTOCOL.md, TEST_PLAN.md, PROJECT_STRUCTURE.md |

---

## 1. Purpose and scope

AEROSAT-01 is a **software-only virtual rocket avionics and telemetry laboratory** for learning how flight software, sensors, telemetry and ground systems fit together.

**In scope**

- A simplified one-dimensional (vertical) flight simulation.
- Simulated sensors, a virtual avionics computer, a telemetry link, a flight data recorder, playback, and a browser dashboard with a 3D view.

**Out of scope (will not be built)**

- Any physical vehicle, propulsion, ignition, hardware interface or real flight-control mechanism.
- Hardware-in-the-loop, serial/radio I/O, or ingestion of measured data from real devices.
- Engineering-grade trajectory prediction. The model is educational and its parameters are fictional.
- 3-DOF/6-DOF dynamics, guidance, navigation by GNSS, wind, Mach-dependent drag, databases, user accounts, cloud deployment.

## 2. Definitions

These four words are used with fixed meanings in every project document. Mixing them up is the most common source of bugs in this kind of system.

| Term | Meaning | Produced by |
|---|---|---|
| **Truth** | The simulator's exact internal state. No real system ever sees this. | Flight simulator |
| **Measurement** | A sensor reading: truth plus bias, noise, lag, saturation and quantization. | Sensor subsystem |
| **Estimate** | The avionics computer's belief about the flight, computed only from measurements. | Avionics computer |
| **Telemetry** | Estimates and measurements packaged and sent to the ground. | Telemetry infrastructure |

| Term | Meaning |
|---|---|
| **Simulated data** | Everything this system produces, including "measurements". All of it is synthetic. |
| **Measured data** | Data from physical instruments. This system never produces or accepts any. The label exists so the two can never be confused. |
| **Tick** | One physics step. Simulation time is always `tick × 0.001 s`. |
| **Sim time** (`t_sim_s`) | Seconds since scenario start, derived from the tick counter. Independent of the wall clock. |
| **Specific force** | What an accelerometer physically senses: acceleration minus gravity's contribution. Reads +g at rest on the pad and ≈0 in drag-free free fall. |
| **AGL / MSL** | Altitude above ground level (the pad) / above mean sea level. |
| **Scenario** | A JSON file fully defining one reproducible run: vehicle, environment, sensor and avionics parameters, and a random seed. |
| **FSM** | Finite state machine (the flight-state logic). |

Requirement keywords: **shall** = mandatory, **should** = desired, **may** = optional.
Verification methods: **T** = automated test, **A** = analysis, **I** = inspection/review, **D** = demonstration.

## 3. System overview

Six subsystems, each independently testable (detail in ARCHITECTURE.md):

1. **Flight simulator** — truth physics.
2. **Synthetic sensor subsystem** — turns truth into measurements.
3. **Virtual avionics computer** — estimates state and detects flight events from measurements only.
4. **Telemetry infrastructure** — encodes, sequences and distributes messages over WebSocket; REST mission control.
5. **Flight data recorder (FDR)** — writes every data category to disk, plus playback.
6. **Ground station** — React/TypeScript dashboard with a Three.js view.

## 4. Functional requirements

### 4.1 Scenarios (SCN)

| ID | Requirement | Ver. |
|---|---|---|
| FR-SCN-01 | A scenario shall be a JSON file validated on load; invalid files shall be rejected with a message naming the offending field. | T |
| FR-SCN-02 | Every physical quantity in a scenario shall use a unit-suffixed key (e.g. `dry_mass_kg`). | T, I |
| FR-SCN-03 | Every scenario shall carry `"fictional": true`; the loader shall reject scenarios without it. | T |
| FR-SCN-04 | A scenario shall contain one master seed from which all random streams are derived. | T |
| FR-SCN-05 | The loader shall reject: a dry mass that is not finite and positive, a propellant mass that is not finite or is negative (zero propellant is valid), thrust breakpoints not on the 1 ms tick grid, sensor/avionics/telemetry rates that do not divide the physics rate exactly, ignition earlier than 2 s after scenario start (pad calibration time), a constant-density scenario whose `environment.air_density_kgpm3` is missing, non-numeric, non-finite or not positive, and that parameter present in a scenario that uses any other atmosphere model. Unsupported or unexpected density parameters shall be rejected, never silently ignored. | T |

### 4.2 Flight simulator (SIM)

| ID | Requirement | Ver. |
|---|---|---|
| FR-SIM-01 | The simulator shall model vertical (1-DOF) translational motion with state altitude and vertical velocity. | T |
| FR-SIM-02 | Forces shall be thrust (scenario table), aerodynamic drag and gravity, as specified in ARCHITECTURE.md §4. | T |
| FR-SIM-03 | Atmospheric properties shall follow the two-layer standard-atmosphere model, valid for 0–20 km geopotential altitude. Leaving that range shall stop the run with a `MODEL_RANGE` fault. | T |
| FR-SIM-04 | Integration shall use a fixed 1 ms step driven by an integer tick counter. RK4 is the default; explicit Euler shall be selectable for teaching comparisons. | T |
| FR-SIM-05 | Vehicle mass shall decrease in proportion to delivered thrust impulse. | T |
| FR-SIM-06 | The simulator shall hold the vehicle on the pad until thrust exceeds weight, detect ground contact on descent, and then hold at rest for a configurable post-touchdown period. | T |
| FR-SIM-07 | The simulator shall accept vehicle commands (v1: `DEPLOY_DRAG_DEVICE`) and apply them on the next tick. | T |
| FR-SIM-08 | Each tick the simulator shall expose a truth record: time, altitude, velocity, acceleration, specific force, mass, thrust, drag, pressure, temperature, density, Mach number, roll rate and roll angle. | T |
| FR-SIM-09 | The simulator shall record one `VALIDITY` warning on each upward crossing of Mach 0.8 (constant drag coefficient no longer credible). It shall not repeat the warning on every tick while Mach remains above 0.8. | T |
| FR-SIM-10 | Simplified environment modes (vacuum, constant density, constant gravity) shall be selectable so results can be checked against closed-form solutions. Constant-density mode is a simplified reference atmosphere, not a realistic altitude-dependent model: density, pressure, temperature and speed of sound shall each hold one fixed reference value at all altitudes (ARCHITECTURE.md §4.5). Its density shall come from the explicit scenario parameter `environment.air_density_kgpm3`; no default density shall be applied. | T |
| FR-SIM-11 | A scenario may define a scripted roll-rate profile. It is kinematic only (prescribed, not computed from torques) and shall be labelled as such. *(Decision D-1.)* | T |
| FR-SIM-12 | The simulator package shall perform no file, network or wall-clock access. | T |

### 4.3 Synthetic sensors (SEN)

| ID | Requirement | Ver. |
|---|---|---|
| FR-SEN-01 | The system shall simulate a barometer, accelerometer, gyroscope and temperature sensor as specified in ARCHITECTURE.md §5. | T |
| FR-SEN-02 | Each sensor shall apply, in this order: dynamics (lag, where defined), bias, noise, saturation, quantization. | T |
| FR-SEN-03 | Each sensor shall sample at its configured rate on exact ticks. | T |
| FR-SEN-04 | Each sensor channel shall draw from its own random stream, so changing one sensor never alters another's noise. | T |
| FR-SEN-05 | Every sample shall carry sim time, sensor ID, channel, value, unit and a saturation flag. | T |
| FR-SEN-06 | An "ideal sensors" mode (all error terms off) shall exist for verification. | T |
| FR-SEN-07 | The accelerometer shall output specific force, not kinematic acceleration. | T |

### 4.4 Virtual avionics computer (AVI)

| ID | Requirement | Ver. |
|---|---|---|
| FR-AVI-01 | The avionics computer shall consume only sensor samples and its own tick count. It shall have no access to truth. | T, I |
| FR-AVI-02 | It shall estimate altitude AGL, vertical velocity and vertical acceleration. | T |
| FR-AVI-03 | It shall calibrate pad reference pressure, pad temperature and accelerometer bias while in `PAD_IDLE`. | T |
| FR-AVI-04 | It shall implement the flight states `PAD_IDLE → BOOST → COAST → DESCENT → LANDED`. Transitions shall be one-way. | T |
| FR-AVI-05 | It shall detect `LIFTOFF`, `BURNOUT`, `APOGEE` and `TOUCHDOWN` using debounced criteria, emitting each event exactly once with its sim time. | T |
| FR-AVI-06 | If the scenario enables the drag device, it shall issue `DEPLOY_DRAG_DEVICE` a configured delay after `APOGEE` and emit `DRAG_DEVICE_DEPLOYED`. *(Decision D-5.)* | T |
| FR-AVI-07 | It shall produce one telemetry frame every 50 ms of sim time (20 Hz). | T |

### 4.5 Telemetry infrastructure (TLM)

| ID | Requirement | Ver. |
|---|---|---|
| FR-TLM-01 | Messages shall be JSON conforming to TELEMETRY_PROTOCOL.md v1 and shall be validated before sending. | T |
| FR-TLM-02 | Vehicle-originated messages (telemetry and events) shall share one sequence counter per mission, starting at 0 and incrementing by 1. | T |
| FR-TLM-03 | Every message shall carry `data_origin: "SIMULATED"`. | T |
| FR-TLM-04 | Non-finite numbers (NaN, ±Infinity) shall never be serialized. Such a frame shall be rejected and logged as a fault. | T |
| FR-TLM-05 | The server shall distribute messages to multiple WebSocket clients in identical order. | T |
| FR-TLM-06 | Each client shall have a bounded send queue. A slow client shall lose its oldest queued messages (counted and reported) without delaying other clients. | T |
| FR-TLM-07 | On connect the server shall send a `hello` message describing protocol version, mission, rates and units; it shall send a `heartbeat` every 1 s. | T |
| FR-TLM-08 | A REST API shall create, start, pause, resume and abort missions, list scenarios and recorded flights, and start playback. | T |
| FR-TLM-09 | An optional seeded link-impairment model (drop, duplicate, delay) shall be available, off by default. *(Decision D-4.)* | T |

### 4.6 Flight data recorder and playback (FDR, PBK)

| ID | Requirement | Ver. |
|---|---|---|
| FR-FDR-01 | For each mission the recorder shall write: `manifest.json`, `truth.csv`, `truth_events.json`, `sensors.csv`, `avionics.csv`, `telemetry.jsonl`. | T |
| FR-FDR-02 | The manifest shall record: mission ID, scenario (embedded copy), seed, software version, protocol version, platform, rates, units per column, data category per file, `data_origin: "SIMULATED"`, completion status and SHA-256 of every data file. | T |
| FR-FDR-03 | Files shall be append-only during a run and flushed at least once per second of wall time. | T |
| FR-FDR-04 | CSV column names shall carry unit suffixes; numbers shall be written so they read back bit-identically. | T |
| FR-FDR-05 | Truth, measurements and estimates shall be stored in separate files and never mixed in one table. | T, I |
| FR-FDR-06 | A reader shall load a flight, verify checksums and report truncated or modified files explicitly. | T |
| FR-PBK-01 | Playback shall replay a recorded `telemetry.jsonl` through the same WebSocket interface as a live mission, marked `source: "playback"`. | T |
| FR-PBK-02 | Playback shall support speeds 0.25×–8×, pause, resume and seek. | T |
| FR-PBK-03 | Playback shall not execute simulator, sensor or avionics code. | T |
| FR-PBK-04 | Replayed message content shall equal the recorded content except `source` and `t_wall_utc`. | T |

### 4.7 Ground station (GND)

| ID | Requirement | Ver. |
|---|---|---|
| FR-GND-01 | The dashboard shall show the latest altitude, velocity, acceleration, pressure, temperature, flight state and sim time, each with its unit. | T, D |
| FR-GND-02 | It shall plot altitude, velocity, acceleration, pressure and temperature against sim time. | D |
| FR-GND-03 | It shall show a flight-state indicator and an event timeline. | T, D |
| FR-GND-04 | It shall render a 3D view driven only by received telemetry. | T, D |
| FR-GND-05 | It shall show link health: connection state, packets received, sequence gaps, duplicates, and age of the last packet. | T |
| FR-GND-06 | A "SIMULATED DATA" indicator and a LIVE/PLAYBACK indicator shall be visible at all times. | T, D |
| FR-GND-07 | It shall provide mission controls (create, start, pause, resume, abort, playback) via the REST API. | D |
| FR-GND-08 | It shall contain no physics. Values derived on the ground (e.g. maximum altitude) shall be labelled as derived. Frame-to-frame smoothing of the 3D view is allowed for display only and shall never appear as a data value. | I, T |
| FR-GND-09 | It shall reconnect automatically with back-off and mark data stale after 2 s without packets. | T |
| FR-GND-10 | Malformed, unknown-version or non-`SIMULATED` messages shall be rejected and counted, never displayed. | T |

## 5. Non-functional requirements

| ID | Requirement | Ver. |
|---|---|---|
| NFR-01 Determinism | Same scenario + seed + software version on the same platform shall give bit-identical recorded data (canonical form, TEST_PLAN §3.4). Across platforms, results shall agree within 1×10⁻⁹ relative. | T |
| NFR-02 Modularity | Module dependencies shall follow the rules in PROJECT_STRUCTURE.md §3, enforced by an automated test. | T |
| NFR-03 Independence | The simulation shall run headless (no server, no browser). The dashboard shall run from recorded data alone (no simulator). | T, D |
| NFR-04 Units | SI units throughout. Every variable, field and column holding a physical quantity carries a unit suffix. Physical constants live in one module. | I, T |
| NFR-05 Data labelling | Simulated data shall be labelled at every layer: scenario, messages, files, UI. Fictional parameters shall never be described as real vehicle specifications. | T, I |
| NFR-06 Dependencies | Third-party packages are limited to the allow-list in ARCHITECTURE.md §11. Each addition needs a written justification. | T, I |
| NFR-07 Testability | The simulation core (simulator, sensors, avionics) shall use the Python standard library only. Target line coverage for backend domain modules (`core`, `sim`, `sensors`, `avionics`): ≥ 90%. | T |
| NFR-08 Portability | Backend: Python 3.11 or newer on Windows, macOS, Linux. Frontend: current Chrome, Firefox, Edge with WebGL2. Exact versions pinned in Phase 2. | D |
| NFR-09 Security | Server binds to `127.0.0.1` by default; no authentication (local lab use only). CORS limited to the dev origin. File downloads are resolved from a fixed set of names, never from client-supplied paths. | T, I |
| NFR-10 Robustness | Client disconnects, malformed input and disk errors shall produce explicit faults, never silent data loss or a crashed server. | T |
| NFR-11 Maintainability | Python type hints and strict TypeScript throughout; function docstrings state units. | I |
| NFR-12 Result integrity | Test reports are produced only by test tooling and stored with the commit hash. No hand-written or estimated results. | I |

## 6. Performance requirements

All values below are **targets**. None has been measured; Phase 2 will measure and report them with the hardware used.

| ID | Target |
|---|---|
| PR-01 | Physics 1000 Hz; accelerometer and gyroscope 200 Hz; barometer 50 Hz; temperature 10 Hz; avionics 200 Hz; telemetry 20 Hz (all in sim time, exact). |
| PR-02 | Batch (unpaced) mode runs the reference scenario at ≥ 20× real time. |
| PR-03 | Paced mode keeps sim time within 50 ms of scaled wall time over a full flight. |
| PR-04 | Telemetry inter-arrival jitter on localhost: p95 ≤ 20 ms around the 50 ms nominal interval. |
| PR-05 | Frame creation to dashboard update on localhost: p95 ≤ 100 ms. |
| PR-06 | Dashboard renders ≥ 30 frames/s with the 3D view active; chart buffers are capped (15 min at 20 Hz). |
| PR-07 | PR-04 holds with 5 concurrent clients. |
| PR-08 | At most 1 s of recorded data is lost on abnormal termination. |
| PR-09 | Playback at 1× reproduces inter-message timing within 20 ms (p95). |

Feasibility note for PR-02: a throwaway calculation of the physics step alone took a few microseconds per step in pure Python. This is an order-of-magnitude indication, not a test result, and it excludes sensors, avionics and recording.

## 7. System constraints

| ID | Constraint |
|---|---|
| C-01 | Stack: Python (simulation), FastAPI (server), React + TypeScript (dashboard), Three.js (3D), WebSockets (live telemetry), CSV/JSON (storage), Pytest (backend tests). |
| C-02 | Software only. No hardware I/O of any kind. |
| C-03 | Vertical 1-DOF model; atmosphere valid 0–20 km; constant drag coefficient (subsonic use). |
| C-04 | No database. Storage is flat files under one data directory. |
| C-05 | Single machine, single server process, one active mission at a time *(decision D-8)*. |
| C-06 | Simulation code and visualization code share only the telemetry protocol. |
| C-07 | Frontend tests need a JavaScript test runner; Pytest cannot execute TypeScript *(decision D-7)*. |

## 8. Assumptions

| ID | Assumption | If wrong |
|---|---|---|
| A-01 | Users run everything on one computer via localhost. | Network exposure would require authentication and TLS (not designed). |
| A-02 | Educational fidelity is the goal; millimetre accuracy is not. | Model would need Mach-dependent drag, wind, 3-DOF. |
| A-03 | Flights last minutes, not hours. | Chart buffers, file sizes and truth logging rate need review. |
| A-04 | One operator; no concurrent command conflicts. | Mission control would need locking. |

## 9. Acceptance criteria

### 9.1 Phase 1 exit (this review)

- All five documents reviewed; open decisions D-1 to D-10 resolved.
- Requirement IDs stable and each mapped to at least one planned test (TEST_PLAN §12).
- Risk register accepted (ARCHITECTURE §12).

### 9.2 System acceptance (end of implementation)

| ID | Criterion |
|---|---|
| AC-01 | A headless run of reference scenario REF-01 completes and produces a complete, checksum-valid flight record. |
| AC-02 | Two runs with the same scenario and seed produce identical canonical hashes. With the drag device disabled, truth is identical for every seed. |
| AC-03 | All physics tests pass against closed-form solutions within the tolerances fixed in TEST_PLAN. |
| AC-04 | Flight events are detected within tolerance for 100 seeds of REF-01, with zero false liftoffs. |
| AC-05 | Every message of a full run validates against the schema; sequence numbers are contiguous; no non-finite values. |
| AC-06 | The dashboard's displayed values equal the received packet values under the documented formatting rules; link health counters match injected faults exactly; the SIMULATED indicator is always present. |
| AC-07 | Playing back a recorded flight yields the same dashboard view-state as the live run. |
| AC-08 | The simulation runs without the server or frontend; the dashboard runs from playback without simulator code executing. |
| AC-09 | Dependency-rule and allow-list tests pass. |
| AC-10 | Every performance target in §6 is measured and reported with hardware details. Misses are documented, not hidden. |
| AC-11 | The test report is tool-generated and tied to a commit hash. |

## 10. Decisions

| ID | Decision | Recommendation | Outcome |
|---|---|---|---|
| D-1 | A vertical-only model has no rotation, so the gyroscope and 3D attitude have nothing physical to show. Options: (a) gyro outputs bias + noise only; (b) scripted kinematic roll profile; (c) upgrade to 3-DOF. | **(b)**. Cheap, honest if labelled, gives the gyro and 3D view real content. (c) roughly doubles the physics and test scope. | **Approved 2026-11-11**, as recommended. |
| D-2 | Telemetry carries avionics estimates and raw measurements. Truth is recorded only by the FDR and never downlinked. | **Approve.** Otherwise the avionics computer is a pass-through and there is nothing to estimate or validate. | **Approved 2026-11-11**, as recommended. |
| D-3 | "Timestamp" is sim time (`t_sim_s`). Wall-clock time is an informational extra added by the server. | **Approve.** A wall-clock timestamp makes runs non-reproducible. | **Approved 2026-11-11**, as recommended. |
| D-4 | WebSocket runs over TCP, which never loses or reorders data, so sequence numbers would never detect anything. Add a seeded link-impairment model (drop, duplicate, delay)? | **Yes, off by default.** It is what makes "telemetry integrity" testable. Corruption/CRC deferred. | **Approved 2026-11-11**, as recommended. |
| D-5 | Include a simulated descent drag device commanded by the avionics. | **Yes.** It closes the loop so that event-detection quality affects the flight. It is a single drag-area parameter change. | **Approved 2026-11-11**, as recommended. |
| D-6 | Storage: CSV for time series, JSON Lines (one JSON object per line) for messages, JSON manifest. | **Approve.** A single JSON document cannot be appended safely; bare CSV loses units and metadata. | **Approved 2026-11-11**, as recommended. |
| D-7 | Frontend tooling: Vitest for tests; uPlot for charts; plain Three.js (no react-three-fiber); no state-management library. | **Approve.** Smallest set that works. | **Approved 2026-11-11**, as recommended. |
| D-8 | One active mission at a time. | **Approve** for v1. | **Approved 2026-11-11**, as recommended. |
| D-9 | Reference hardware and browser for performance targets. | Needs your input: the machine you will develop on. | **Decided 2026-11-11:** ASUS TUF A15 (the development machine), Windows 11, Google Chrome Stable. The actual CPU, GPU, RAM, operating-system version and browser version are recorded at measurement time, together with the results. |
| D-10 | No NumPy/SciPy/pandas in the simulation core. | **Approve.** The model is two scalar equations; the standard library is enough and removes a determinism variable. | **Approved 2026-11-11**, as recommended. |
