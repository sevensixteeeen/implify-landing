# AEROSAT-01 — System Architecture

| | |
|---|---|
| Document | ARCHITECTURE.md |
| Version | 0.1 — DRAFT FOR REVIEW |
| Date | 2026-11-10 |
| Scope | Subsystems, mathematical model, sensors, avionics logic, software interfaces, risks |

> Every number describing the vehicle, its thrust or its sensors in this document is a **fictional simulation parameter** chosen for convenient arithmetic. None is derived from, or should be read as, a real rocket, motor or sensor specification.

---

## 1. Architectural principles

1. **One-way data flow.** Truth → measurements → estimates → telemetry → display. The only path backwards is an explicit vehicle command from avionics to simulator.
2. **Truth is quarantined.** Only the sensor subsystem and the recorder may read truth. The avionics computer and the dashboard never do.
3. **Sim time, not wall time.** Physics advances by integer ticks. Real-time pacing is a separate layer that only decides *when* to run the next ticks.
4. **Pure core, thin shell.** Simulator, sensors and avionics are plain Python with no I/O, no async and no third-party packages. Files, network and timing live in outer layers.
5. **The protocol is the only contract** between backend and frontend.
6. **Everything is labelled** simulated, with units.

## 2. Subsystem decomposition

| # | Subsystem | Responsibility | Reads | Produces | Must not |
|---|---|---|---|---|---|
| 1 | Flight simulator | Truth physics, ground constraints, command execution | Scenario, vehicle commands | Truth state per tick | Do I/O, read the clock, know about sensors |
| 2 | Sensor subsystem | Convert truth to measurements with error models | Truth state | Sensor samples | Expose truth to anyone else |
| 3 | Avionics computer | Calibration, state estimation, flight FSM, event detection, telemetry frames, commands | Sensor samples | Estimates, events, frames, commands | Import simulator or read truth |
| 4 | Telemetry infrastructure | Encode, validate, sequence, impair (optional), fan out over WebSocket; REST control | Frames, events | JSON messages | Alter values, run physics |
| 5 | Flight data recorder + playback | Persist all categories; replay recorded telemetry | Truth, samples, estimates, messages | Flight record on disk; replayed messages | Mix data categories in one file |
| 6 | Ground station | Display, charts, 3D view, link health, mission controls | JSON messages, REST | UI | Compute physics, show unlabelled derived data |

```
            scenario.json  (fictional parameters + seed)
                   |
                   v
  +----------------------------------------------------------+
  |  MISSION RUNNER  (tick scheduler; composition root)      |
  |                                                          |
  |   [1 Flight simulator] --truth--> [2 Sensors]            |
  |          ^                            |                  |
  |          | vehicle commands           | samples          |
  |          | (applied next tick)        v                  |
  |          +------------------- [3 Avionics computer]      |
  |                                       |                  |
  |                         frames, events| estimates        |
  |                                       v                  |
  |   truth, samples, estimates --> [5 Flight data recorder] |
  +---------------------------------------|------------------+
                                          | messages (pre-link)
                                          v
     [5 Playback] --recorded messages--> [4 Telemetry infrastructure]
                                          |  link model -> hub -> WebSocket
                                          |  REST mission control (FastAPI)
                                          v
                               [6 Ground station: React + TS + Three.js]
```

## 3. Data categories

| Category | Type name | File | Who may consume |
|---|---|---|---|
| Truth | `TruthState` | `truth.csv`, `truth_events.json` | Sensors, recorder, tests |
| Measurement | `SensorSample` | `sensors.csv` | Avionics, recorder |
| Estimate | `AvionicsEstimate` | `avionics.csv` | Telemetry encoder, recorder |
| Telemetry | `DownlinkMessage` | `telemetry.jsonl` | Telemetry infrastructure, playback, ground |

Using distinct types (not one shared dictionary) means a truth value cannot be passed where a measurement is expected without an explicit, visible conversion.

---

## 4. Mathematical model (educational, 1-DOF vertical)

### 4.1 Conventions

- One axis, **positive up**. All quantities SI.
- `h` = geometric altitude above mean sea level (m). `h_agl = h − h_pad`.
- `v` = vertical velocity (m/s). `a` = vertical acceleration (m/s²).
- `t` = sim time (s) = `tick × Δt`, with `Δt = 0.001 s`. Time is never accumulated by repeated addition.

### 4.2 Constants

| Symbol | Value | Unit | Meaning |
|---|---|---|---|
| g₀ | 9.80665 | m/s² | Standard gravity |
| r₀ | 6 356 766 | m | Reference Earth radius used by the standard atmosphere |
| T₀ | 288.15 | K | Sea-level standard temperature |
| p₀ | 101 325 | Pa | Sea-level standard pressure |
| L | 0.0065 | K/m | Temperature lapse rate, 0–11 km |
| R | 287.05287 | J/(kg·K) | Specific gas constant of dry air |
| γ | 1.4 | – | Ratio of specific heats (for speed of sound) |

These are standard published constants, not fictional parameters. In code they are named `G0_MPS2`, `R0_M`, `T0_K`, `P0_PA`, `LAPSE_RATE_KPM`, `R_AIR_JPKGK` and `GAMMA` (PROJECT_STRUCTURE §4).

### 4.3 Equations of motion

```
dh/dt = v
dv/dt = a = ( F_T(t) − F_D(h, v) ) / m(t) − g(h)
```

Unit check: N / kg = m/s² ✓.

### 4.4 Gravity

```
g(h) = g₀ · ( r₀ / (r₀ + h) )²
```

At 2 km this differs from g₀ by 0.06 %, at 10 km by 0.3 %. It is included because it costs one line and makes the truth model deliberately different from the avionics model (which assumes constant g₀), a realistic source of estimation error.

### 4.5 Atmosphere

Geopotential altitude (the altitude scale the standard atmosphere is defined on):

```
H = r₀ · h / (r₀ + h)
```

Layer 1, 0 ≤ H ≤ 11 000 m:

```
T = T₀ − L·H
p = p₀ · (T / T₀)^( g₀ / (R·L) )          exponent = 5.25588
```

Layer 2, 11 000 < H ≤ 20 000 m (isothermal, T₁₁ = 216.65 K):

```
T = T₁₁
p = p₁₁ · exp( −g₀ · (H − 11 000) / (R · T₁₁) )
```

Both layers:

```
ρ = p / (R · T)                 density, kg/m³
c = sqrt( γ · R · T )           speed of sound, m/s
M = |v| / c                     Mach number (diagnostic only)
```

Values the formulas produce (used as a self-consistency check; the independent oracle for tests is a published table, see TEST_PLAN PHY-01):

| H (m) | T (K) | p (Pa) | ρ (kg/m³) |
|---|---|---|---|
| 0 | 288.15 | 101 325 | 1.2250 |
| 1 000 | 281.65 | 89 874.6 | 1.11164 |
| 5 000 | 255.65 | 54 019.9 | 0.736116 |
| 11 000 | 216.65 | 22 632.1 | 0.363918 |
| 20 000 | 216.65 | 5 474.88 | 0.0880347 |

**Constant-density mode** (`atmosphere_model: "constant_density"`, FR-SIM-10). This is a simplified reference atmosphere for checking results against closed-form solutions. It is not a realistic altitude-dependent atmospheric model: every atmospheric property keeps one fixed educational reference value at all altitudes.

| Quantity | Constant value | Origin |
|---|---|---|
| Density ρ | 1.225 kg/m³ | Scenario parameter `environment.air_density_kgpm3`, stated explicitly in the scenario |
| Pressure p | 101 325 Pa | p₀ (§4.2) |
| Temperature T | 288.15 K | T₀ (§4.2) |
| Speed of sound c | ≈ 340.3 m/s | `sqrt(γ·R·T₀)` |

The density is never applied as a hidden default: the scenario must state `environment.air_density_kgpm3`, and validation is strict (FR-SCN-05). A missing, non-numeric, non-finite or non-positive value is rejected, and so is the parameter in a scenario that uses any other atmosphere model. Nothing is silently ignored. The constant-density verification scenario sets 1.225 kg/m³ (TEST_PLAN §4). The 0–20 km model range (§4.10) still applies in this mode.

**Vacuum mode** (`atmosphere_model: "vacuum"`, FR-SIM-10). Density and pressure are zero at every altitude, so there is no drag. Temperature and speed of sound keep the sea-level reference values (288.15 K, ≈ 340.3 m/s). This is a computational convention only, so that the truth record and the Mach number stay finite. It is not a physical model of sound speed in a vacuum. The 0–20 km model range still applies.

### 4.6 Drag

```
F_D = ½ · ρ(h) · (C_d·A) · v · |v|
```

The `v·|v|` form makes drag always oppose motion, on ascent and descent, with no branching. Unit check: kg/m³ · m² · m²/s² = N ✓.

`C_d·A` (drag coefficient × reference area, m²) is one constant per configuration: `C_d·A` for the vehicle, and a larger value after `DEPLOY_DRAG_DEVICE`. On deployment the drag device's `C_d·A` (1.5 m² in REF-01) replaces the vehicle's value; it is not added to it.

### 4.7 Thrust and mass (fictional input)

Thrust is a piecewise-linear table of (time since ignition in integer ms, thrust in N). Outside the table thrust is zero.

```
J(t)  = ∫₀ᵗ F_T dτ                               delivered impulse, N·s (exact: trapezoids)
m(t)  = m_dry + m_prop · ( 1 − J(t) / J_total )
```

Mass is an algebraic function of time, not an integrated state. A vehicle without a thrust table (`J_total = 0`) keeps its initial mass `m_dry + m_prop`. Zero propellant is valid. Mass validation is strict: the dry mass must be finite and positive, and the propellant mass finite and not negative. This assumes propellant is consumed in proportion to thrust, i.e. a constant effective exhaust velocity `J_total / m_prop`, which for fictional inputs is itself a fictional number with no engineering meaning.

Breakpoints are integer milliseconds so every thrust discontinuity lands exactly on a tick. RK4 loses accuracy when a slope discontinuity falls inside a step.

### 4.8 Ground constraints

- **Initial conditions:** a normal launch scenario starts at rest on the pad at tick 0. A verification run may instead be initialised in flight, with an initial altitude above the pad and an initial velocity, so that it can be compared with a closed-form solution. These two values are simulator settings (`initial_altitude_agl_m`, `initial_velocity_mps`) supplied by the test. They are not scenario-file fields.
- **Pad hold:** while the vehicle has never left the pad, if the computed `a < 0` then `a = 0`, `v = 0`, `h = h_pad`.
- **Touchdown:** when `h` crosses `h_pad` with `v < 0`, the touchdown time is linearly interpolated inside the step, the impact velocity is recorded as a truth event, and the state is set to `h = h_pad`, `v = 0`, `a = 0`. The run continues for the post-touchdown hold (default 5 s) so the avionics can detect landing.
- **Range check scope:** the `MODEL_RANGE` check applies to accepted states at the end of a step. Intermediate RK4 stage evaluations that dip fractionally below the pad use the pad-altitude atmosphere.
- **Specific force** (accelerometer truth) is always `f = a + g(h)`. On the pad and after touchdown this gives `+g`; in powered flight `(F_T − F_D)/m`; in coast `−F_D/m`.

### 4.9 Numerical integration

State `y = [h, v]`, derivative `f(t, y) = [v, a(t, h, v)]`, classical fourth-order Runge–Kutta:

```
k1 = f(t,        y)
k2 = f(t + Δt/2, y + Δt/2 · k1)
k3 = f(t + Δt/2, y + Δt/2 · k2)
k4 = f(t + Δt,   y + Δt   · k3)
y(t + Δt) = y + Δt/6 · (k1 + 2·k2 + 2·k3 + k4)
```

Explicit Euler (`y + Δt·k1`) is selectable so the difference in accuracy can be demonstrated.

**Why Δt = 1 ms.** Accuracy alone would allow a much larger step. 1 ms is chosen because it puts every sensor and telemetry rate (200, 50, 20, 10 Hz) on an exact integer tick grid and bounds event-timing resolution to 1 ms. The cost is negligible for a two-variable system.

### 4.10 Validity limits

| Limit | Handling |
|---|---|
| 0 ≤ H ≤ 20 km | Hard stop with `MODEL_RANGE` fault outside |
| Constant C_d, credible only well below Mach 1 | One `VALIDITY` warning recorded on each upward crossing of M = 0.8; not repeated on every tick while M stays above it |
| No wind, no lateral motion, no rotation dynamics | Stated limitation; roll is scripted (D-1) |
| Standard atmosphere only (no weather) | Stated limitation |
| Constant-density mode is a fixed reference atmosphere, not altitude-dependent (§4.5) | Stated limitation; for verification against closed-form solutions only |
| Instantaneous drag-device deployment | Stated limitation; deployment occurs near zero velocity in REF-01 |

### 4.11 Reference scenario REF-01 (fictional)

| Parameter | Value |
|---|---|
| Dry mass / propellant mass | 15.0 kg / 5.0 kg |
| C_d, reference area | 0.5, 0.010 m² (C_d·A = 0.005 m²) |
| Thrust table (ms after ignition → N) | 0 → 0; 100 → 1500; 3000 → 1500; 3200 → 0 |
| Total impulse (exact, trapezoids) | 75 + 4350 + 150 = 4575 N·s |
| Ignition | 3000 ms after scenario start (pad calibration time) |
| Drag device C_d·A, deploy delay | 1.5 m², 1000 ms after detected apogee |
| Pad altitude | 0 m MSL |
| Seed | 42 |

### 4.12 Design-time sanity check (not a test result)

Before writing this document, the equations above were exercised in a throwaway script that is not part of the project. It showed:

- The atmosphere formulas reproduce the table in §4.5.
- RK4 agreed with the closed-form quadratic-drag solutions (TEST_PLAN PHY-07, PHY-08) to round-off level, which confirms the oracle formulas are stated correctly.
- REF-01 stays inside the model's validity limits: apogee roughly 2.1 km AGL, peak Mach roughly 0.66, flight roughly three minutes, descent roughly 12.7 m/s under the drag device.
- Changing Δt between 4, 2 and 1 ms moved apogee by millimetres, far below the barometer's noise-equivalent altitude (~0.25 m).

These figures only confirm the scenario is sensible. Authoritative reference values will be generated and frozen in Phase 2, after the physics tests pass.

---

## 5. Sensor architecture

### 5.1 Common measurement model

For each sensor channel, at each sample tick `k`:

```
x_k   = true quantity (from TruthState)
x'_k  = lag(x_k)                    only where a time constant τ is defined
y_k   = Q( sat( x'_k + b + σ·w_k ) )       w_k ~ N(0, 1), independent
```

- `b` — bias, drawn once per run from `N(0, σ_b)`, constant thereafter.
- `σ` — white-noise standard deviation per sample.
- `sat` — clip to the sensor range; sets a `saturated` flag.
- `Q` — round to the nearest multiple of the resolution.
- First-order lag: `x' ← x' + (1 − exp(−T_s/τ)) · (x − x')`, `T_s` = sample period. The lag state is initialised to the first true value, so the first sample equals truth (before bias and noise) and there is no artificial start-up transient.

### 5.2 Sensor definitions

Default values are **illustrative**, of the general magnitude of hobby-grade parts, not taken from any datasheet. All are scenario-configurable.

**Channels.** A scalar sensor (barometer, temperature) has one channel, named after its quantity: `pressure`, `temperature`. A vector sensor (accelerometer, gyroscope) has three channels: `x`, `y`, `z`.

**Units.** Unit strings carried in data (samples, the `hello` message, the manifest) are exactly `Pa`, `m/s^2`, `rad/s` and `K`, written in ASCII.

**Enabled flag.** Each sensor has an `enabled` Boolean in its scenario section. It defaults to true when omitted; any value that is not a JSON Boolean is rejected. A disabled sensor is never sampled and generates no samples. The enabled status of all four sensors is recorded in the mission metadata (manifest, FR-FDR-02).

| | Barometer | Accelerometer | Gyroscope | Temperature |
|---|---|---|---|---|
| Sensor ID | `baro` | `accel` | `gyro` | `temp` |
| Quantity | Static pressure | Specific force | Angular rate | Ambient air temperature |
| Kind | Scalar | Vector | Vector | Scalar |
| Unit string | `Pa` | `m/s^2` | `rad/s` | `K` |
| Channels | 1: `pressure` | 3: `x`, `y`, `z` (z = up) | 3: `x`, `y`, `z` (z = roll) | 1: `temperature` |
| Sample rate | 50 Hz | 200 Hz | 200 Hz | 10 Hz |
| Ticks per sample | 20 | 5 | 5 | 100 |
| Range | 1 000 – 110 000 | ±160 | ±35 | 200 – 330 |
| Resolution | 1 | 0.005 | 0.001 | 0.01 |
| Noise σ | 3 | 0.05 | 0.002 | 0.05 |
| Bias σ_b | 30 | 0.10 | 0.005 | 0.5 |
| Lag τ | – | – | – | 2.0 s |
| Truth source | `p(h)` | `f = a + g(h)` on z; 0 on x, y | Scripted roll rate on z; 0 on x, y | `T(h)` |

Notes:

- 3 Pa of pressure noise corresponds to about 0.25 m of altitude at sea level (`Δh ≈ Δp / (ρ·g)`).
- In a 1-DOF model the lateral accelerometer axes and the pitch/yaw gyro axes have zero truth; they output bias and noise only. These channels are recorded in `sensors.csv` and are not downlinked.
- The temperature sensor is idealised as measuring ambient air with a thermal lag. A real board-mounted sensor mostly measures the board. This is a simplification, not a claim.
- Not modelled in v1: scale-factor error, axis misalignment, temperature-dependent drift, bias random walk, sample-time jitter, sensor latency.

### 5.3 Random number generation

- Standard library only. One `random.Random` instance per stream.
- Stream seed = first 8 bytes of `SHA-256("{master_seed}:{stream_name}")`, where the text is encoded as UTF-8 and the eight bytes are interpreted as a big-endian unsigned integer (0 to 2⁶⁴ − 1). Stream names: a scalar sensor uses `{sensor_id}.noise` and `{sensor_id}.bias` (`baro.noise`, `baro.bias`, `temp.noise`, `temp.bias`); a vector sensor uses `{sensor_id}.{channel}.noise` and `{sensor_id}.{channel}.bias` (`accel.x.noise`, `accel.y.noise`, `accel.z.noise`, `gyro.z.bias`).
- Gaussian values are produced by a project-owned Box–Muller transform on `Random.random()`. Python guarantees the reproducibility of `random()` for a given seed across versions; it does not guarantee that for `gauss()`.

---

## 6. Virtual avionics computer

Runs at 200 Hz (every 5 ticks). Knows g₀, the standard-atmosphere constants and its own configuration. Knows nothing else about the vehicle or the truth model.

### 6.1 Pad calibration (state `PAD_IDLE`)

Over the first 1.0 s: `p_ref` = mean barometer reading, `T_ref` = mean temperature reading, `b_a` = mean z-accelerometer reading − g₀. The scenario must provide at least 2 s before ignition.

Until calibration completes, the estimates sent in telemetry (`altitude_m`, `velocity_mps`, `acceleration_mps2`) are 0.0 (TELEMETRY_PROTOCOL §8).

### 6.2 Estimator

Barometric altitude (AGL), inverting the layer-1 pressure law:

```
h_baro = (T_ref / L) · ( 1 − (p / p_ref)^(R·L/g₀) )         exponent = 0.190263
```

Complementary filter (blends fast-but-drifting accelerometer integration with slow-but-stable barometer altitude):

```
predict, 200 Hz:   a_est = (f_z − b_a) − g₀
                   h_est += v_est·T + ½·a_est·T²
                   v_est += a_est·T                      T = 0.005 s
correct, 50 Hz:    r = h_baro − h_est
                   h_est += K_h · r                      K_h dimensionless
                   v_est += K_v · r                      K_v in 1/s
```

Gains `K_h`, `K_v` are stored in the scenario as `avionics.estimator_gain_h` and `avionics.estimator_gain_v_hz`. REF-01 uses `K_h = 0.24` and `K_v = 0.72 s⁻¹`, chosen by this analysis. With corrections every `T_b = 0.02 s`, the estimate error behaves like a second-order system with `2ζωₙ = K_h / T_b` and `ωₙ² = K_v / T_b`; these gains give `ωₙ = 6 rad/s`, `ζ = 1` (critically damped, no overshoot). At touchdown the true velocity stops instantly but the accelerometer sees no impulse, so the velocity estimate starts about 12.7 m/s in error and decays as `(1 + ωₙt)·e^(−ωₙt)`. It falls below the 2 m/s touchdown criterion after about 0.55 s, so `TOUCHDOWN` is detected about 2.55 s after ground contact, inside the 3.0 s tolerance. The lower gains first tried (`ωₙ = 3 rad/s`) detected it 3.05 to 3.13 s after contact and were rejected. A Kalman filter is a possible later replacement behind the same interface; it is not justified for v1.

### 6.3 Flight state machine

| Transition | Event | Default criterion (all configurable) |
|---|---|---|
| `PAD_IDLE → BOOST` | `LIFTOFF` | Calibration complete and `f_z − b_a > 15 m/s²` for 10 consecutive accelerometer samples (50 ms) |
| `BOOST → COAST` | `BURNOUT` | `f_z − b_a < 0` for 10 consecutive samples (50 ms) |
| `COAST → DESCENT` | `APOGEE` | `v_est < 0` for 10 consecutive avionics ticks (50 ms), and ≥ 1.0 s since `BURNOUT` (lock-out) |
| (within `DESCENT`) | `DRAG_DEVICE_DEPLOYED` | Configured delay after `APOGEE`; issues `DEPLOY_DRAG_DEVICE` |
| `DESCENT → LANDED` | `TOUCHDOWN` | Over a sliding 2.0 s window: range of `h_est` < 3 m and `|v_est|` < 2 m/s |

Rules: transitions are one-way; each event fires once; counters reset when a criterion is interrupted (debounce). Physical reasoning behind the burnout test: after thrust ends the only non-gravitational force is drag, so specific force turns negative.

---

## 7. Timing and scheduling

Per tick `n` (sim time `n × 1 ms`), in this fixed order:

1. Simulator applies commands issued on the previous tick, then advances one step.
2. Sensors whose period divides `n` sample the new truth state.
3. If `n mod 5 = 0`: avionics consumes new samples, updates estimate and FSM, may emit events and commands.
4. If `n mod 50 = 0`: avionics emits a telemetry frame.
5. Recorder receives truth (every 5th tick by default), samples, estimates and messages.
6. Messages are handed to the telemetry infrastructure.

Tick 0 is the initial state: the simulator does not step (step 1 is skipped), and steps 2–6 run on the initial truth state. Sensors, avionics and telemetry therefore all produce output at tick 0.

**Run modes**

- **Batch:** ticks run as fast as possible (CLI, tests).
- **Paced:** ticks run in 50 ms sim-time slices; after each slice the runner waits until `wall_start + t_sim / time_scale`. Each slice yields exactly one telemetry frame. Pacing affects only *when* slices run, never their results.

---

## 8. Software architecture

### 8.1 Backend modules

| Package | Responsibility | May import |
|---|---|---|
| `aerosat.core` | Constants, unit conventions, tick clock, seeded RNG streams, shared data types, fault types | standard library |
| `aerosat.sim` | Atmosphere, gravity, vehicle (thrust, mass, drag), dynamics, integrators, simulator | `core` |
| `aerosat.sensors` | Error pipeline, four sensor models, sensor suite | `core` |
| `aerosat.avionics` | Calibration, estimator, FSM, flight computer | `core` |
| `aerosat.telemetry` | Message models and JSON Schema export, encoder, link model, subscriber hub | `core`, Pydantic |
| `aerosat.recorder` | Writers, manifest, checksums, reader | `core` |
| `aerosat.playback` | Replay a recorded flight as a message stream | `core`, `recorder`, `telemetry` |
| `aerosat.mission` | Scenario loading, tick scheduler, pacing, mission lifecycle (composition root) | `core`, `sim`, `sensors`, `avionics`, `telemetry`, `recorder` |
| `aerosat.server` | FastAPI app, REST routes, WebSocket endpoint | `core`, `mission`, `telemetry`, `recorder`, `playback`, FastAPI |
| `aerosat.cli` | Headless batch runs, schema export, golden-file generation | `core`, `mission`, `recorder`, `telemetry` |

`sim`, `sensors` and `avionics` never import each other. They meet only in `mission`.

### 8.2 Core interfaces (design sketch, not implementation)

```python
class Simulator(Protocol):
    def step(self, commands: Sequence[VehicleCommand]) -> TruthState: ...

class Sensor(Protocol):
    sensor_id: str
    def is_due(self, tick: int) -> bool: ...
    def sample(self, truth: TruthState) -> list[SensorSample]: ...

class FlightComputer(Protocol):
    def tick(self, tick: int, samples: Sequence[SensorSample]) -> AvionicsOutput: ...
    # AvionicsOutput: estimate, flight_state, events, commands, optional telemetry frame

class MessageSink(Protocol):
    def publish(self, message: DownlinkMessage) -> None: ...

class Recorder(Protocol):
    def record_truth(self, s: TruthState) -> None: ...
    def record_samples(self, s: Sequence[SensorSample]) -> None: ...
    def record_estimate(self, e: AvionicsEstimate) -> None: ...
    def record_message(self, line: str) -> None: ...
    def finalize(self, status: str) -> None: ...
```

`record_message` receives the line already serialized by the telemetry encoder (the recorded form, TELEMETRY_PROTOCOL §9). The recorder never serializes messages itself, so there is exactly one serializer and `recorder` needs no import from `telemetry`.

### 8.3 REST API (prefix `/api/v1`)

| Method and path | Purpose | Success | Errors |
|---|---|---|---|
| `GET /health` | Liveness, software and protocol versions | 200 | – |
| `GET /scenarios` | List scenarios | 200 | – |
| `GET /scenarios/{scenario_id}` | Scenario content | 200 | 404 |
| `POST /missions` | Create mission: `{scenario_id, seed?, time_scale?, link_profile?}` | 201 `{mission_id, status}` | 404, 409 (one already active), 422 |
| `GET /missions/{mission_id}` | Status, sim time, flight state, client count | 200 | 404 |
| `POST /missions/{mission_id}/commands` | `{command: START \| PAUSE \| RESUME \| ABORT}` | 202 | 404, 409 (invalid transition), 422 |
| `GET /flights` | List recorded flights | 200 | – |
| `GET /flights/{mission_id}` | Manifest | 200 | 404 |
| `GET /flights/{mission_id}/files/{name}` | Download one file; `name` from a fixed list | 200 | 404 |
| `POST /playbacks` | Start playback: `{flight_id, speed?}` | 201 `{mission_id}` | 404, 409, 422 |
| `POST /playbacks/{mission_id}/commands` | `{command: PLAY \| PAUSE \| SEEK, t_sim_s?, speed?}` | 202 | 404, 409, 422 |

Mission status: `CREATED → RUNNING ⇄ PAUSED → COMPLETED | ABORTED | FAILED`.

Playback reuses the recorded flight's `mission_id`: the `mission_id` returned by `POST /playbacks` equals `flight_id`. A playback counts as the one active mission (C-05), so `POST /missions` and `POST /playbacks` return 409 while either a live mission or a playback is active.

### 8.4 WebSocket

- Endpoint: `/ws/v1/missions/{mission_id}`.
- Server → client only. Commands go through REST, which keeps the telemetry path read-only and simple to test.
- Message types: `hello`, `telemetry`, `event`, `status`, `heartbeat`, `error` (TELEMETRY_PROTOCOL.md).
- No history backfill in v1: a client that joins or reconnects mid-flight sees data from that moment. The full flight is available from the recorder afterwards.

### 8.5 Frontend modules

| Folder | Responsibility | May import |
|---|---|---|
| `protocol/` | Message types, runtime validators | – |
| `net/` | WebSocket client with reconnect; REST client | `protocol` |
| `state/` | Pure reducer: messages → view-state (latest values, ring buffers, link statistics, event timeline); selectors and unit formatting | `protocol` |
| `scene/` | Three.js scene as a plain class: `init(canvas)`, `update(viewState)`, `dispose()` | `state` types, Three.js |
| `components/` | React panels, charts, controls | `protocol` (types only), `state`, `scene`, `net`, React, uPlot |
| `App.tsx`, `main.tsx` | Application shell and entry point | `components`, `state`, `net`, React |

The reducer and selectors have no React, DOM or network dependency, so display consistency is testable without a browser.

---

## 9. Flight data recorder and playback

Flight record: `data/flights/{mission_id}/`

| File | Category | Format | Default rate |
|---|---|---|---|
| `manifest.json` | Metadata | JSON | once (updated at finalize) |
| `truth.csv` | Truth | CSV, one row per logged tick | 200 Hz (configurable to 1000) |
| `truth_events.json` | Truth | JSON: true liftoff, burnout, apogee, touchdown, impact velocity | once |
| `sensors.csv` | Measurement | CSV, long format: `tick, t_sim_s, sensor_id, channel, value, unit, saturated` | per sensor |
| `avionics.csv` | Estimate | CSV | 200 Hz |
| `telemetry.jsonl` | Telemetry | JSON Lines, every vehicle-originated message in sequence order, before the link model | 20 Hz + events |

- Numbers are written with Python's shortest round-trip representation, so reading them back is bit-exact.
- Wall-clock times appear only in the manifest. Data files contain no wall-clock values, which keeps them reproducible.
- The recorder never reads a clock. `mission` passes it the wall-clock timestamps written to the manifest and triggers the periodic flush (FR-FDR-03).
- Manifest status is `INCOMPLETE` until `finalize` writes checksums and the final status.
- Playback reads `telemetry.jsonl`, sets `source` to `playback`, and paces messages by their `t_sim_s`. It never touches simulator, sensor or avionics code.

## 10. Ground station design notes

- **Store outside React.** WebSocket messages update a plain store; components subscribe to slices. Charts are appended to imperatively rather than re-rendered from props.
- **Three.js stays imperative.** One scene object with its own animation loop reads the latest view-state. React only mounts and unmounts it.
- **Scale problem.** A metre-scale vehicle on a kilometre-scale trajectory is invisible at true scale. The 3D view uses a chase camera that follows the vehicle, with an altitude tape and ground reference, rather than a fixed wide shot.
- **Roll angle.** Telemetry carries roll rate only. The roll angle used to spin the 3D model is accumulated on the ground from received roll-rate values, for display, and is labelled as ground-derived.
- **Display smoothing.** Between 20 Hz packets the scene may interpolate position for smooth motion. Numeric readouts always show the last received packet value, never an interpolated one.

## 11. Dependency allow-list

Exact versions are pinned in `backend/uv.lock` (uv) and `frontend/package-lock.json` (npm); both are committed.

| Area | Package | Why needed |
|---|---|---|
| Backend runtime | FastAPI | Required stack |
| | Uvicorn (installed as `uvicorn[standard]`) | ASGI server to run FastAPI. The `standard` extra supplies the WebSocket protocol implementation Uvicorn needs to serve FastAPI/Starlette WebSocket endpoints (approved 2026-11-11). The frontend uses the native browser WebSocket API; no WebSocket client library. |
| | Pydantic | Comes with FastAPI; message models and JSON Schema export |
| Backend dev | Pytest | Required stack |
| | httpx2 | Required by Starlette's test client, which FastAPI's test client wraps (Starlette 1.10 deprecates `httpx` for this) (approved 2026-11-11). |
| | pytest-cov | Coverage reporting for NFR-07 |
| Frontend runtime | React, React DOM | Required stack |
| | Three.js | Required stack |
| | uPlot | Small canvas chart library (D-7) |
| Frontend dev | TypeScript, Vite, Vite React plugin | Build tooling |
| | Vitest | Frontend unit tests (D-7) |
| | Type definitions for React and Three.js | Type checking |

**Explicitly excluded:** NumPy, SciPy, pandas (in the core); databases and ORMs; Redis, Celery, message brokers; Socket.IO; Redux, Zustand, MobX; react-three-fiber, drei; UI component frameworks; Docker (may be added later for packaging only).

## 12. Risk register

Likelihood / impact: H, M, L.

| ID | Risk | L / I | Mitigation |
|---|---|---|---|
| R-01 | **Truth leakage.** Avionics or dashboard ends up showing truth labelled as telemetry, which makes the estimator untestable and the lab misleading. | M / H | Separate types; import-rule test; a test with a deliberately biased sensor asserts telemetry follows the sensor, not truth. |
| R-02 | **Accelerometer semantics.** Modelling kinematic acceleration instead of specific force (classic bug: 0 on the pad, −g in free fall). | H / H | FR-SEN-07 and tests for pad, powered and drag-free coast cases. |
| R-03 | **Hidden non-determinism.** Wall-clock use, float time accumulation, shared RNG, async scheduling order. | M / H | Integer ticks; per-stream RNG; pure core; bit-identity test in CI. |
| R-04 | **Event detection on noisy data.** False or late apogee; false liftoff from noise. | M / M | Debounce, lock-outs, multi-seed Monte Carlo test with tolerances fixed before the first run. |
| R-05 | **Real-time pacing in Python.** Event-loop stalls and garbage-collection pauses cause telemetry jitter. | M / L | Timestamps are sim time, so jitter is cosmetic; sliced execution; jitter measured, not assumed. |
| R-06 | **1-DOF versus gyro and 3D view.** The requested model gives them no physical content. | H / M | Decision D-1 (scripted roll, clearly labelled). |
| R-07 | **Model used outside validity.** Constant C_d near Mach 1; altitude above 20 km. | L / M | `VALIDITY` warning, `MODEL_RANGE` stop, limits documented. |
| R-08 | **Frontend is the largest work package.** Three technologies (React, TypeScript, Three.js), plus Three.js resource lifecycle inside React and 20 Hz updates. | H / M | Pure tested store first; 2D dashboard before 3D; scene kept imperative; 3D is the last feature built. |
| R-09 | **Schema drift** between Python and TypeScript. | M / M | One JSON Schema exported from the Pydantic models; backend-generated golden messages are the frontend test fixtures. |
| R-10 | **CSV loses meaning.** No types, units or metadata; float rounding. | M / M | Manifest with units and categories; round-trip float format; checksums. |
| R-11 | **Sequence numbers are decorative over TCP.** | H / L | Decision D-4 (seeded link impairment) plus slow-client drop counting. |
| R-12 | **Fictional numbers read as real; simulated data read as measured.** | M / H | `fictional: true` in scenarios, `data_origin` in every message and file, permanent UI banner, wording rules in documentation. |
| R-13 | **Tests that prove nothing.** Tolerances tuned until green; golden files that freeze a bug. | M / H | Analytic oracles first; tolerances fixed in the plan; golden files regenerated only by a reviewed command after oracle tests pass. |
| R-14 | **JSON number pitfalls.** Python emits `NaN`/`Infinity` by default, which is invalid JSON and breaks the browser parser. | M / M | Serializer configured to refuse non-finite values; explicit test. |
| R-15 | **Scope creep.** 6-DOF, Kalman filters, GNSS, databases. | M / M | Out-of-scope list in SRS §1; changes need a recorded decision. |
| R-16 | **No deployment path if BURNOUT is never detected.** The backup deployment timer starts at detected BURNOUT. If burnout is never detected (for example the accelerometer is invalid for the whole burn), the timer never starts, apogee is never looked for, and the drag device is never commanded. | L / H | **Unresolved safety requirement.** Needs a separate design for a liftoff-anchored emergency timeout. Deliberately not implemented: no arbitrary timeout has been added. |
