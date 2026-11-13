# AEROSAT-01 — F1 Decision Record

| | |
|---|---|
| Document | F1_DECISIONS.md |
| Version | 1.0 |
| Decisions approved | 2026-11-13, by the project owner |
| Scope | F1 only. F2 and R-16 are separate investigations (section 7). |

> All data in this project is simulated. Every number in section 3 is a **reported** figure from an earlier working session. None of it can be regenerated from this repository yet, so none of it is a result.

---

## 1. What F1 is

**F1: barometer/accelerometer disagreement.** A sensor can deliver values that are finite and in range but wrong (bias step, drift, scale error). Sample rejection (saturated or non-finite, TELEMETRY_PROTOCOL section 3.4) does not catch this. The estimator then follows the wrong sensor, which can produce a false APOGEE and a premature drag-device deployment.

F1 asks for a detector that notices when the two sensors disagree by more than their noise explains, and a defined avionics response.

## 2. Approved decisions

| ID | Decision |
|---|---|
| F1-D1 | **The audit lives in the repository** as a script with fixed seeds (proposed location `backend/tools/f1_audit.py`, outside the `aerosat` package so the pure-core import rules are unaffected). Every F1 number must be regenerable by one documented command. |
| F1-D2 | **Deployments during ascent: the target is 0.** The reported 10 are not accepted. Their common cause must be found before any detector policy is adopted. |
| F1-D3 | **Speed bias: the detector threshold grows with estimated speed.** The detector is not switched off at high speed. |
| F1-D4 | **Response to detected disagreement: mark the estimate `UNRELIABLE`.** No new deployment path. The existing rules then apply: an unreliable apogee deploys only through barometric confirmation or the backup timer (`state_machine.py`). |
| F1-D5 | **Pass criteria are fixed before the rerun** (section 4). |

## 3. Evidence status

Reported in an earlier session. Not in the repository. Not reproducible today.

| Reported item | Status |
|---|---|
| Old `dz` denominator omitted a `sqrt(2/5)` factor; statistic corrected | Derivation and code not available. Must be re-derived in writing. |
| Filter innovation spread = 1.084 x barometer noise, matching theory | Unverified |
| Altitude-scale bias on seed 0: appears above 120 m/s, dominates above about 150 m/s | Unverified; single seed. These are two separate claims and are to be measured separately. |
| Policy B replayed at `W = 1 s` on the 5 ms avionics tick, including high-pad cases | Policy B definition not available |
| 3,820 fault runs; 0 false alarms in 1,206 healthy flights | Unverified; fault list and run definitions not available |
| 10 deployments during ascent; "accelerometer rule cut out" | Unverified; meaning of both not available |

Rule: none of these figures may be quoted as a result, in documents or commit messages, until the in-repo audit regenerates it.

## 4. Pass criteria (F1-D5)

Fixed now. Changing one later needs a written reason.

1. **False alarms:** 0 on every healthy flight in the audit set.
2. **Ascent deployments:** 0 across every fault run. "Ascent" is judged against truth: a `DEPLOY_DRAG_DEVICE` command issued before the true apogee tick recorded by the simulator.
3. **Detection rate:** reported per fault type. The minimum acceptable rate per fault type is an open item (section 6) and must be written into this document **before** the first full audit run.
4. **Scope statement:** results are stated only for the enumerated runs. "0 false alarms" is never worded as a guarantee.

## 5. Constraints on the F1 work

- The avionics still consumes sensor samples only. The detector may not read truth (FR-AVI-01). The audit script may read truth, to score the detector.
- The detector's only output is a quality downgrade to `UNRELIABLE` (F1-D4). It never commands deployment and never changes flight state by itself.
- Fixed seeds, integer ticks, no wall clock: same inputs give identical audit output.
- Tolerances and thresholds are set from a written noise derivation, not tuned until the audit passes (TEST_PLAN section 3.2).
- No avionics code changes until the audit can reproduce the baseline behaviour of the current code.

## 6. Open items, in work order

1. **Recover or rebuild the audit.** Search for the earlier session's script and outputs. If found, bring them into the repository unchanged, then check them. If not, rebuild from this record.
2. **Write the definitions:** `dD`, `dz`, the filter innovation, and the corrected disagreement statistic, each with units, and the `sqrt(2/5)` derivation.
3. **Define the audit set:** fault types, magnitudes, onset times, seeds, healthy-flight set, high-pad cases.
4. **Baseline run on current code** (no detector): which fault runs deploy during ascent today, by fault type, seed, time, and deployment source (`DETECTED`, `BAROMETRIC_CONFIRMATION`, `BACKUP_TIMER`).
5. **Root cause of ascent deployments (F1-D2).** Hypothesis to test, not a finding: barometric confirmation trusts the barometer, so a barometer fault that lowers the reading during ascent could both cause an unreliable apogee and confirm it.
6. **Set the minimum detection rate per fault type** (section 4, item 3).
7. Define Policy B in writing, implement the detector with a speed-dependent threshold (F1-D3), rerun the audit, compare against section 4.
8. Close F1 with a short note: what changed, regenerated results, known limits.

## 6a. Item 1: recovered material

The earlier session's scripts and outputs were found in that session's temporary scratch folder (dated 2026-11-13) and copied unchanged to `backend/tools/f1_recovered/` (25 files, SHA-256 identical to the originals). Known properties, checked by reading, not by rerunning:

- They hard-code absolute paths to this machine and to the scratch folder, and import the test harness `tests/unit/avionics/conftest.py`. They do not run from the repository as they stand.
- Two scripts were edited after some of their outputs were written: `f1_lib.py` (new fault kinds added after `f1_faults.json`) and `f1f_run.py` (normalised disagreement added after `f1f_results.json`; `f1f2_results.json` is the later run). The edits add options; whether they change earlier behaviour is not verified.
- Most of their fault runs disabled the drag device and replayed deployment policies offline. They are therefore not closed-loop results of the current code.

They are evidence of what was done, not results. The audit of record is `backend/tools/f1_audit.py`.

## 6b. Item 2: definitions

Symbols, with units. `k` indexes valid barometer samples (50 Hz, 20 ms apart). REF-01 sensor values are quoted from `scenarios/ref-01.json`; they are inputs, not results.

| Symbol | Unit | Definition |
|---|---|---|
| `h_b,k` | m | Barometric altitude above the pad from sample `k`, by the avionics pressure law: `h = (T_ref/L)(1 - (p/p_ref)^(R L/g0))` (`estimator.py`). |
| `sigma_p` | Pa | Per-sample barometer noise including quantisation: `sigma_p = sqrt(sigma_n^2 + q^2/12)`. REF-01: `sigma_n = 3.0 Pa`, `q = 1.0 Pa`. |
| `dh/dp` | m/Pa | Slope of the pressure law at the measured pressure: `(T_ref/L)(R L/g0)(p/p_ref)^(R L/g0 - 1)/p_ref`, taken as a magnitude. |
| `sigma_h,k` | m | Barometric altitude noise of sample `k`: `sigma_p * dh/dp(p_k)`. Grows as air density falls. |
| `r_k` | m | **Filter innovation**: `h_b,k` minus the filter altitude after the prediction step and before the correction (`residual_m` in `ComplementaryFilter.correct`). |
| `q_I,k` | 1 | Normalised innovation: `abs(r_k) / sigma_h,k`. Its expected spread `s_I` (in units of `sigma_h`) follows from the steady-state covariance of the fixed-gain filter (gains `K_h`, `K_v`, 20 ms correction period, four 5 ms predictions driven by accelerometer noise). `s_I` is to be computed by the audit, not assumed. |
| `h_DR(t)`, `v_DR(t)` | m, m/s | Accelerometer-only dead reckoning. Starts at the end of calibration with `h = v = 0`. Every 5 ms: `a = (f_z - b_a) - g0`, `h += v dt + a dt^2/2`, `v += a dt`, the last valid acceleration is held for at most 100 ms; after that the dead reckoning is invalid for the rest of the flight. It never uses the barometer. |
| `D_k` | m | `h_b,k - h_DR(t_k)`. |
| `dD_k` | m | Mean of `D` over samples `k .. k-4`, minus mean of `D` over samples `k-10 .. k-14`. Block centres are 200 ms apart. Needs 15 contiguous valid samples (20 ms spacing) and valid dead reckoning. |
| `dz_k` | 1 | **Historical statistic, superseded:** `abs(dD_k) / sigma_h,k`. Its denominator is not the spread of `dD` (see below). |
| `q_D,k` | 1 | **Corrected disagreement statistic** (below). Not called a z-score: it is not shown to be unit-normal. |

**Derivation of the `sqrt(2/5)` factor.** In the sensor model each barometer sample carries independent noise of spread `sigma_h` (the bias is constant and cancels in a difference). The mean of 5 independent samples has spread `sigma_h / sqrt(5)`. The two blocks in `dD` share no samples, so their noise is independent and the variances add: `Var(dD) = 2 * sigma_h^2 / 5`, spread `sqrt(2/5) * sigma_h`. Dividing `dD` by `sigma_h` alone (`dz`) therefore yields a statistic whose noise spread is `sqrt(2/5)`, not 1.

**Other contributions to `dD`, to be measured by the audit:**

- Dead reckoning: velocity error spread `sigma_v,DR(t)^2 = (sigma_a t / sqrt(N_cal))^2 + sigma_a^2 dt t` (calibration bias error and integrated accelerometer noise; `N_cal` accelerometer samples in calibration, `t` time since calibration). It contributes `sigma_DR = 0.2 s * sigma_v,DR(t)`.
- Altitude scale error: an error `dT` in the reference temperature scales barometric altitude by `kappa = dT/T_ref`, so over 0.2 s it adds about `kappa * abs(v) * 0.2 s`. This grows with speed and is the reason for F1-D3.
- Held acceleration (up to 100 ms) is not modelled; the audit reports it if it occurs.
- Successive `dD_k` overlap and are correlated; this affects counts of consecutive alarms, not the per-sample spread.

**Corrected statistic:**

`q_D,k = abs(dD_k) / sqrt( (2/5) sigma_h,k^2 + sigma_DR(t_k)^2 + (kappa * abs(v_hat,k) * 0.2 s)^2 )`

with `v_hat` the estimated vertical speed (avionics data only, FR-AVI-01). An alarm threshold on `q_D` is equivalent to a threshold on `abs(dD)` that grows with estimated speed, as F1-D3 requires. `kappa` has to become an avionics configuration value; its value is an open decision.

## 6c. Item 3: audit set

All scenarios are built in memory from `scenarios/ref-01.json`; no scenario file is changed. `B` and `A` are the true burnout and apogee ticks of each scenario, computed at audit start from a sensor-free truth run. Fault onsets are rounded to the 20 ms barometer grid.

**Scenarios**

| ID | Change from REF-01 | Backup timer |
|---|---|---|
| `ref01` | none | REF-01 values |
| `zero_drag` | `drag_coefficient = 0` | expected coast 23665 ms, delay 26000 ms |
| `low_thrust` | thrust 280 N from 0.1 s to 16.0 s, zero at 16.2 s | expected coast 8956 ms, delay 11000 ms |
| `high4k` | pad altitude 4000 m MSL | expected coast `A - B`, delay `A - B + 2000` ms |
| `high8k` | pad altitude 8000 m MSL | as `high4k` |

The timer values for the derived scenarios are test settings for this audit, not proposed configuration.

**Onset times:** `early = B + 1800 ms`, `mid = (A + B)/2`, `late = A - 3000 ms`, `near = A - 1500 ms`.

**Fault cases (122 per scenario for `ref01`, `zero_drag`, `low_thrust`)**

| Group | Cases |
|---|---|
| Barometer step (altitude offset) | -5, -10, -20, -30, -50, -100, -200 m at early, mid, late, near (28) |
| Barometer drift | -2, -5, -10, -20, -50, -100, -200 m/s at early, mid, late, near (28) |
| Barometer frozen | at mid, late, `A-2 s`, `A-1 s`, `A-0.5 s`, `A` (6) |
| False peak | +60 m for 60 ms, +150 m for 200 ms, +500 m for 1 s; at mid and `A-2 s` (6) |
| Barometer invalid (saturated flag) | mid to `A+3 s`; late to `A-1 s` (2) |
| Accelerometer | stuck at 0, 9.80665, 80 m/s^2 and held at last value from `B-1 s`; stuck at 0, 9.80665, held, and invalid from mid (8) |
| Recovery | step -100 m at mid for 20, 40, 60, 100, 300, 1000 ms; drift -50 m/s at mid for 200, 1000 ms; step -30 m at near for 40, 100, 300 ms; frozen at `A-2 s` for 500 ms (12) |
| New fault during the deployment delay | at `A+0.5 s`: step -30 m, step +30 m, drift -50 m/s, accelerometer stuck at 0, barometer frozen (5) |
| Accelerometer invalid from `B+1 s`, then drift | -5, -10, -20, -50 m/s at mid and late (8) |
| Simultaneous | drift -20 m/s + accel invalid; drift -50 m/s + accel invalid; frozen + accel stuck 0; step -100 m + accel invalid; drift -20 m/s + accel stuck 9.80665; frozen + accel held; all at mid; barometer invalid + accel stuck 0 at late (7) |
| Single-sample spikes | +5000 Pa at `B-1.2 s`, mid, `A-2 s`, `A+0.5 s`; -5 m and +5 m at mid, `A-2 s`, `A+0.5 s`; -5 m on two samples at mid; -20 m at mid (12) |

**High-pad fault cases (`high4k`, `high8k`):** step -100 m at mid; step -30 m at near; drift -20 and -50 m/s at mid; frozen at `A-2 s`; accelerometer stuck at 0 from `B-1 s`; +5000 Pa spike at mid; -5 m spike at mid (8 each).

**Seeds:** fault runs use seeds 0-9 for every case: 122 x 3 x 10 + 8 x 2 x 10 = 3,820 runs by construction.

**Healthy flights:** `ref01`, `zero_drag`, `low_thrust` seeds 0-299 (0-99 training, 100-299 held out); one ideal-sensor flight per scenario (5); pad-only `ref01` (ignition moved to 120 s, 60 s run) seeds 0-99 plus one ideal; `high4k` and `high8k` seeds 0-99. 1,206 flights by construction. High-pad flights are never used to set thresholds.

**Scoring (section 4, item 2):** a run deploys during ascent when its `DEPLOY_DRAG_DEVICE` command tick is earlier than the true apogee tick recorded by the simulator in the same closed-loop run. Baseline runs keep the drag device enabled and run the unchanged avionics.

## 7. Out of scope here

- **F2:** frozen barometer causes false TOUCHDOWN/LANDED while airborne. Investigated separately, after F1.
- **R-16:** BURNOUT never detected, so the backup timer never arms (ARCHITECTURE section 12).

F1 evidence is not to be reused as evidence for either.
