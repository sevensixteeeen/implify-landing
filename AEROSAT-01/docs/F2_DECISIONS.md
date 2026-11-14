# AEROSAT-01 — F2 Decision Record

| | |
|---|---|
| Document | F2_DECISIONS.md |
| Version | 1.0 |
| Decisions approved | Sections 2 and 4: 2026-11-13, by the project owner, as proposed (F2-D4: options A and C; B recorded as a later option). |
| Date | 2026-11-13 |
| Scope | F2 only. F1 is closed (F1_DECISIONS.md, section 6g) and its results must not change. |

> All data in this project is simulated. Section 3 separates reported figures, exploratory figures and results. Only the audit of record (section 6) produces results.

---

## 1. What F2 is

**F2: false TOUCHDOWN / LANDED while airborne.** When the barometer freezes (keeps reporting the same reading), the avionics can declare TOUCHDOWN and enter LANDED while the vehicle is still descending.

Required outcome: **the flight computer never declares LANDED because a frozen barometer makes altitude or velocity appear stable.**

Questions F2 must answer:

1. Why frozen barometer readings satisfy the current touchdown criteria.
2. Whether touchdown needs independent evidence from more than one sensor.
3. How to distinguish ground contact from a sensor freeze.
4. Whether LANDED must be inhibited when the sensor evidence is unreliable.
5. How to test frozen, drifting, stuck and invalid sensor conditions.
6. How to ensure recovery from a temporary sensor fault does not create a false landing.

## 2. Decisions

Approved by the project owner on 2026-11-13, as proposed.

| ID | Proposed decision |
|---|---|
| F2-D1 | **The audit lives in the repository** (`backend/tools/f2_audit.py`), fixed seeds, every number regenerable by one command. Every run is flown closed-loop **until the true touchdown plus the post-touchdown hold**, not stopped after deployment. |
| F2-D2 | **Target: 0 false TOUCHDOWN events.** A false TOUCHDOWN is a TOUCHDOWN event whose tick is earlier than the true touchdown tick recorded by the simulator in the same run. Applies to every run, healthy and fault. |
| F2-D3 | **Safety asymmetry.** A false LANDED is the hazard; a late or missing LANDED in a fault run is a lesser failure. Late or missing TOUCHDOWN in fault runs is reported, with no minimum. Healthy flights must still detect TOUCHDOWN within the existing tolerance (0 to +3 s after true touchdown, TEST_PLAN EVT tolerances). |
| F2-D4 | **Evidence for touchdown: A (barometer liveness) + C (timing plausibility)** (section 5). B (simulator impact model) is recorded as a later option, not part of F2. |
| F2-D5 | **F1 is preserved.** The F1 audit (`tools/f1_audit.py rerun`) must give results identical to F1 section 6g after F2 is implemented. Deployment behaviour does not change. |
| F2-D6 | **Pass criteria are fixed before the rerun** (section 4), as in F1. |

## 3. Evidence

### 3.1 Reported (earlier Step 5 review, not reproduced)

- False TOUCHDOWN/LANDED in 18 of 18 drag-enabled runs.
- False TOUCHDOWN/LANDED in 10 of 10 earlier E6 fault-injection runs.

Run definitions and code for these are not in the repository. They are not results.

### 3.2 Exploratory probe on the current code (commit `c774d93`), not the audit of record

REF-01, barometer frozen from the given time to the end of the run, 3 seeds each, flown to the end of the post-touchdown hold. `A` = true apogee.

| Freeze at | TOUCHDOWN declared | Criterion | True altitude AGL at the event | True velocity at the event |
|---|---|---|---|---|
| none (healthy) | about 2.5 s after true touchdown | primary | 0 m | 0 m/s |
| `A - 1 s` | about 4 s after the freeze | primary | 2,074 m | -14.3 m/s |
| `A + 3 s` | about 4 s after the freeze | primary | 2,060 m | -14.0 m/s |
| `A + 10 s` | about 4 s after the freeze | primary | 1,962 m | -13.9 m/s |
| `A + 30 s` | about 4 s after the freeze | primary | 1,685 m | -13.7 m/s |
| `A + 60 s` | about 4 s after the freeze | primary | 1,277 m | -13.5 m/s |
| `A + 100 s` | about 4 s after the freeze | primary | 746 m | -13.1 m/s |
| `A + 140 s` | about 4 s after the freeze | primary | 227 m | -12.8 m/s |

All 21 frozen runs declared a false TOUCHDOWN. The audit of record must regenerate this as its baseline.

## 4. Pass criteria (F2-D6)

1. **False TOUCHDOWN:** 0 across every run of the audit set (F2-D2).
2. **Healthy touchdown:** every healthy flight declares TOUCHDOWN within 0 to +3 s of true touchdown.
3. **Fault runs, late or missing TOUCHDOWN:** reported per fault type, no minimum (F2-D3).
4. **F1 preserved:** F1 audit rerun identical to F1 section 6g (F2-D5).
5. **Scope:** results apply only to the enumerated runs. Never worded as a guarantee.

## 5. Mechanism (question 1) and evidence options (questions 2 and 3)

### 5.1 Why a frozen barometer satisfies the touchdown criteria

From the code (`state_machine.py`) and the simulator (`simulator.py`):

- **Primary criterion:** over 2 s, the altitude estimate stays within 3 m and the velocity estimate stays below 2 m/s. With a frozen barometer, every correction pulls the altitude estimate back to the same value, which drives the velocity estimate to zero. The criterion is met about 4 s after the freeze. This is the path seen in every probe run.
- **Fallback criterion:** barometric altitude within 3 m for 2.8 s and estimated acceleration below 2 m/s². A frozen barometer satisfies the first part by construction.
- **The accelerometer cannot tell.** In steady descent under the drag device, drag balances weight and the accelerometer reads +g. On the ground it also reads +g.
- **The simulator has no impact pulse.** At ground contact it sets velocity to zero within one tick and reports zero acceleration from that tick. Checked on REF-01 with the drag device deployed: true specific force stays within 0.008 m/s² of g in steady descent, well below the accelerometer noise of 0.05 m/s², and is g on the ground with no spike between. The accelerometer sees no landing signature.

Consequence: in this simulator, **after the drag device has slowed the descent, the only sensor that can distinguish descent from ground is the barometer.** A frozen barometer removes that evidence while still looking valid (finite, not saturated).

### 5.2 Evidence options

| Option | What it adds | Catches | Does not catch | Cost |
|---|---|---|---|---|
| **A. Barometer liveness** | Reject the barometer as `STALE` when its readings repeat beyond what its own noise allows. Threshold derived from the noise model, as in F1. A live barometer on the ground still shows noise, so real landings are unaffected. | Frozen and stuck barometer | A barometer that keeps its noise but drifts upward at the descent rate (looks still, looks alive) | Avionics only. Only valid when the configured noise is well above the resolution; must be checked at configuration load. |
| **B. Impact model in the simulator** | A short deceleration pulse at ground contact, so the accelerometer sees landing. Touchdown then requires an impact signature. | Every barometer fault | Soft landings below the pulse threshold | Changes the truth model: FR-SIM-06, every touchdown test and the F1 audit physics. Separate decision. |
| **C. Timing plausibility** | TOUCHDOWN not accepted before the earliest physically possible landing time, from the highest altitude seen and the fastest plausible descent rate. | Any fault that makes landing appear early, including drift | A fault that also corrupts the altitude used for the bound | Avionics only. Bound must be derived in writing. |

**Rejected option: run the F1 detector in descent.** With no impact pulse, after a real landing the accelerometer dead reckoning keeps "falling" at the touchdown speed while the barometer stops. The detector would flag every real landing.

### 5.3 Known conflict to resolve in the design (question 4)

F1 latches the estimate `UNRELIABLE` for the rest of the flight after a disagreement: 3,306 of 3,820 F1 fault runs. "Never declare LANDED from an unreliable estimate" would mean those flights never land. The design must say which evidence allows LANDED for a flight whose estimate is `UNRELIABLE`, for example a live barometer (A) plus timing plausibility (C).

## 6. Open items, in work order

1. **Approve section 2 and section 4.** Done, 2026-11-13.
2. **Define the audit set:** scenarios, fault types, onsets, seeds, healthy set. Must cover, at minimum (question 5):
   - frozen barometer at onsets through ascent, near apogee, during the deployment delay, and through descent to landing;
   - stuck barometer at a value different from the true one;
   - barometer drift in descent, including upward drifts near the descent rate (the adversarial case for option A);
   - barometer invalid (saturated flag) through landing;
   - temporary freezes that recover (question 6);
   - accelerometer faults in descent;
   - flights where F1 has latched `UNRELIABLE`;
   - drag device disabled (ballistic descent);
   - every F1 audit scenario.
3. **Baseline on the current code:** every false TOUCHDOWN by fault type, onset, seed, criterion (primary or fallback), true altitude and velocity at the event.
4. **Design (before code):** the chosen evidence options, thresholds from written derivations, the rule for `UNRELIABLE` flights (5.3), recovery behaviour, tests. Stop for review.
5. Implement, test, rerun the F2 audit and the F1 audit, compare against section 4.
6. Close F2 with a short note: what changed, regenerated results, known limits.

## 6a. Item 2: audit set

All scenarios are built in memory from `scenarios/ref-01.json`, exactly as in F1_DECISIONS 6c (same timer settings and `detector_kappa` per scenario), plus one ballistic variant. No scenario file is changed. Every run is flown closed-loop, drag device as configured, **until the simulator finishes: true touchdown plus the 5 s post-touchdown hold** (F2-D1), or the 600 s maximum duration.

**Scenarios:** `ref01`, `zero_drag`, `low_thrust`, `high4k`, `high8k` (every F1 audit scenario), and `ref01_nodrag`: REF-01 with `vehicle.drag_device.enabled = false` (ballistic descent; the avionics then commands nothing).

**Reference times** (per scenario, from a sensor-free truth run in which the drag device is commanded at `A + 1000 ms`, the nominal normal-path time; for `ref01_nodrag` no command): `B` true burnout, `A` true apogee, `TD` true touchdown tick, `v_d` = abs(true velocity) on the last tick before touchdown. Descent fraction `D(f) = A + f (TD - A)`. Onsets are rounded to the 20 ms barometer grid.

**Fault cases** (for each of the five drag-device scenarios):

| Group | Cases | Count |
|---|---|---|
| Barometer frozen (to the end) | at `(A+B)/2` (ascent), `A - 1 s` (near apogee), `A + 0.5 s` (deployment delay), `D(0.1)`, `D(0.25)`, `D(0.5)`, `D(0.75)`, `D(0.9)`, `D(0.98)` | 9 |
| Barometer stuck at a wrong value (to the end) | reading jumps to the altitude at onset +200 m and -200 m, held, at `D(0.25)` and `D(0.75)`; reading held at the pad reference pressure (0 m AGL) at `D(0.5)` | 5 |
| Barometer drift in descent (to the end) | upward at `0.5 v_d`, `1.0 v_d` (adversarial for option A: looks still), `1.5 v_d`; downward at 5 and 20 m/s; each at `D(0.25)` and `D(0.75)` | 10 |
| Barometer invalid (saturated flag) through landing | from `D(0.25)` and from `D(0.75)` to the end | 2 |
| Temporary freeze that recovers (question 6) | at `D(0.25)` for 1, 3, 5, 10 s; at `D(0.9)` for 3 s | 5 |
| Accelerometer in descent (to the end) | at `D(0.5)`: stuck at 0, stuck at 9.80665 (equal to truth in steady descent), stuck at 20 m/s², held at last value, invalid | 5 |
| F1 has latched `UNRELIABLE` | accelerometer invalid for 200 ms at `(A+B)/2` (`UNCHECKABLE`); barometer step -100 m for 100 ms at `(A+B)/2` (`DISAGREEMENT`); the same step plus barometer frozen at `D(0.5)` | 3 |

39 cases per drag-device scenario.

**`ref01_nodrag` fault cases:** barometer frozen at `A - 1 s`, `A + 3 s`, `D(0.5)`; barometer invalid from `D(0.5)` (4 cases).

**Seeds:** every fault case uses seeds 0-9: (39 x 5 + 4) x 10 = **1,990 fault runs** by construction.

**Healthy flights:** each of the six scenarios seeds 0-49 (300 flights), plus one ideal-sensor flight per scenario (6): **306 healthy flights** by construction.

**Scoring:**
- A **false TOUCHDOWN** is a TOUCHDOWN event with a tick earlier than the true touchdown tick of the same run (F2-D2).
- **Criterion** of a TOUCHDOWN: `primary` for source `DETECTED`, `fallback` for `FALLBACK_CRITERION`.
- Recorded at the TOUCHDOWN event: true altitude AGL and true vertical velocity on that tick.
- Healthy flights: TOUCHDOWN delay = event time minus true touchdown time (seconds, interpolated true contact time), required within 0 to +3 s (section 4, item 2).
- Fault runs without a false TOUCHDOWN: TOUCHDOWN `on time` (0 to +3 s), `late` (> +3 s) or `missing` (none by the end of the run), reported per fault type (section 4, item 3).

## 6b. Item 3: baseline results

Generated by `uv run python tools/f2_audit.py baseline` (run from `backend/`), on the current code (F1 closed, commit `971c68a`), audit set of 6a. Full tables: `backend/tools/f2_out/baseline_summary.md`; per run: `baseline_runs.csv`. A rerun of every 37th run (63 runs) reproduced its rows exactly. Results apply only to these runs.

**Independent verification** (2026-11-14, by the project owner): all counts in this section were recounted from `baseline_runs.csv`, and 44 runs (40 random plus 4 key cases) were reproduced with 0 mismatches.

**Reference times** (ticks; `v_d` m/s): `ref01` B 6200, A 24247, TD 184661, v_d 12.7; `zero_drag` 6200, 29865, 258148, 12.7; `low_thrust` 19200, 28156, 113317, 12.7; `high4k` 6200, 25677, 170123, 15.5; `high8k` 6200, 26963, 152216, 19.3; `ref01_nodrag` 6200, 24247, 46431, 168.8.

| Section 4 criterion | Baseline result |
|---|---|
| 1. False TOUCHDOWN | **1,060 of 2,296 runs** (1,060 of 1,990 fault runs; 0 of 306 healthy flights) |
| 2. Healthy TOUCHDOWN within 0 to +3 s | **299 of 306.** Not met today: 7 `high8k` healthy flights declare TOUCHDOWN late (seeds 2, 7, 8, 16, 22, 33, 43: +4.716, +4.117, +3.005, +3.042, +3.317, +3.937, +3.117 s). Delay over all healthy flights: +2.485 to +4.716 s, median +2.591 s. |
| 3. Fault runs, late or missing TOUCHDOWN | Table below |
| 4. F1 preserved | Not applicable to the baseline |

No run issued more than one deployment command or repeated an event type.

**Outcome per fault group** (fault runs):

| Group | Runs | False | On time | Late | Missing |
|---|---|---|---|---|---|
| baro frozen | 480 | 460 | 20 | 0 | 0 |
| baro stuck | 250 | 250 | 0 | 0 | 0 |
| baro drift | 500 | 100 | 0 | 0 | 400 |
| baro invalid | 110 | 0 | 0 | 0 | 110 |
| temporary freeze | 250 | 200 | 47 | 3 | 0 |
| accelerometer in descent | 250 | 0 | 140 | 10 | 100 |
| F1 latched | 150 | 50 | 98 | 2 | 0 |

**Findings** (observations from these runs; causes are for item 4 to confirm):

- **Every frozen or stuck barometer that persists more than about 3 s before landing gives a false TOUCHDOWN**, in every scenario: frozen 460 of 480, stuck 250 of 250. A freeze in descent is followed by TOUCHDOWN 2.50 to 2.70 s later (primary criterion; 6 of the 460 frozen events used the fallback); a freeze before or near apogee is declared once the flight reaches DESCENT (3.0 to 16.8 s after the freeze). A reading stuck at a wrong value is followed by TOUCHDOWN exactly 2.80 s later (fallback criterion, all 250). With the drag device the vehicle was 3.8 to 3,030 m above ground and descending at 12.7 to 22.2 m/s; ballistic (`ref01_nodrag`) at about 460 m and -156 m/s.
- The 20 frozen runs without a false TOUCHDOWN froze at `D(0.98)` in `low_thrust` and `high8k`, close enough to the ground to be declared on time.
- **Upward drift at the descent rate (`1.0 v_d`) gives a false TOUCHDOWN in all 100 runs**: the adversarial case for option A. Drift at `0.5 v_d` or `1.5 v_d`, and downward drift, give no false TOUCHDOWN but **no TOUCHDOWN at all** (400 missing).
- **Barometer invalid through landing: TOUCHDOWN missing in all 110 runs.**
- **Temporary freezes of 3 s or more give a false TOUCHDOWN in all 200 runs**, although the barometer recovers; LANDED is latched before recovery. A 1 s freeze gives none (47 on time, 3 late, all late ones `high8k`).
- **Accelerometer faults in descent never give a false TOUCHDOWN.** Stuck at 0 or at 20 m/s²: TOUCHDOWN missing (100 runs). Stuck at 9.80665, held, or invalid: on time or late.
- **Flights with F1 latched `UNRELIABLE` land on time** (or late in `high8k`) when the barometer is healthy. With a later freeze they give a false TOUCHDOWN like any other frozen run (50 of 50).

**Every false TOUCHDOWN.** All 1,060 false TOUCHDOWN events are listed below, grouped by scenario and case. In every affected case all 10 seeds (0-9) give a false TOUCHDOWN, so each row covers 10 events. The values are ranges over the 10 seeds. The individual events (tick, seed, criterion, true altitude and velocity per event) are in the last table of `backend/tools/f2_out/baseline_summary.md`. Onset is in simulation seconds; "TOUCHDOWN after onset" is measured from the freeze for the F1-latched case.
| Scenario | Fault type | Case | Onset (s) | Seeds | Criterion | TOUCHDOWN after onset (s) | True altitude at TOUCHDOWN (m) | True velocity at TOUCHDOWN (m/s) |
|---|---|---|---|---|---|---|---|---|
| ref01 | baro frozen | frozen @ascent | 15.22 | 0-9 | primary 10 | 13.08 | 2,074.0 | -14.26 |
| ref01 | baro frozen | frozen @A-1s | 23.24 | 0-9 | primary 10 | 5.06 | 2,074.0 | -14.26 |
| ref01 | baro frozen | frozen @A+0.5s | 24.74 | 0-9 | primary 10 | 3.56 | 2,074.0 | -14.26 |
| ref01 | baro frozen | frozen @D(0.1) | 40.28 | 0-9 | primary 10 | 2.50 to 2.60 | 1,877.6 to 1,878.9 | -13.88 |
| ref01 | baro frozen | frozen @D(0.25) | 64.36 | 0-9 | primary 10 | 2.52 to 2.58 | 1,546.3 to 1,547.5 | -13.65 |
| ref01 | baro frozen | frozen @D(0.5) | 104.46 | 0-9 | primary 10 | 2.50 to 2.58 | 1,006.3 to 1,007.4 | -13.29 |
| ref01 | baro frozen | frozen @D(0.75) | 144.56 | 0-9 | primary 10 | 2.50 to 2.56 | 480.4 to 481.2 | -12.96 to -12.95 |
| ref01 | baro frozen | frozen @D(0.9) | 168.62 | 0-9 | fallback 1, primary 9 | 2.52 to 2.56 | 171.2 to 171.8 | -12.76 |
| ref01 | baro frozen | frozen @D(0.98) | 181.46 | 0-9 | primary 10 | 2.52 to 2.58 | 7.7 to 8.6 | -12.66 |
| ref01 | baro stuck | stuck at altitude +200 m @D(0.25) | 64.36 | 0-9 | fallback 10 | 2.80 | 1,543.3 to 1,543.7 | -13.65 |
| ref01 | baro stuck | stuck at altitude -200 m @D(0.25) | 64.36 | 0-9 | fallback 10 | 2.80 | 1,543.3 to 1,543.7 | -13.65 |
| ref01 | baro stuck | stuck at altitude +200 m @D(0.75) | 144.56 | 0-9 | fallback 10 | 2.80 | 477.3 to 477.6 | -12.95 |
| ref01 | baro stuck | stuck at altitude -200 m @D(0.75) | 144.56 | 0-9 | fallback 10 | 2.80 | 477.3 to 477.6 | -12.95 |
| ref01 | baro stuck | stuck at pad pressure @D(0.5) | 104.46 | 0-9 | fallback 10 | 2.80 | 1,003.3 to 1,003.7 | -13.29 |
| ref01 | baro drift | drift up 1.0 v_d (12.7 m/s) @D(0.25) | 64.36 | 0-9 | primary 10 | 2.64 to 3.96 | 1,527.5 to 1,545.6 | -13.65 to -13.64 |
| ref01 | baro drift | drift up 1.0 v_d (12.7 m/s) @D(0.75) | 144.56 | 0-9 | primary 10 | 2.52 to 2.64 | 479.5 to 481.2 | -12.96 to -12.95 |
| ref01 | temporary freeze | frozen @D(0.25) for 3 s | 64.36 | 0-9 | primary 10 | 2.52 to 2.58 | 1,546.3 to 1,547.5 | -13.65 |
| ref01 | temporary freeze | frozen @D(0.25) for 5 s | 64.36 | 0-9 | primary 10 | 2.52 to 2.58 | 1,546.3 to 1,547.5 | -13.65 |
| ref01 | temporary freeze | frozen @D(0.25) for 10 s | 64.36 | 0-9 | primary 10 | 2.52 to 2.58 | 1,546.3 to 1,547.5 | -13.65 |
| ref01 | temporary freeze | frozen @D(0.9) for 3 s | 168.62 | 0-9 | fallback 1, primary 9 | 2.52 to 2.56 | 171.2 to 171.8 | -12.76 |
| ref01 | F1 latched | step -100 m 100 ms @ascent + frozen @D(0.5) | 15.22 (step), 104.46 (freeze) | 0-9 | primary 10 | 2.50 to 2.58 | 999.6 to 1,000.7 | -13.29 |
| zero_drag | baro frozen | frozen @ascent | 18.04 | 0-9 | primary 10 | 16.82 to 16.84 | 3,029.7 to 3,030.0 | -14.93 to -14.92 |
| zero_drag | baro frozen | frozen @A-1s | 28.86 | 0-9 | primary 10 | 6.00 to 6.02 | 3,029.7 to 3,030.0 | -14.93 to -14.92 |
| zero_drag | baro frozen | frozen @A+0.5s | 30.36 | 0-9 | primary 10 | 4.50 to 4.52 | 3,029.7 to 3,030.0 | -14.93 to -14.92 |
| zero_drag | baro frozen | frozen @D(0.1) | 52.70 | 0-9 | fallback 1, primary 9 | 2.50 to 2.60 | 2,743.7 to 2,745.0 | -14.50 |
| zero_drag | baro frozen | frozen @D(0.25) | 86.94 | 0-9 | primary 10 | 2.54 to 2.60 | 2,253.3 to 2,254.5 | -14.14 |
| zero_drag | baro frozen | frozen @D(0.5) | 144.00 | 0-9 | primary 10 | 2.50 to 2.58 | 1,462.6 to 1,463.6 | -13.59 |
| zero_drag | baro frozen | frozen @D(0.75) | 201.08 | 0-9 | primary 10 | 2.50 to 2.56 | 701.4 to 702.4 | -13.09 |
| zero_drag | baro frozen | frozen @D(0.9) | 235.32 | 0-9 | primary 10 | 2.52 to 2.56 | 258.0 to 258.6 | -12.82 |
| zero_drag | baro frozen | frozen @D(0.98) | 253.58 | 0-9 | primary 10 | 2.50 to 2.56 | 25.1 to 26.1 | -12.67 |
| zero_drag | baro stuck | stuck at altitude +200 m @D(0.25) | 86.94 | 0-9 | fallback 10 | 2.80 | 2,250.4 to 2,250.8 | -14.14 |
| zero_drag | baro stuck | stuck at altitude -200 m @D(0.25) | 86.94 | 0-9 | fallback 10 | 2.80 | 2,250.4 to 2,250.8 | -14.14 |
| zero_drag | baro stuck | stuck at altitude +200 m @D(0.75) | 201.08 | 0-9 | fallback 10 | 2.80 | 698.2 to 698.6 | -13.09 |
| zero_drag | baro stuck | stuck at altitude -200 m @D(0.75) | 201.08 | 0-9 | fallback 10 | 2.80 | 698.2 to 698.6 | -13.09 |
| zero_drag | baro stuck | stuck at pad pressure @D(0.5) | 144.00 | 0-9 | fallback 10 | 2.80 | 1,459.5 to 1,459.9 | -13.59 |
| zero_drag | baro drift | drift up 1.0 v_d (12.7 m/s) @D(0.25) | 86.94 | 0-9 | primary 10 | 5.52 to 23.56 | 1,959.0 to 2,212.0 | -14.11 to -13.93 |
| zero_drag | baro drift | drift up 1.0 v_d (12.7 m/s) @D(0.75) | 201.08 | 0-9 | primary 10 | 2.56 to 2.66 | 700.2 to 701.5 | -13.09 |
| zero_drag | temporary freeze | frozen @D(0.25) for 3 s | 86.94 | 0-9 | primary 10 | 2.54 to 2.60 | 2,253.3 to 2,254.5 | -14.14 |
| zero_drag | temporary freeze | frozen @D(0.25) for 5 s | 86.94 | 0-9 | primary 10 | 2.54 to 2.60 | 2,253.3 to 2,254.5 | -14.14 |
| zero_drag | temporary freeze | frozen @D(0.25) for 10 s | 86.94 | 0-9 | primary 10 | 2.54 to 2.60 | 2,253.3 to 2,254.5 | -14.14 |
| zero_drag | temporary freeze | frozen @D(0.9) for 3 s | 235.32 | 0-9 | primary 10 | 2.52 to 2.56 | 258.0 to 258.6 | -12.82 |
| zero_drag | F1 latched | step -100 m 100 ms @ascent + frozen @D(0.5) | 18.04 (step), 144.00 (freeze) | 0-9 | primary 10 | 2.50 to 2.58 | 1,451.3 to 1,452.3 | -13.59 |
| low_thrust | baro frozen | frozen @ascent | 23.68 | 0-9 | primary 10 | 8.61 | 1,043.5 | -13.58 |
| low_thrust | baro frozen | frozen @A-1s | 27.16 | 0-9 | primary 10 | 5.13 | 1,043.5 | -13.58 |
| low_thrust | baro frozen | frozen @A+0.5s | 28.66 | 0-9 | primary 10 | 3.63 | 1,043.5 | -13.58 |
| low_thrust | baro frozen | frozen @D(0.1) | 36.68 | 0-9 | primary 10 | 2.52 to 2.58 | 958.9 to 959.9 | -13.26 |
| low_thrust | baro frozen | frozen @D(0.25) | 49.44 | 0-9 | primary 10 | 2.52 to 2.56 | 790.4 to 791.2 | -13.15 |
| low_thrust | baro frozen | frozen @D(0.5) | 70.74 | 0-9 | primary 10 | 2.52 to 2.56 | 512.2 to 513.3 | -12.98 to -12.97 |
| low_thrust | baro frozen | frozen @D(0.75) | 92.02 | 0-9 | primary 10 | 2.52 to 2.56 | 238.2 to 239.0 | -12.80 |
| low_thrust | baro frozen | frozen @D(0.9) | 104.80 | 0-9 | primary 10 | 2.50 to 2.56 | 75.2 to 76.2 | -12.70 |
| low_thrust | baro stuck | stuck at altitude +200 m @D(0.25) | 49.44 | 0-9 | fallback 10 | 2.80 | 787.3 to 787.8 | -13.15 |
| low_thrust | baro stuck | stuck at altitude -200 m @D(0.25) | 49.44 | 0-9 | fallback 10 | 2.80 | 787.3 to 787.8 | -13.15 |
| low_thrust | baro stuck | stuck at altitude +200 m @D(0.75) | 92.02 | 0-9 | fallback 10 | 2.80 | 234.8 to 235.4 | -12.80 |
| low_thrust | baro stuck | stuck at altitude -200 m @D(0.75) | 92.02 | 0-9 | fallback 10 | 2.80 | 234.8 to 235.4 | -12.80 |
| low_thrust | baro stuck | stuck at pad pressure @D(0.5) | 70.74 | 0-9 | fallback 10 | 2.80 | 509.1 to 509.6 | -12.97 |
| low_thrust | baro drift | drift up 1.0 v_d (12.7 m/s) @D(0.25) | 49.44 | 0-9 | primary 10 | 2.58 to 2.76 | 788.1 to 790.7 | -13.15 |
| low_thrust | baro drift | drift up 1.0 v_d (12.7 m/s) @D(0.75) | 92.02 | 0-9 | primary 10 | 2.50 to 2.62 | 237.7 to 239.2 | -12.80 |
| low_thrust | temporary freeze | frozen @D(0.25) for 3 s | 49.44 | 0-9 | primary 10 | 2.52 to 2.56 | 790.4 to 791.2 | -13.15 |
| low_thrust | temporary freeze | frozen @D(0.25) for 5 s | 49.44 | 0-9 | primary 10 | 2.52 to 2.56 | 790.4 to 791.2 | -13.15 |
| low_thrust | temporary freeze | frozen @D(0.25) for 10 s | 49.44 | 0-9 | primary 10 | 2.52 to 2.56 | 790.4 to 791.2 | -13.15 |
| low_thrust | temporary freeze | frozen @D(0.9) for 3 s | 104.80 | 0-9 | primary 10 | 2.50 to 2.56 | 75.2 to 76.2 | -12.70 |
| low_thrust | F1 latched | step -100 m 100 ms @ascent + frozen @D(0.5) | 23.68 (step), 70.74 (freeze) | 0-9 | primary 10 | 2.52 to 2.56 | 504.4 to 504.9 | -12.97 |
| high4k | baro frozen | frozen @ascent | 15.94 | 0-9 | primary 10 | 13.88 | 2,301.0 | -17.74 |
| high4k | baro frozen | frozen @A-1s | 24.68 | 0-9 | primary 10 | 4.56 to 4.58 | 2,318.3 to 2,318.6 | -16.91 to -16.90 |
| high4k | baro frozen | frozen @A+0.5s | 26.18 | 0-9 | primary 10 | 3.02 to 3.06 | 2,317.2 to 2,318.3 | -17.01 to -16.95 |
| high4k | baro frozen | frozen @D(0.1) | 40.12 | 0-9 | primary 10 | 2.54 to 2.66 | 2,082.2 to 2,083.9 | -17.32 to -17.31 |
| high4k | baro frozen | frozen @D(0.25) | 61.78 | 0-9 | primary 10 | 2.56 to 2.62 | 1,711.3 to 1,712.7 | -16.96 |
| high4k | baro frozen | frozen @D(0.5) | 97.90 | 0-9 | primary 10 | 2.56 to 2.62 | 1,108.8 to 1,109.8 | -16.42 to -16.41 |
| high4k | baro frozen | frozen @D(0.75) | 134.02 | 0-9 | primary 10 | 2.56 to 2.62 | 525.0 to 526.3 | -15.91 |
| high4k | baro frozen | frozen @D(0.9) | 155.68 | 0-9 | primary 10 | 2.54 to 2.60 | 183.8 to 184.8 | -15.62 |
| high4k | baro frozen | frozen @D(0.98) | 167.24 | 0-9 | primary 10 | 2.54 to 2.62 | 3.8 to 5.2 | -15.48 |
| high4k | baro stuck | stuck at altitude +200 m @D(0.25) | 61.78 | 0-9 | fallback 10 | 2.80 | 1,708.2 to 1,708.7 | -16.96 |
| high4k | baro stuck | stuck at altitude -200 m @D(0.25) | 61.78 | 0-9 | fallback 10 | 2.80 | 1,708.2 to 1,708.7 | -16.96 |
| high4k | baro stuck | stuck at altitude +200 m @D(0.75) | 134.02 | 0-9 | fallback 10 | 2.80 | 522.1 to 522.5 | -15.91 |
| high4k | baro stuck | stuck at altitude -200 m @D(0.75) | 134.02 | 0-9 | fallback 10 | 2.80 | 522.1 to 522.5 | -15.91 |
| high4k | baro stuck | stuck at pad pressure @D(0.5) | 97.90 | 0-9 | fallback 10 | 2.80 | 1,105.6 to 1,106.1 | -16.41 |
| high4k | baro drift | drift up 1.0 v_d (15.5 m/s) @D(0.25) | 61.78 | 0-9 | primary 10 | 12.22 to 30.90 | 1,237.7 to 1,549.6 | -16.81 to -16.53 |
| high4k | baro drift | drift up 1.0 v_d (15.5 m/s) @D(0.75) | 134.02 | 0-9 | fallback 1, primary 9 | 2.62 to 2.92 | 520.5 to 525.2 | -15.91 to -15.90 |
| high4k | temporary freeze | frozen @D(0.25) for 3 s | 61.78 | 0-9 | primary 10 | 2.56 to 2.62 | 1,711.3 to 1,712.7 | -16.96 |
| high4k | temporary freeze | frozen @D(0.25) for 5 s | 61.78 | 0-9 | primary 10 | 2.56 to 2.62 | 1,711.3 to 1,712.7 | -16.96 |
| high4k | temporary freeze | frozen @D(0.25) for 10 s | 61.78 | 0-9 | primary 10 | 2.56 to 2.62 | 1,711.3 to 1,712.7 | -16.96 |
| high4k | temporary freeze | frozen @D(0.9) for 3 s | 155.68 | 0-9 | primary 10 | 2.54 to 2.60 | 183.8 to 184.8 | -15.62 |
| high4k | F1 latched | step -100 m 100 ms @ascent + frozen @D(0.5) | 15.94 (step), 97.90 (freeze) | 0-9 | primary 10 | 2.56 to 2.62 | 1,102.3 to 1,103.2 | -16.41 |
| high8k | baro frozen | frozen @ascent | 16.58 | 0-9 | primary 10 | 14.62 | 2,510.6 | -22.07 |
| high8k | baro frozen | frozen @A-1s | 25.96 | 0-9 | primary 10 | 4.96 | 2,522.8 to 2,522.9 | -21.10 to -21.09 |
| high8k | baro frozen | frozen @A+0.5s | 27.46 | 0-9 | primary 10 | 3.38 to 3.44 | 2,522.6 to 2,523.4 | -21.15 to -21.11 |
| high8k | baro frozen | frozen @D(0.1) | 39.48 | 0-9 | fallback 1, primary 9 | 2.60 to 2.70 | 2,272.1 to 2,274.5 | -22.15 |
| high8k | baro frozen | frozen @D(0.25) | 58.28 | 0-9 | fallback 1, primary 9 | 2.62 to 2.68 | 1,861.1 to 1,862.8 | -21.59 |
| high8k | baro frozen | frozen @D(0.5) | 89.58 | 0-9 | fallback 1, primary 9 | 2.60 to 2.64 | 1,199.7 to 1,200.8 | -20.74 |
| high8k | baro frozen | frozen @D(0.75) | 120.90 | 0-9 | fallback 1, primary 9 | 2.60 to 2.68 | 561.7 to 563.5 | -19.96 |
| high8k | baro frozen | frozen @D(0.9) | 139.70 | 0-9 | primary 10 | 2.56 to 2.68 | 190.7 to 193.5 | -19.53 |
| high8k | baro stuck | stuck at altitude +200 m @D(0.25) | 58.28 | 0-9 | fallback 10 | 2.80 | 1,858.5 to 1,859.1 | -21.59 |
| high8k | baro stuck | stuck at altitude -200 m @D(0.25) | 58.28 | 0-9 | fallback 10 | 2.80 | 1,858.5 to 1,859.1 | -21.59 |
| high8k | baro stuck | stuck at altitude +200 m @D(0.75) | 120.90 | 0-9 | fallback 10 | 2.80 | 559.4 to 559.9 | -19.96 |
| high8k | baro stuck | stuck at altitude -200 m @D(0.75) | 120.90 | 0-9 | fallback 10 | 2.80 | 559.4 to 559.9 | -19.96 |
| high8k | baro stuck | stuck at pad pressure @D(0.5) | 89.58 | 0-9 | fallback 10 | 2.80 | 1,196.4 to 1,197.0 | -20.73 |
| high8k | baro drift | drift up 1.0 v_d (19.3 m/s) @D(0.25) | 58.28 | 0-9 | primary 10 | 48.60 to 69.98 | 468.6 to 899.0 | -20.37 to -19.85 |
| high8k | baro drift | drift up 1.0 v_d (19.3 m/s) @D(0.75) | 120.90 | 0-9 | fallback 1, primary 9 | 2.80 to 10.30 | 410.7 to 559.7 | -19.96 to -19.78 |
| high8k | temporary freeze | frozen @D(0.25) for 3 s | 58.28 | 0-9 | fallback 1, primary 9 | 2.62 to 2.68 | 1,861.1 to 1,862.8 | -21.59 |
| high8k | temporary freeze | frozen @D(0.25) for 5 s | 58.28 | 0-9 | fallback 1, primary 9 | 2.62 to 2.68 | 1,861.1 to 1,862.8 | -21.59 |
| high8k | temporary freeze | frozen @D(0.25) for 10 s | 58.28 | 0-9 | fallback 1, primary 9 | 2.62 to 2.68 | 1,861.1 to 1,862.8 | -21.59 |
| high8k | temporary freeze | frozen @D(0.9) for 3 s | 139.70 | 0-9 | primary 10 | 2.56 to 2.68 | 190.7 to 193.5 | -19.53 |
| high8k | F1 latched | step -100 m 100 ms @ascent + frozen @D(0.5) | 16.58 (step), 89.58 (freeze) | 0-9 | fallback 1, primary 9 | 2.60 to 2.64 | 1,194.4 to 1,195.2 | -20.73 |
| ref01_nodrag | baro frozen | frozen @A-1s | 23.24 | 0-9 | primary 10 | 20.34 to 20.36 | 460.5 to 463.6 | -156.04 to -155.94 |
| ref01_nodrag | baro frozen | frozen @A+3s | 27.24 | 0-9 | primary 10 | 16.34 to 16.36 | 460.5 to 463.6 | -156.04 to -155.94 |
| ref01_nodrag | baro frozen | frozen @D(0.5) | 35.34 | 0-9 | primary 10 | 8.24 to 8.26 | 460.5 to 463.6 | -156.04 to -155.94 |

## 6c. Item 4: design (before code)

Status: **design for review. No code written.** Values are derived from the noise model, the filter equations and the scenario configuration. Measurements quoted as support come from read-only probes on healthy flights of the current code (scratch probes, not the audit of record); they are marked "probe". Expected outcomes are predictions for the item 5 rerun, not results.

**Constraint from F2-D5 (F1 preserved).** Every F2 rule acts only on the TOUCHDOWN decision in DESCENT. None of them changes the estimator, sample rejection, estimate quality, the F1 detector, or any deployment path. The F1 dead reckoning is not modified; F2 uses its own (6c.3).

### 6c.1 Barometer liveness (option A)

**Rule.** The barometer is `STALE` while its last `N` valid samples are identical (same reading, no rejected sample in between). It is live again on the first valid sample that differs from the previous one. `STALE` is not a rejection: the sample still corrects the estimator (so F1 is unaffected). It only gates touchdown (6c.4).

**Threshold, from the noise model.** For a live barometer, the readings are `Q(p + n)` with `n` Gaussian, spread `sigma_n`, and quantisation step `q`. The chance that `N` consecutive readings fall in the same quantisation bin is largest when the pressure is constant (on the pad or the ground). With `s = sigma_n / q`, the continuous approximation (valid for `s >= 1`) is

`P_same(N) = s^(1-N) * (2 pi)^(-(N-1)/2) * N^(-1/2)`.

- Samples per flight: at most `600 s * 50 Hz = 30,000` (maximum duration).
- Budget, as in F1: false `STALE` at most `1e-5` per flight, so `30,000 * P_same(N) <= 1e-5`.
- REF-01 (`sigma_n = 3.0 Pa`, `q = 1.0 Pa`, `s = 3`): **`N = 12`** (`P_same = 6.6e-11`, `2.0e-6` per flight; `N = 11` would give `1.6e-5`).
- **Detection time:** the 12th identical sample arrives `(N - 1) * 20 ms = 220 ms` after the freeze, well inside the 2.5 s before the baseline's false TOUCHDOWN.
- Probe: the longest run of identical readings in 120 noisy healthy flights (pad, flight and ground, all six scenarios) was 6.

**Configuration check** (at load, `ConfigurationFault` naming the field):

- New avionics fields `baro_noise_sigma_pa` and `baro_resolution_pa`, written by the scenario author from the sensor specification (FR-AVI-01: the avionics does not read the sensors section). REF-01: 3.0 and 1.0.
- `baro_noise_sigma_pa = 0` disables liveness. This is required for ideal-sensor runs: a noise-free barometer repeats its reading on the pad and on the ground (probe: 101 and 250 identical samples), so the rule cannot work there.
- If enabled: `baro_noise_sigma_pa / baro_resolution_pa >= 1` (the derivation's validity), and the derived `N` must give `(N - 1) / f_baro <= 0.5 s`. Examples: `s = 2` gives `N = 14` (260 ms); `s = 1` gives `N = 24` (460 ms) at 50 Hz.
- Cross-check against the scenario, validation only (precedent: the loader already reads `sensors.baro.rate_hz` for the gain check): if `sensors.ideal` is true, liveness must be disabled; if false, the two fields must equal the sensor section's `noise_sigma_pa` and `resolution_pa`.

**Consequence for the audit set.** The six ideal-sensor healthy flights of 6a must run with `baro_noise_sigma_pa = 0`. The list of runs does not change.

**Catches:** frozen and stuck barometer, and temporary freezes longer than 220 ms (960 of the 1,060 baseline false TOUCHDOWNs). **Does not catch:** a barometer that keeps its noise while drifting (the 100 drift-up false TOUCHDOWNs). That needs 6c.3.

### 6c.2 Healthy high8k late touchdown: cause and fix

**Cause, confirmed** (probe, 50 noisy `high8k` flights; 7 late, as in the baseline):

- The primary criterion needs `|v_est| < 2 m/s` on every tick of a 2 s window. On the ground the velocity estimate is pure barometer noise passed through the filter.
- Filter equations: velocity-estimate spread on the ground = `1.11 * sigma_h`.
- `sigma_h` at the pad is `sigma_p * dh/dp(p_ref)`: 0.251 m at sea level, 0.375 m at 4 km, 0.584 m at 8 km.
- At 8 km, then, `sigma_v = 0.65 m/s`, and 2 m/s is only 3.1 sigma. The probe measured `|v_est|` up to 2.77 m/s on the ground.
- In every late flight the window was restarted by `|v_est| >= 2 m/s` between 1.0 and 2.7 s after contact. The altitude-range test was not the cause (its last failure was at the end of the window fill, about 1.9 s, in every flight). Delay = last restart + 2.0 s.

**Fix.** The avionics derives noise floors for the touchdown thresholds at calibration, from `p_ref` and its own `detector_sigma_p_pa` (F1 configuration), and uses the larger of the configured value and the floor:

- `v_thr = max(touchdown_speed_mps, z * 1.11 * sigma_h,pad)`
- `range_thr = max(touchdown_altitude_range_m, 2 * z * 0.399 * sigma_h,pad)` (altitude-estimate spread `0.399 * sigma_h`)
- fallback barometric range `= max(touchdown_altitude_range_m, 2 * z * sigma_h,pad)`

`z` makes a window restart by noise in a healthy landing at most `1e-5` per flight. The union bound is over about 100 independent barometer corrections in the window, so `2 Q(z) <= 1e-7`, giving `z = 5.33`.

| Scenario | `sigma_h,pad` (m) | `v_thr` (m/s) | `range_thr` (m) | Fallback range (m) |
|---|---|---|---|---|
| `ref01`, `zero_drag`, `low_thrust`, `ref01_nodrag` | 0.251 | **2.00** (unchanged) | 3.00 (unchanged) | 3.00 (unchanged) |
| `high4k` | 0.375 | 2.22 | 3.00 | 4.00 |
| `high8k` | 0.584 | **3.46** | 3.00 | 6.23 |

REF-01 is unchanged, so the EVT-13 touchdown ticks are unchanged. Probe check: the largest ground `|v_est|` seen at 8 km (2.77 m/s in 50 flights) is below 3.46. Predicted `high8k` delay: the velocity estimate falls below 3.46 m/s about 0.45 s after contact (from 19.3 m/s), plus the 2.0 s window, about 2.5 s, inside 0 to +3 s.

### 6c.3 Timing plausibility (option C): two variants evaluated

**C2, dead-reckoned remaining altitude (recommended).** A second accelerometer-only dead reckoning, owned by the touchdown logic:

- Same start (calibration end, `h = v = 0`), same samples and 100 ms hold as F1.
- Plus a relative gravity correction: `a = (f_z - b_a) - g0 + 2 g0 h_DR / R0`. Pad gravity is already absorbed by the calibrated bias, so only its change with height above the pad is added.
- Probe: without this correction the error at touchdown was -12 to -453 m, mostly a systematic fall. With it, the ideal-sensor error stays within 2.1 m in every scenario, and the noisy error never exceeded 2.2 `sigma_b(t)` in 60 flights. Trapezoidal integration made no difference.

*Error bound.* The dominant term is the calibrated bias error, constant per flight: `sigma_b(t) = 0.5 * (sigma_a / sqrt(N_cal)) * t^2`, with `t` the time since calibration. Integrated accelerometer noise is smaller (`sigma_a * sqrt(dt t^3 / 3)`, 5 m at 184 s), and the residual model error is below 3 m (probe, ideal sensors). The bound:

`E(t) = 4.42 * sigma_b(t) + 3 m`

4.42 gives `P(|Z| > 4.42) = 1e-5` per flight, the same budget. Values at true touchdown: `ref01` 267 m (t = 184 s), `zero_drag` 520 m, `low_thrust` 102 m, `high4k` 227 m, `high8k` 182 m, `ref01_nodrag` 19 m.

*Validity.* C2 is invalid for the rest of the flight after an accelerometer gap over 100 ms (as F1). It is also invalid if the velocity is implausible:
- After deployment plus a settling time `3 * v_max / g0` (descent with a drag device): `v_DR` must stay in `[-v_max - Ev(t), +Ev(t)]`, with `Ev(t) = 4.42 * (sigma_a / sqrt(N_cal)) * t`.
- From APOGEE (no drag device): the same interval, with the ballistic `v_max`.

This catches an accelerometer stuck at a wrong value. Stuck at 0, `v_DR` grows by `g` per second; stuck at 20 m/s², it turns upward.

*Rule.* A barometer-based TOUCHDOWN is accepted only if `h_DR(t) <= E(t)`. A real landing always passes, because `h_DR = error <= E`.

*Detection floor.* A false TOUCHDOWN at true altitude `h` is blocked with certainty if `h > 2 E(t)`; between 0 and `2 E(t)` it depends on the sign of the actual error.

**Upward drift at the descent rate (the adversarial case):**
- **In the audit set:** all 100 baseline drift-up false TOUCHDOWNs occur above `2 E(t)`. The smallest margin is 51 m (`zero_drag`, drift at `D(0.75)`, about 700 m). So F2-D2 is predicted achievable for every enumerated drift case.
- **In general, F2-D2 is not achievable for this case.** A drift that starts below `2 E(t)` (for REF-01 near the end of descent, up to about 530 m) can produce a false TOUCHDOWN that neither A nor C2 can distinguish. The barometer is alive and looks still, and the dead reckoning cannot tell `h` from 0 within its error. This is a stated residual limit, not a target miss in the audit set.

**C1, apogee-and-maximum-rate bound (fallback only).** A TOUCHDOWN is not accepted before `t_APOGEE + h_peak / v_max`.
- `v_max` is a new avionics value, `max_descent_rate_mps`: the drag-device terminal velocity at the highest altitude times 1.25 for mass and drag uncertainty, or the ballistic impact speed bound when no drag device is fitted.
- Values: `ref01` 17.6 m/s, `zero_drag` 18.5, `low_thrust` 16.7, `high4k` 22.0, `high8k` 28.2, `ref01_nodrag` 211.
- Floor: it blocks nothing after `h_peak / v_max` into the descent. That is 121 of 160 s in `ref01`, so the drift at `D(0.75)` (false TOUCHDOWN 123 s after apogee) is **not** blocked.
- It also depends on the barometer-based `h_peak`.

C1 is used only when C2 is invalid.

### 6c.4 Touchdown decision (all rules together)

A TOUCHDOWN is declared when either path completes.

**Barometric path** (primary or fallback criterion, unchanged apart from the 6c.2 thresholds). All of the following must hold:
1. The barometer has been live and valid for the whole criterion window, plus a settling hold `H_s = 1.04 s` before it. `H_s` is the time for the slower filter mode (4.44 rad/s) to decay to 1%, so the estimate no longer carries the frozen period.
2. Timing plausibility: C2 (`h_DR <= E(t)`) if C2 is valid, otherwise C1.

**Dead-reckoning path** (independent evidence, late by design):
- Fires when C2 is valid and `h_DR(t) <= -E(t)`, so the vehicle must be on the ground even with the worst-case error.
- Source `FALLBACK_CRITERION`, quality `UNRELIABLE`, reason "dead-reckoning landing".
- Delay after true contact about `E / v_d`: `ref01` 21 s, `zero_drag` 41 s, `low_thrust` 8 s, `high4k` 15 s, `high8k` 9 s, `ref01_nodrag` 0.1 s.
- A healthy landing is always declared first by the barometric path, about 2.5 s after contact. The exception is `ref01_nodrag`, where 0.1 s is still within 0 to +3 s.

Unchanged: one TOUCHDOWN at most; events never rewritten; LANDED is one-way.

**Question 4, LANDED with unreliable evidence.** Yes, LANDED is inhibited on barometer evidence while the barometer is `STALE`, invalid, or recovering. It can then come only from the dead-reckoning path.

### 6c.5 Flights where F1 latched `UNRELIABLE` (section 5.3)

The estimate quality alone does not inhibit LANDED. For a flight whose estimate is `UNRELIABLE` (F1 flag, or stale inputs), LANDED requires the same evidence as any other flight:
- either the barometric path with a live barometer and timing plausibility,
- or the dead-reckoning path.

The TOUCHDOWN event keeps the estimate's quality (`UNRELIABLE`).

For an `UNCHECKABLE` flight the accelerometer gap has also invalidated C2, so C1 is the timing check and the dead-reckoning path is not available. Probe/baseline: these flights land on time with a healthy barometer. Predicted unchanged.

### 6c.6 Recovery (question 6)

- `STALE` clears on the first differing valid sample. There is no latch: unlike F1, a recovered barometer is again evidence.
- The barometric touchdown windows restart and can only contain samples taken at least `H_s = 1.04 s` after recovery. A temporary freeze can therefore never complete a window with the frozen value. After recovery the estimate shows the true descent, so no TOUCHDOWN occurs until the real landing.
- Recorded in the TOUCHDOWN reason: any `STALE` period of the flight, with ticks.

### 6c.7 Expected outcome on the audit set (predictions for item 5)

| Group | Baseline (6b) | Expected |
|---|---|---|
| baro frozen (480) | 460 false, 20 on time | 0 false. Late through the dead-reckoning path (the 20 freezes at `D(0.98)` also become late), except `ref01_nodrag` (on time, 0.1 s) |
| baro stuck (250) | 250 false | 0 false; late (dead-reckoning path) |
| baro drift (500) | 100 false, 400 missing | 0 false (C2); the 500 become late (dead-reckoning path) |
| baro invalid (110) | 110 missing | late (dead-reckoning path) |
| temporary freeze (250) | 200 false, 47 on time, 3 late | 0 false; on time (the 3 late `high8k` fixed by 6c.2) |
| accelerometer in descent (250) | 140 on time, 10 late, 100 missing | stuck at 0 or 20 m/s²: still missing (100); the dead reckoning is implausible and the estimator is wrong. Stuck at g, held, invalid: on time |
| F1 latched (150) | 50 false, 98 on time, 2 late | 0 false; freeze cases late; others on time |
| Healthy (306) | 299 on time, 7 late | 306 on time |

Missing TOUCHDOWN: from 610 to 100 (the accelerometer stuck at 0 or 20 m/s² cases).

### 6c.8 Unit tests (F2-T01 to F2-T20)

| ID | Rule | Test |
|---|---|---|
| F2-T01 | Liveness count | 11 identical valid samples: live; 12: `STALE` on the 12th; a differing sample clears it |
| F2-T02 | Liveness and rejection | A rejected sample between identical readings restarts the count |
| F2-T03 | `N` derivation | `s = 3` gives 12, `s = 2` gives 14, `s = 1` gives 24 (formula of 6c.1) |
| F2-T04 | Configuration check | `s < 1`, detection over 0.5 s, ideal sensors with liveness on, or values differing from the sensor section: `ConfigurationFault` naming the field; noise 0 disables liveness |
| F2-T05 | Liveness never changes F1 | A stale barometer still corrects the estimator; estimate quality and deployment unchanged |
| F2-T06 | Threshold floors | REF-01 gives 2.00 / 3.00 / 3.00; `high8k` gives 3.46 / 3.00 / 6.23 |
| F2-T07 | Settling hold | A window cannot complete within `H_s + window` of a `STALE` period or an invalid sample |
| F2-T08 | C2 dead reckoning | Relative gravity term equals `2 g0 h / R0`; constant specific force `g` plus bias gives the hand-computed altitude |
| F2-T09 | `E(t)` | Equals the formula at several `t` |
| F2-T10 | C2 gate | Barometric criteria met with `h_DR > E`: no TOUCHDOWN; with `h_DR <= E`: TOUCHDOWN |
| F2-T11 | C2 validity | Accelerometer gap over 100 ms, or `v_DR` outside the plausible interval after settling: C2 invalid, C1 used |
| F2-T12 | C1 | Barometric criteria met before `t_APOGEE + h_peak / v_max`: no TOUCHDOWN |
| F2-T13 | Dead-reckoning landing | `h_DR <= -E` with C2 valid: TOUCHDOWN, source `FALLBACK_CRITERION`, quality `UNRELIABLE`; not when C2 is invalid |
| F2-T14 | One TOUCHDOWN | Both paths complete on the same tick: one event |
| F2-T15 | F1-latched flight | Estimate `UNRELIABLE`, live barometer, C2 valid: TOUCHDOWN allowed; quality `UNRELIABLE` |
| F2-T16 | Recovery | Freeze for 3 s in descent, then live: no TOUCHDOWN until the real landing |
| F2-T17 | Closed loop, frozen barometer | REF-01 seed 0, frozen at `D(0.5)`: no TOUCHDOWN before true touchdown; late TOUCHDOWN by the dead-reckoning path |
| F2-T18 | Closed loop, `high8k` | Seeds 2 and 7 (late in the baseline): TOUCHDOWN within 0 to +3 s |
| F2-T19 | Regression | EVT-13 touchdown ticks and all existing EVT tests unchanged; F1 audit rerun identical to F1 6g (F2-D5) |
| F2-T20 | Purity | The F2 rules import nothing from `aerosat.sim` or `aerosat.sensors` and use no truth identifiers |

### 6c.9 Decisions for review

1. Liveness configuration: new fields `baro_noise_sigma_pa` and `baro_resolution_pa`; 0 disables; validation-only cross-check against the sensor section; ideal-sensor audit flights run with liveness disabled.
2. Budgets: `1e-5` per flight for false `STALE`, healthy-landing lateness, and the C2 bound (`N = 12`, `z = 5.33`, 4.42).
3. Touchdown threshold floors derived from `sigma_h,pad` (6c.2), REF-01 unchanged.
4. C2 as the timing check with a separate F2 dead reckoning (the F1 one is not changed); C1 only as a fallback, with the new `max_descent_rate_mps`.
5. The dead-reckoning landing path (late LANDED by design), source `FALLBACK_CRITERION`, quality `UNRELIABLE`.
6. Stated residual: upward drift at the descent rate starting below `2 E(t)` cannot be told apart from a landing.

## 7. Out of scope

- Changing deployment behaviour (F1, closed).
- LIFTOFF never detected (F1-D6 out of scope).
