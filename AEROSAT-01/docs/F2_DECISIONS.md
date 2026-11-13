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

## 7. Out of scope

- Changing deployment behaviour (F1, closed).
- LIFTOFF never detected (F1-D6 out of scope).
