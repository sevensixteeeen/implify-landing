"""F2 audit (docs/F2_DECISIONS.md, sections 4 and 6a).

Run from `backend/`:

    uv run python tools/f2_audit.py baseline

Flies every run of the audit set closed loop until the simulator finishes (true touchdown plus
the post-touchdown hold, F2-D1) and records each TOUCHDOWN against the simulator's true
touchdown. Output (deterministic for the same code and inputs):

    tools/f2_out/baseline_runs.csv     one row per run
    tools/f2_out/baseline_summary.md   section 4 scoring and every false TOUCHDOWN

Scenarios and fault injection are those of the F1 audit (`tools/f1_audit.py`), plus the
ballistic variant `ref01_nodrag` and two stuck-barometer fault kinds. The script lives outside the
`aerosat` package; it reads truth only to score runs.
"""
import argparse
import collections
import copy
import csv
import os
import sys
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import f1_audit as F1  # noqa: E402

from aerosat.avionics.estimator import baro_altitude_agl_m  # noqa: E402
from aerosat.avionics.flight_computer import FlightComputer, load_avionics_config  # noqa: E402
from aerosat.core.types import VehicleCommand  # noqa: E402
from aerosat.sensors.suite import SensorSuite, suite_config_from_scenario  # noqa: E402
from aerosat.sim.simulator import Simulator  # noqa: E402

OUT = Path(__file__).resolve().parent / "f2_out"
DRAG_SCENARIOS = F1.SCENARIOS                      # ref01, zero_drag, low_thrust, high4k, high8k
SCENARIOS = DRAG_SCENARIOS + ("ref01_nodrag",)
FAULT_SEEDS = range(10)
HEALTHY_SEEDS = range(50)
AVIONICS_PERIOD = 5


# ------------------------------------------------------------------ scenarios and reference times
def build(name: str, ba: dict) -> dict:
    if name == "ref01_nodrag":
        s = F1.base_scenario("ref01")
        s["vehicle"]["drag_device"]["enabled"] = False
        return s
    return F1.scenario(name, ba)


def reference_times() -> dict:
    """Per scenario: B, A from a sensor-free truth run; TD and v_d from a sensor-free run with the drag
    device commanded at A + 1000 ms (none for ref01_nodrag)."""
    ba = {name: F1.truth_times(F1.base_scenario(name)) for name in DRAG_SCENARIOS}
    ba["ref01_nodrag"] = ba["ref01"]
    ref = {}
    for name in SCENARIOS:
        B, A = ba[name]
        sim = Simulator(F1.simulator_config(build(name, ba)))
        v_last = 0.0
        while not sim.finished:
            cmd = (VehicleCommand.DEPLOY_DRAG_DEVICE,) if (name != "ref01_nodrag" and sim.tick == A + 1000) else ()
            state = sim.step(cmd)
            if not state.on_ground:
                v_last = state.velocity_mps
            if sim.truth_events.touchdown_tick is not None:
                break
        ref[name] = dict(B=B, A=A, TD=sim.truth_events.touchdown_tick, v_d=round(abs(v_last), 1))
    return ref


# ------------------------------------------------------------------ faults
class Fault(F1.Fault):
    """F1 fault kinds plus: baro_stuck_alt (reading held at the altitude at onset + value, m) and
    baro_stuck_pad (reading held at the pad reference pressure, 0 m AGL)."""

    def apply(self, tick, samples, calibration):
        if self.kind not in ("baro_stuck_alt", "baro_stuck_pad"):
            return super().apply(tick, samples, calibration)
        out = []
        for s in samples:
            if s.sensor_id == "baro" and self.active(tick):
                if self.held is None:
                    if self.kind == "baro_stuck_pad":
                        self.held = calibration.reference_pressure_pa
                    else:
                        h = baro_altitude_agl_m(s.value, calibration.reference_pressure_pa, calibration.reference_temperature_k)
                        self.held = F1.pressure_from_altitude(h + self.value, calibration)
                out.append(replace(s, value=self.held))
            else:
                out.append(s)
        return out


def fault_cases(name: str, ref: dict) -> list[tuple[str, str, list[tuple]]]:
    """(group, label, specs) per section 6a. Specs are (kind, t0, t1, value)."""
    r = ref[name]
    B, A, TD, vd = r["B"], r["A"], r["TD"], r["v_d"]
    r20 = lambda t: int(round(t / 20.0)) * 20
    D = lambda f: r20(A + f * (TD - A))
    ascent = r20((A + B) / 2)
    if name == "ref01_nodrag":
        return [("baro frozen", "frozen @A-1s", [("baro_frozen", r20(A - 1000), None, 0.0)]),
                ("baro frozen", "frozen @A+3s", [("baro_frozen", r20(A + 3000), None, 0.0)]),
                ("baro frozen", "frozen @D(0.5)", [("baro_frozen", D(0.5), None, 0.0)]),
                ("baro invalid", "invalid from D(0.5)", [("baro_invalid", D(0.5), None, 0.0)])]
    c = []
    for lab, t in (("ascent", ascent), ("A-1s", r20(A - 1000)), ("A+0.5s", r20(A + 500))) + tuple(
            (f"D({f})", D(f)) for f in (0.1, 0.25, 0.5, 0.75, 0.9, 0.98)):
        c.append(("baro frozen", f"frozen @{lab}", [("baro_frozen", t, None, 0.0)]))
    for f in (0.25, 0.75):
        for off in (200, -200):
            c.append(("baro stuck", f"stuck at altitude {off:+d} m @D({f})", [("baro_stuck_alt", D(f), None, float(off))]))
    c.append(("baro stuck", "stuck at pad pressure @D(0.5)", [("baro_stuck_pad", D(0.5), None, 0.0)]))
    for f in (0.25, 0.75):
        for k in (0.5, 1.0, 1.5):
            rate = round(k * vd, 1)
            c.append(("baro drift", f"drift up {k} v_d ({rate} m/s) @D({f})", [("baro_drift", D(f), None, rate)]))
        for rate in (5, 20):
            c.append(("baro drift", f"drift down {rate} m/s @D({f})", [("baro_drift", D(f), None, -float(rate))]))
    for f in (0.25, 0.75):
        c.append(("baro invalid", f"invalid from D({f})", [("baro_invalid", D(f), None, 0.0)]))
    for f, dur in ((0.25, 1000), (0.25, 3000), (0.25, 5000), (0.25, 10000), (0.9, 3000)):
        c.append(("temporary freeze", f"frozen @D({f}) for {dur // 1000} s", [("baro_frozen", D(f), D(f) + dur, 0.0)]))
    m = D(0.5)
    for lab, sp in (("accel stuck 0", ("accel_value", m, None, 0.0)), ("accel stuck 9.81", ("accel_value", m, None, 9.80665)),
                    ("accel stuck 20", ("accel_value", m, None, 20.0)), ("accel held", ("accel_hold", m, None, 0.0)),
                    ("accel invalid", ("accel_invalid", m, None, 0.0))):
        c.append(("accelerometer in descent", f"{lab} @D(0.5)", [sp]))
    c.append(("F1 latched", "accel invalid 200 ms @ascent (UNCHECKABLE)", [("accel_invalid", ascent, ascent + 200, 0.0)]))
    c.append(("F1 latched", "step -100 m 100 ms @ascent (DISAGREEMENT)", [("baro_offset", ascent, ascent + 100, -100.0)]))
    c.append(("F1 latched", "step -100 m 100 ms @ascent + frozen @D(0.5)",
              [("baro_offset", ascent, ascent + 100, -100.0), ("baro_frozen", D(0.5), None, 0.0)]))
    return c


def audit_jobs(ref: dict) -> list[tuple]:
    """(set, scenario, group, case, specs, seed, ideal), fixed order."""
    jobs = []
    for name in SCENARIOS:
        for group, label, specs in fault_cases(name, ref):
            jobs += [("fault", name, group, label, specs, seed, False) for seed in FAULT_SEEDS]
    for name in SCENARIOS:
        jobs += [("healthy", name, "healthy", "noisy", [], seed, False) for seed in HEALTHY_SEEDS]
        jobs.append(("healthy", name, "healthy", "ideal", [], 0, True))
    return jobs


# ------------------------------------------------------------------ one run, to the end of the hold
def run(arg) -> dict:
    job, ref = arg
    set_, name, group, label, specs, seed, ideal = job
    ba = {n: (ref[n]["B"], ref[n]["A"]) for n in ref}
    s = build(name, ba)
    sensors = copy.deepcopy(s["sensors"])
    sensors["ideal"] = ideal
    sim = Simulator(F1.simulator_config(s))
    suite = SensorSuite(suite_config_from_scenario(sensors), master_seed=seed)
    computer = FlightComputer(load_avionics_config(s))
    faults = [Fault(*sp) for sp in specs]
    events, commands = [], []
    state, pending = sim.state, ()
    while True:
        samples = suite.sample(state)
        if computer.calibration is not None:
            for f in faults:
                samples = f.apply(state.tick, samples, computer.calibration)
        else:
            for f in faults:
                if f.kind == "baro_frozen":
                    f.held = next((x.value for x in samples if x.sensor_id == "baro"), f.held)
        if state.tick % AVIONICS_PERIOD == 0:
            out = computer.tick(state.tick, samples)
            for e in out.events:
                events.append((e.event.value, e.tick, e.source.value, e.quality.value, state.altitude_agl_m, state.velocity_mps))
            commands += [(state.tick, k.value) for k in out.commands]
            pending = out.commands
        if sim.finished:
            break
        state = sim.step(pending)
        pending = ()
    te = sim.truth_events
    td = next((e for e in events if e[0] == "TOUCHDOWN"), None)
    dep = next((e for e in events if e[0] == "DRAG_DEVICE_DEPLOYED"), None)
    false_td = td is not None and te.touchdown_tick is not None and td[1] < te.touchdown_tick
    delay = None if td is None or te.touchdown_time_s is None else td[1] / 1000.0 - te.touchdown_time_s
    outcome = ("false" if false_td else "missing" if td is None else "on time" if 0.0 <= delay <= 3.0 else "late" if delay > 3.0 else "early")
    flag = computer.disagreement_flag
    return dict(
        set=set_, scenario=name, group=group, case=label, seed=seed, ideal=ideal,
        onset_tick=min((sp[1] for sp in specs), default=""),
        touchdown_tick=td[1] if td else "", criterion=({"DETECTED": "primary", "FALLBACK_CRITERION": "fallback"}.get(td[2], td[2]) if td else ""),
        touchdown_quality=td[3] if td else "",
        true_alt_at_touchdown_m=f"{td[4]:.1f}" if td else "", true_v_at_touchdown_mps=f"{td[5]:.2f}" if td else "",
        true_touchdown_tick=te.touchdown_tick, true_touchdown_time_s=f"{te.touchdown_time_s:.3f}" if te.touchdown_time_s is not None else "",
        delay_s=f"{delay:.3f}" if delay is not None else "", outcome=outcome, false_touchdown=false_td,
        deploy_tick=dep[1] if dep else "", deploy_source=dep[2] if dep else "", commands=len(commands),
        flag_reason=flag.reason.value if flag else "", final_state=computer.flight_state.value,
        events_unique=all(n <= 1 for n in collections.Counter(e[0] for e in events).values()),
    )


# ------------------------------------------------------------------ baseline
def baseline(workers: int) -> None:
    ref = reference_times()
    jobs = audit_jobs(ref)
    with Pool(workers) as pool:
        rows = pool.map(run, [(j, ref) for j in jobs], chunksize=2)
    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / "baseline_runs.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()), lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    write_summary(rows, ref, "baseline_summary.md", "F2 baseline: current avionics (F1 closed), before any F2 change")


def write_summary(rows: list[dict], ref: dict, filename: str, title: str) -> None:
    faults = [r for r in rows if r["set"] == "fault"]
    healthy = [r for r in rows if r["set"] == "healthy"]
    false_all = [r for r in rows if r["false_touchdown"]]
    h_bad = [r for r in healthy if r["outcome"] != "on time"]
    L = [f"# {title}", "",
         "Generated by `uv run python tools/f2_audit.py baseline` (run from `backend/`). Every run flown closed loop to true "
         "touchdown plus the post-touchdown hold. Results apply only to these runs (section 4, item 5).", "",
         "## Reference times (ticks; v_d in m/s)", "", "| Scenario | B | A | TD | v_d |", "|---|---|---|---|---|"]
    L += [f"| {n} | {r['B']} | {r['A']} | {r['TD']} | {r['v_d']} |" for n, r in ref.items()]
    L += ["", "## Section 4 criteria", "", "| Criterion | Result |", "|---|---|",
          f"| 1. False TOUCHDOWN, all runs | {len(false_all)} of {len(rows)} ({sum(1 for r in false_all if r['set'] == 'fault')} fault, "
          f"{sum(1 for r in false_all if r['set'] == 'healthy')} healthy) |",
          f"| 2. Healthy TOUCHDOWN within 0 to +3 s | {len(healthy) - len(h_bad)} of {len(healthy)} |",
          "| 3. Fault runs, late or missing TOUCHDOWN | table below |",
          "| 4. F1 preserved | not applicable to the baseline (no F2 change yet) |", "",
          f"Other checks: runs with more than one command {sum(1 for r in rows if r['commands'] > 1)}; runs with a repeated event type "
          f"{sum(1 for r in rows if not r['events_unique'])}.", ""]
    if h_bad:
        L += ["Healthy flights outside 0 to +3 s:", ""] + [f"- {r['scenario']} seed {r['seed']} ideal {r['ideal']}: {r['outcome']} {r['delay_s']} s" for r in h_bad] + [""]
    delays = sorted(float(r["delay_s"]) for r in healthy if r["delay_s"])
    if delays:
        L += [f"Healthy TOUCHDOWN delay: min {delays[0]:+.3f} s, median {delays[len(delays) // 2]:+.3f} s, max {delays[-1]:+.3f} s.", ""]
    L += ["## Outcome per fault group", "", "| Group | Runs | False | On time | Late | Missing | Early, not false |", "|---|---|---|---|---|---|---|"]
    for g in dict.fromkeys(r["group"] for r in faults):
        rs = [r for r in faults if r["group"] == g]
        c = collections.Counter(r["outcome"] for r in rs)
        L.append(f"| {g} | {len(rs)} | {c['false']} | {c['on time']} | {c['late']} | {c['missing']} | {c['early']} |")
    L += ["", "## Outcome per fault case", "", "| Scenario | Case | Runs | False | Criterion (false) | On time | Late | Missing |", "|---|---|---|---|---|---|---|---|"]
    cases = collections.defaultdict(list)
    for r in faults:
        cases[(r["scenario"], r["case"])].append(r)
    for (n, case), rs in cases.items():
        c = collections.Counter(r["outcome"] for r in rs)
        crit = collections.Counter(r["criterion"] for r in rs if r["false_touchdown"])
        L.append(f"| {n} | {case} | {len(rs)} | {c['false']} | {', '.join(f'{k} {v}' for k, v in sorted(crit.items())) or '-'} | "
                 f"{c['on time']} | {c['late']} | {c['missing']} |")
    L += ["", "## Every false TOUCHDOWN", "",
          "Times in seconds of simulation time. Altitude and velocity are the truth on the TOUCHDOWN tick.", "",
          "| Set | Scenario | Group | Case | Onset (s) | Seed | TOUCHDOWN (s) | True touchdown (s) | Criterion | True altitude (m) | True velocity (m/s) |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in false_all:
        onset = f"{r['onset_tick'] / 1000:.2f}" if r["onset_tick"] != "" else "-"
        L.append(f"| {r['set']} | {r['scenario']} | {r['group']} | {r['case']} | {onset} | {r['seed']} | {r['touchdown_tick'] / 1000:.3f} | "
                 f"{r['true_touchdown_time_s']} | {r['criterion']} | {r['true_alt_at_touchdown_m']} | {r['true_v_at_touchdown_mps']} |")
    (OUT / filename).write_text("\n".join(L) + "\n", encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mode", choices=["baseline"])
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    args = ap.parse_args()
    baseline(args.workers)


if __name__ == "__main__":
    main()
