"""F1 audit (docs/F1_DECISIONS.md, sections 6b and 6c).

Run from `backend/`:

    uv run python tools/f1_audit.py baseline

`baseline` flies every run of the audit set closed-loop with the unchanged avionics and no
detector, drag device enabled, and records each run's deployment command against the
simulator's true apogee. Output (deterministic for the same code and inputs):

    tools/f1_out/baseline_runs.csv     one row per run
    tools/f1_out/baseline_summary.md   totals, and every run that deploys during ascent

The script lives outside the `aerosat` package. It reads truth only to score runs; the
avionics receives sensor samples and its tick only (FR-AVI-01).
"""
import argparse
import collections
import copy
import csv
import json
import os
from dataclasses import dataclass, replace
from multiprocessing import Pool
from pathlib import Path

from aerosat.avionics.estimator import _BARO_EXPONENT, baro_altitude_agl_m
from aerosat.avionics.flight_computer import FlightComputer, load_avionics_config
from aerosat.core.constants import LAPSE_RATE_KPM
from aerosat.sensors.suite import SensorSuite, suite_config_from_scenario
from aerosat.sim.simulator import DragDevice, Simulator, SimulatorConfig
from aerosat.sim.vehicle import RollProfile, ThrustCurve, Vehicle

BACKEND = Path(__file__).resolve().parents[1]
ROOT = BACKEND.parent
OUT = BACKEND / "tools" / "f1_out"
REF01 = json.loads((ROOT / "scenarios" / "ref-01.json").read_text(encoding="utf-8"))
AVIONICS_PERIOD = 5
SCENARIOS = ("ref01", "zero_drag", "low_thrust", "high4k", "high8k")
FAULT_SEEDS = range(10)


# ------------------------------------------------------------------ scenarios (section 6c)
def simulator_config(s: dict) -> SimulatorConfig:
    v = s["vehicle"]
    return SimulatorConfig(
        vehicle=Vehicle(v["dry_mass_kg"], v["propellant_mass_kg"], v["drag_coefficient"] * v["reference_area_m2"],
                        ThrustCurve([(p["t_ms"], p["thrust_n"]) for p in v["thrust_curve"]])),
        t_ignition_ms=s["timing"]["t_ignition_ms"],
        pad_altitude_msl_m=s["environment"]["pad_altitude_msl_m"],
        drag_device=DragDevice(v["drag_device"]["enabled"], v["drag_device"]["cd_area_m2"]),
        roll_profile=RollProfile([(p["t_ms"], p["roll_rate_radps"]) for p in v["roll_profile"]]),
        post_touchdown_hold_ms=s["timing"]["post_touchdown_hold_ms"],
        max_duration_ms=s["timing"]["max_duration_ms"],
    )


def base_scenario(name: str) -> dict:
    s = copy.deepcopy(REF01)
    if name == "zero_drag":
        s["vehicle"]["drag_coefficient"] = 0.0
        s["avionics"]["expected_coast_duration_ms"] = 23665
        s["avionics"]["backup_deployment_delay_ms"] = 26000
    elif name == "low_thrust":
        s["vehicle"]["thrust_curve"] = [{"t_ms": 0, "thrust_n": 0.0}, {"t_ms": 100, "thrust_n": 280.0},
                                        {"t_ms": 16000, "thrust_n": 280.0}, {"t_ms": 16200, "thrust_n": 0.0}]
        s["avionics"]["expected_coast_duration_ms"] = 8956
        s["avionics"]["backup_deployment_delay_ms"] = 11000
    elif name in ("high4k", "high8k"):
        s["environment"]["pad_altitude_msl_m"] = 4000.0 if name == "high4k" else 8000.0
    elif name != "ref01":
        raise ValueError(name)
    return s


def truth_times(s: dict) -> tuple[int, int]:
    """True burnout and apogee ticks from a sensor-free truth run (no deployment)."""
    sim = Simulator(simulator_config(s))
    while not (sim.truth_events.burnout_tick is not None and sim.state.velocity_mps < -5.0):
        sim.step()
    return sim.truth_events.burnout_tick, sim.truth_events.apogee_tick


def scenario(name: str, truth: dict) -> dict:
    s = base_scenario(name)
    if name in ("high4k", "high8k"):
        B, A = truth[name]
        s["avionics"]["expected_coast_duration_ms"] = A - B
        s["avionics"]["backup_deployment_delay_ms"] = A - B + 2000
    return s


# ------------------------------------------------------------------ faults (section 6c)
@dataclass
class Fault:
    """kind: baro_offset (m), baro_drift (m/s), baro_pa (Pa), baro_frozen, baro_invalid,
    accel_value (m/s^2), accel_hold, accel_invalid. Active for t0 <= tick < t1."""

    kind: str
    t0: int
    t1: int | None = None
    value: float = 0.0
    held: float | None = None

    def active(self, tick: int) -> bool:
        return self.t0 <= tick and (self.t1 is None or tick < self.t1)

    def apply(self, tick, samples, calibration):
        out = []
        for s in samples:
            if s.sensor_id == "baro" and self.kind.startswith("baro"):
                if not self.active(tick):
                    if self.kind == "baro_frozen" and tick < self.t0:
                        self.held = s.value
                    out.append(s)
                elif self.kind == "baro_frozen":
                    out.append(replace(s, value=self.held))
                elif self.kind == "baro_invalid":
                    out.append(replace(s, saturated=True))
                elif self.kind == "baro_pa":
                    out.append(replace(s, value=s.value + self.value))
                else:
                    h = baro_altitude_agl_m(s.value, calibration.reference_pressure_pa, calibration.reference_temperature_k)
                    dh = self.value if self.kind == "baro_offset" else self.value * (tick - self.t0) / 1000.0
                    out.append(replace(s, value=pressure_from_altitude(h + dh, calibration)))
            elif s.sensor_id == "accel" and s.channel == "z" and self.kind.startswith("accel"):
                if not self.active(tick):
                    if self.kind == "accel_hold" and tick < self.t0:
                        self.held = s.value
                    out.append(s)
                elif self.kind == "accel_hold":
                    out.append(replace(s, value=self.held))
                elif self.kind == "accel_value":
                    out.append(replace(s, value=self.value))
                else:
                    out.append(replace(s, saturated=True))
            else:
                out.append(s)
        return out


def pressure_from_altitude(h: float, cal) -> float:
    return cal.reference_pressure_pa * (1.0 - h * LAPSE_RATE_KPM / cal.reference_temperature_k) ** (1.0 / _BARO_EXPONENT)


def fault_cases(name: str, truth: dict) -> list[tuple[str, str, list[tuple]]]:
    """(group, label, fault specs). Specs are (kind, t0, t1, value) tuples."""
    B, A = truth[name]
    r20 = lambda t: int(round(t / 20.0)) * 20
    on = {"early": r20(B + 1800), "mid": r20((A + B) / 2), "late": r20(A - 3000), "near": r20(A - 1500)}
    m, t5 = on["mid"], r20(A + 500)
    c = []
    for R in (5, 10, 20, 30, 50, 100, 200):
        for k, t in on.items():
            c.append(("baro step", f"step -{R} m @{k}", [("baro_offset", t, None, -R)]))
    for d in (2, 5, 10, 20, 50, 100, 200):
        for k, t in on.items():
            c.append(("baro drift", f"drift -{d} m/s @{k}", [("baro_drift", t, None, -d)]))
    for k, t in (("mid", m), ("late", on["late"]), ("A-2s", r20(A - 2000)), ("A-1s", r20(A - 1000)),
                 ("A-0.5s", r20(A - 500)), ("A", r20(A))):
        c.append(("baro frozen", f"baro frozen @{k}", [("baro_frozen", t, None, 0.0)]))
    for R, dur in ((60, 60), (150, 200), (500, 1000)):
        for k, t in (("mid", m), ("A-2s", r20(A - 2000))):
            c.append(("false peak", f"false peak +{R} m {dur} ms @{k}", [("baro_offset", t, t + dur, R)]))
    c.append(("baro invalid", "baro invalid mid..A+3s", [("baro_invalid", m, r20(A + 3000), 0.0)]))
    c.append(("baro invalid", "baro invalid late..A-1s", [("baro_invalid", on["late"], r20(A - 1000), 0.0)]))
    for lab, sp in (("accel stuck 0 @B-1s", ("accel_value", B - 1000, None, 0.0)),
                    ("accel stuck 9.81 @B-1s", ("accel_value", B - 1000, None, 9.80665)),
                    ("accel stuck 80 @B-1s", ("accel_value", B - 1000, None, 80.0)),
                    ("accel held @B-1s", ("accel_hold", B - 1000, None, 0.0)),
                    ("accel stuck 0 @mid", ("accel_value", m, None, 0.0)),
                    ("accel stuck 9.81 @mid", ("accel_value", m, None, 9.80665)),
                    ("accel held @mid", ("accel_hold", m, None, 0.0)),
                    ("accel invalid @mid", ("accel_invalid", m, None, 0.0))):
        c.append(("accelerometer", lab, [sp]))
    for dur in (20, 40, 60, 100, 300, 1000):
        c.append(("recovery", f"REC step -100 m @mid {dur} ms", [("baro_offset", m, m + dur, -100)]))
    for dur in (200, 1000):
        c.append(("recovery", f"REC drift -50 m/s @mid {dur} ms", [("baro_drift", m, m + dur, -50)]))
    for dur in (40, 100, 300):
        c.append(("recovery", f"REC step -30 m @near {dur} ms", [("baro_offset", on["near"], on["near"] + dur, -30)]))
    c.append(("recovery", "REC frozen @A-2s 500 ms", [("baro_frozen", r20(A - 2000), r20(A - 1500), 0.0)]))
    for lab, sp in (("DELAY step -30 m @A+0.5s", [("baro_offset", t5, None, -30)]),
                    ("DELAY step +30 m @A+0.5s", [("baro_offset", t5, None, 30)]),
                    ("DELAY drift -50 m/s @A+0.5s", [("baro_drift", t5, None, -50)]),
                    ("DELAY accel stuck 0 @A+0.5s", [("accel_value", t5, None, 0.0)]),
                    ("DELAY baro frozen @A+0.5s", [("baro_frozen", t5, None, 0.0)])):
        c.append(("during delay", lab, sp))
    for d in (5, 10, 20, 50):
        for k in ("mid", "late"):
            c.append(("accel invalid then drift", f"ACCINV@B+1s drift -{d} m/s @{k}",
                      [("accel_invalid", B + 1000, None, 0.0), ("baro_drift", on[k], None, -d)]))
    c += [("simultaneous", "SIM drift -20 + accel invalid @mid", [("baro_drift", m, None, -20), ("accel_invalid", m, None, 0.0)]),
          ("simultaneous", "SIM drift -50 + accel invalid @mid", [("baro_drift", m, None, -50), ("accel_invalid", m, None, 0.0)]),
          ("simultaneous", "SIM frozen + accel 0 @mid", [("baro_frozen", m, None, 0.0), ("accel_value", m, None, 0.0)]),
          ("simultaneous", "SIM step -100 + accel invalid @mid", [("baro_offset", m, None, -100), ("accel_invalid", m, None, 0.0)]),
          ("simultaneous", "SIM drift -20 + accel 9.81 @mid", [("baro_drift", m, None, -20), ("accel_value", m, None, 9.80665)]),
          ("simultaneous", "SIM baro invalid + accel 0 @late", [("baro_invalid", on["late"], None, 0.0), ("accel_value", on["late"], None, 0.0)]),
          ("simultaneous", "SIM frozen + accel held @mid", [("baro_frozen", m, None, 0.0), ("accel_hold", m, None, 0.0)])]
    for k, t in (("boost", r20(B - 1200)), ("mid", m), ("A-2s", r20(A - 2000)), ("A+0.5s", t5)):
        c.append(("spike", f"SPIKE +5000 Pa x1 @{k}", [("baro_pa", t, t + 1, 5000.0)]))
    for k, t in (("mid", m), ("A-2s", r20(A - 2000)), ("A+0.5s", t5)):
        c.append(("spike", f"SPIKE -5 m x1 @{k}", [("baro_offset", t, t + 1, -5)]))
        c.append(("spike", f"SPIKE +5 m x1 @{k}", [("baro_offset", t, t + 1, 5)]))
    c.append(("spike", "SPIKE -5 m x2 @mid", [("baro_offset", m, m + 21, -5)]))
    c.append(("spike", "SPIKE -20 m x1 @mid", [("baro_offset", m, m + 1, -20)]))
    if name in ("high4k", "high8k"):
        keep = ("step -100 m @mid", "step -30 m @near", "drift -20 m/s @mid", "drift -50 m/s @mid", "baro frozen @A-2s",
                "accel stuck 0 @B-1s", "SPIKE +5000 Pa x1 @mid", "SPIKE -5 m x1 @mid")
        c = [x for x in c if x[1] in keep]
    return c


def audit_jobs(truth: dict) -> list[tuple]:
    """Every run of the audit set, in a fixed order: (set, scenario, group, label, specs, seed, ideal, pad)."""
    jobs = []
    for name in SCENARIOS:
        for group, label, specs in fault_cases(name, truth):
            jobs += [("fault", name, group, label, specs, seed, False, False) for seed in FAULT_SEEDS]
    for name in ("ref01", "zero_drag", "low_thrust"):
        jobs += [("healthy", name, "healthy", "train" if seed < 100 else "held-out", [], seed, False, False) for seed in range(300)]
    jobs += [("healthy", name, "healthy", "ideal", [], 0, True, False) for name in SCENARIOS]
    jobs += [("healthy", "ref01", "healthy", "pad-only", [], seed, False, True) for seed in range(100)]
    jobs.append(("healthy", "ref01", "healthy", "pad-only ideal", [], 0, True, True))
    for name in ("high4k", "high8k"):
        jobs += [("healthy", name, "healthy", "high-pad", [], seed, False, False) for seed in range(100)]
    return jobs


# ------------------------------------------------------------------ one closed-loop run
def run(job_and_truth) -> dict:
    job, truth = job_and_truth
    set_, name, group, label, specs, seed, ideal, pad = job
    s = scenario(name, truth)
    if pad:
        s["timing"]["t_ignition_ms"] = 120000
    stop_max = 60000 if pad else truth[name][1] + 30000
    sensors = copy.deepcopy(s["sensors"])
    sensors["ideal"] = ideal
    sim = Simulator(simulator_config(s))
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
                events.append((e.event.value, e.tick, e.source.value, e.quality.value, state.velocity_mps))
            for k in out.commands:
                commands.append((state.tick, k.value, state.velocity_mps))
            pending = out.commands
        if sim.finished or state.tick >= stop_max or (commands and state.velocity_mps < 0.0 and state.tick > commands[0][0]):
            break
        state = sim.step(pending)
        pending = ()
    true_apogee = sim.truth_events.apogee_tick
    dep = next((e for e in events if e[0] == "DRAG_DEVICE_DEPLOYED"), None)
    ap = next((e for e in events if e[0] == "APOGEE"), None)
    cmd = commands[0] if commands else None
    return dict(
        set=set_, scenario=name, group=group, case=label, seed=seed, ideal=ideal,
        onset_tick=min((sp[1] for sp in specs), default=""),
        command_tick=cmd[0] if cmd else "", command_time_s=f"{cmd[0] / 1000:.3f}" if cmd else "",
        deploy_source=dep[2] if dep else "", deploy_quality=dep[3] if dep else "",
        apogee_event_tick=ap[1] if ap else "", apogee_source=ap[2] if ap else "", apogee_quality=ap[3] if ap else "",
        true_apogee_tick=true_apogee, true_v_at_command=f"{cmd[2]:.2f}" if cmd else "",
        ascent=bool(cmd and true_apogee is not None and cmd[0] < true_apogee),
        commands=len(commands),
        events_unique=all(n <= 1 for n in collections.Counter(e[0] for e in events).values()),
    )


# ------------------------------------------------------------------ baseline report
def baseline(workers: int) -> None:
    truth = {name: truth_times(base_scenario(name)) for name in SCENARIOS}
    jobs = audit_jobs(truth)
    with Pool(workers) as pool:
        rows = pool.map(run, [(j, truth) for j in jobs], chunksize=4)
    OUT.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys())
    with open(OUT / "baseline_runs.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    write_summary(rows, truth)


def write_summary(rows: list[dict], truth: dict) -> None:
    faults = [r for r in rows if r["set"] == "fault"]
    healthy = [r for r in rows if r["set"] == "healthy"]
    asc = [r for r in rows if r["ascent"]]
    L = ["# F1 baseline: current avionics, no detector", "",
         "Generated by `uv run python tools/f1_audit.py baseline` (run from `backend/`). Closed loop, drag device enabled, avionics unchanged.",
         "A run deploys during ascent when its deployment command tick is earlier than the true apogee tick of the same run.",
         "Results apply only to the enumerated runs (docs/F1_DECISIONS.md, section 4, item 4).", "",
         "## Scenario truth times (ticks)", "", "| Scenario | true burnout | true apogee (no deployment) |", "|---|---|---|"]
    L += [f"| {n} | {b} | {a} |" for n, (b, a) in truth.items()]
    L += ["", "## Totals", "",
          f"- Runs: {len(rows)} ({len(faults)} fault, {len(healthy)} healthy).",
          f"- Ascent deployments: {sum(r['ascent'] for r in faults)} fault runs, {sum(r['ascent'] for r in healthy)} healthy flights.",
          f"- Runs with no deployment command: {sum(1 for r in faults if not r['commands'])} fault, "
          f"{sum(1 for r in healthy if not r['commands'] and 'pad-only' not in r['case'])} healthy (pad-only runs never deploy by design).",
          f"- Runs with more than one command: {sum(1 for r in rows if r['commands'] > 1)}. Runs with a repeated event type: {sum(1 for r in rows if not r['events_unique'])}.",
          "", "## Ascent deployments by fault group and deployment source", "",
          "| Group | Fault runs | Ascent deployments | DETECTED | BAROMETRIC_CONFIRMATION | BACKUP_TIMER |", "|---|---|---|---|---|---|"]
    for g in dict.fromkeys(r["group"] for r in faults):
        rs = [r for r in faults if r["group"] == g]
        a = [r for r in rs if r["ascent"]]
        src = collections.Counter(r["deploy_source"] for r in a)
        L.append(f"| {g} | {len(rs)} | {len(a)} | {src['DETECTED']} | {src['BAROMETRIC_CONFIRMATION']} | {src['BACKUP_TIMER']} |")
    L += ["", "## Every run that deploys during ascent", "",
          "Command time is simulation time in seconds. `v` is the true vertical velocity at the command.", "",
          "| Scenario | Fault | Seed | Command time (s) | True apogee (s) | v (m/s) | Source | Quality |", "|---|---|---|---|---|---|---|---|"]
    for r in asc:
        L.append(f"| {r['scenario']} | {r['case'] if r['set'] == 'fault' else 'healthy ' + r['case']} | {r['seed']} | {r['command_time_s']} | "
                 f"{r['true_apogee_tick'] / 1000:.3f} | {r['true_v_at_command']} | {r['deploy_source']} | {r['deploy_quality']} |")
    (OUT / "baseline_summary.md").write_text("\n".join(L) + "\n", encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mode", choices=["baseline"])
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    args = ap.parse_args()
    if args.mode == "baseline":
        baseline(args.workers)


if __name__ == "__main__":
    main()
