"""F1 final reconciliation batch (read-only, in memory; nothing in the repository is written).

Phase 1  healthy flights: thresholds from TRAINING data only (REF-01/zero-drag/low-thrust seeds 0-99, ideal,
         pad-only); false alarms per flight on every other dataset, incl. high-pad-altitude variants
         (pad at 4,000 m and 8,000 m MSL: thinner air, larger barometric altitude noise).
Phase 2  fault runs with the drag device disabled (ballistic logging); every detector variant and every
         response policy of the proposed state model is replayed offline up to the tick it would command.
"""
import bisect
import copy
import json
import math
import os
import statistics
import sys
import time
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import f1_lib as L  # noqa: E402
from f1v_run import path2_ticks  # noqa: E402
from aerosat.avionics.estimator import _BARO_EXPONENT  # noqa: E402
from aerosat.core.constants import LAPSE_RATE_KPM  # noqa: E402
import conftest as C  # noqa: E402
from aerosat.sim.simulator import Simulator  # noqa: E402

SIGMA_P = math.sqrt(3.0 ** 2 + 1.0 / 12.0)
STATS = ["r1", "r2", "r3", "rk", "z1", "z2", "z3", "zk", "dmean", "dmed", "dz", "dzmed"]
COMBOS = {"Mr1": ("r1", "dmean"), "Mz1": ("z1", "dmean"), "Mz2": ("z2", "dmean"), "Mz3": ("z3", "dmean"),
          "Mzk": ("zk", "dmean"), "Mz1med": ("z1", "dmed"), "Mz2med": ("z2", "dmed"), "Mz3med": ("z3", "dmed"),
          "Mzkmed": ("zk", "dmed"),
          "Mz1dz": ("z1", "dz"), "Mz2dz": ("z2", "dz"), "Mz3dz": ("z3", "dz"), "Mzkdz": ("zk", "dz"),
          "Mz1dzm": ("z1", "dzmed"), "Mz3dzm": ("z3", "dzmed"), "Mzkdzm": ("zk", "dzmed")}
POLICIES = {"QA": ("cal", "cal", False), "QB1": ("apo-1000", "burn", False), "QB3": ("apo-3000", "burn", False),
            "QB5": ("apo-5000", "burn", False), "QD": ("burn", "burn", False), "QC3": ("apo-3000", "burn", True),
            "QAacc": ("cal", "cal", True)}
OUT = os.path.join(HERE, os.environ.get("F1F_OUT", "f1f_results.json"))


# ------------------------------------------------------------------ scenarios
def build(name, drag=True):
    if name in ("ref01", "zero_drag", "low_thrust"):
        s = L.scenario(name)
        if name == "low_thrust":
            s["avionics"]["expected_coast_duration_ms"] = 8956
            s["avionics"]["backup_deployment_delay_ms"] = 11000
    else:
        s = L.scenario("ref01")
        s["environment"]["pad_altitude_msl_m"] = {"high4k": 4000.0, "high8k": 8000.0}[name]
        B, A = truth_times(s)
        s["avionics"]["expected_coast_duration_ms"] = A - B
        s["avionics"]["backup_deployment_delay_ms"] = A - B + 2000
    s["vehicle"]["drag_device"]["enabled"] = drag
    return s


def truth_times(s):
    sim = Simulator(C.ref01_simulator_config(s))
    while not (sim.truth_events.burnout_tick is not None and sim.state.velocity_mps < -5.0):
        sim.step()
    return sim.truth_events.burnout_tick, sim.truth_events.apogee_tick


# ------------------------------------------------------------------ per-sample statistics
def sample_stats(fl, der):
    cal, rows = fl["cal"], fl["rows"]
    press = {row[0]: row[6] for row in rows}
    k = _BARO_EXPONENT
    D = {d[0]: d[2] for d in der if d[2] is not None}
    out = []          # (tick, {stat: value})
    hist_r, hist_z = [], []
    prev = None
    for tick, r, Dv, dD, P, hb, h, v in der:
        if r is None:
            continue
        p = press[tick]
        dhdp = cal.reference_temperature_k / LAPSE_RATE_KPM * k * (p / cal.reference_pressure_pa) ** (k - 1) / cal.reference_pressure_pa
        z = r / (SIGMA_P * dhdp)
        if prev is None or tick - prev != L.BARO_DT:
            hist_r, hist_z = [], []
        prev = tick
        hist_r = (hist_r + [abs(r)])[-5:]
        hist_z = (hist_z + [abs(z)])[-5:]
        s = {"r1": abs(r), "z1": abs(z)}
        if len(hist_r) >= 2:
            s["r2"], s["z2"] = min(hist_r[-2:]), min(hist_z[-2:])
        if len(hist_r) >= 3:
            s["r3"], s["z3"] = min(hist_r[-3:]), min(hist_z[-3:])
        if len(hist_r) == 5:
            s["rk"], s["zk"] = sorted(hist_r)[-3], sorted(hist_z)[-3]
        a = [D.get(tick - j * 20) for j in range(5)]
        b = [D.get(tick - 200 - j * 20) for j in range(5)]
        if all(x is not None for x in a + b):
            s["dmean"] = abs(sum(a) / 5 - sum(b) / 5)
            s["dmed"] = abs(statistics.median(a) - statistics.median(b))
            s["dz"] = s["dmean"] / (SIGMA_P * dhdp)
            s["dzmed"] = s["dmed"] / (SIGMA_P * dhdp)
        out.append((tick, s))
    return out


def ev(fl, name):
    return next((e for e in fl["events"] if e[0] == name), None)


# ------------------------------------------------------------------ phase 1: healthy
def healthy_job(job):
    name, seed, ideal, pad = job
    s = build(name, True)
    if pad:
        s["timing"]["t_ignition_ms"] = 120000
        stop = 60000
    else:
        B, A = TRUTH[name]
        stop = B + s["avionics"]["backup_deployment_delay_ms"] + 2000
    fl = L.fly(s, seed, ideal=ideal, stop_tick=stop)
    der, _ = L.derive(fl)
    st = sample_stats(fl, der)
    te = fl["truth"]
    dep = ev(fl, "DRAG_DEVICE_DEPLOYED")
    bo = ev(fl, "BURNOUT")
    timer = bo[1] + s["avionics"]["backup_deployment_delay_ms"] if bo else None
    scopes = {}
    for tick, sv in st:
        if te.apogee_tick is None or tick <= te.apogee_tick:
            sc = "ascent"
        elif dep is None or tick <= dep[1]:
            sc = "apogee_to_command"
        elif timer is None or tick <= timer + 1000:
            sc = "command_to_timer+1s"
        else:
            continue
        m = scopes.setdefault(sc, {})
        for key, val in sv.items():
            if val > m.get(key, (-1, None))[0]:
                m[key] = (val, tick)
    return dict(job=job, scopes=scopes, events=fl["events"])


# ------------------------------------------------------------------ phase 2: faults
def alarms_for(st, stat, thr):
    return [t for t, sv in st if sv.get(stat, 0.0) > thr]


def replay(ctx, alarm_ticks, policy):
    ap, bo = ctx["apogee"], ctx["burnout"]
    if ap is None:
        return None, "none"
    if ap[2] == "BACKUP_TIMER":
        return ap[1], "timer"
    a = ap[1]
    d = a + 1000
    hist = lambda lo, hi: bisect.bisect_right(alarm_ticks, hi) > bisect.bisect_left(alarm_ticks, lo)
    na_from, pa_from, acc_rule = POLICIES[policy] if policy != "P0" else (None, None, False)
    lim = lambda key: {"cal": 1000, "burn": bo}.get(key) if not key.startswith("apo") else a - int(key.split("-")[1])
    if policy == "P0":
        na = ap[3] != "UNRELIABLE"
    else:
        na = ap[3] != "UNRELIABLE" and not hist(lim(na_from), d)
    if na:
        return d, "normal"
    timer = ctx["timer"]
    for t in ctx["ready"]:
        if t < d:
            continue
        if timer is not None and t >= timer:
            break
        ok = policy == "P0" or not hist(lim(pa_from), t)
        if acc_rule and ctx["acc_bad"] is not None and ctx["acc_bad"] <= t:
            ok = False
        if ok:
            return t, "path2"
    if timer is not None:
        return max(timer, a), "timer"
    return None, "none"


def fault_job(job):
    name, label, specs, seed, T = job
    faults = L.Faults(*[L.Fault(*sp) for sp in specs])
    s = build(name, False)
    B, A = TRUTH[name]
    fl = L.fly(s, seed, fault=faults, stop_tick=A + 25000)
    rows = fl["rows"]
    tv = {row[0]: row[2] for row in rows}
    der, _ = L.derive(fl)
    st = sample_stats(fl, der)
    ap, bo = ev(fl, "APOGEE"), ev(fl, "BURNOUT")
    burn = bo[1] if bo else None
    acc_bad = next((row[0] for row in rows if burn is not None and row[0] > burn and row[4] is not None and not row[5]), None)
    ctx = dict(apogee=ap, burnout=burn, timer=(burn + s["avionics"]["backup_deployment_delay_ms"]) if burn else None,
               ready=sorted(path2_ticks(rows, burn)), acc_bad=acc_bad)
    onset = faults.onset
    first, pre = {}, {}
    raw = {}
    for stat in STATS:
        al = alarms_for(st, stat, T[stat])
        raw[stat] = al
        first[stat] = next((t for t in al if t >= onset), None)
        pre[stat] = sum(1 for t in al if t < onset)
    res = {"P0": None}
    t0, p0 = replay(ctx, [], "P0")
    res["P0"] = (t0, p0, None if t0 is None else round(tv.get(t0, float("nan")), 2))
    for cname, (a_stat, b_stat) in COMBOS.items():
        al = sorted(set(raw[a_stat]) | set(raw[b_stat]))
        for pol in POLICIES:
            t, p = replay(ctx, al, pol)
            res[f"{cname}|{pol}"] = (t, p, None if t is None else round(tv.get(t, float("nan")), 2))
    d_avail = sum(1 for t, sv in st if t >= onset and "dmean" in sv)
    return dict(name=name, label=label, seed=seed, onset=onset, specs=specs,
                apogee=None if ap is None else (ap[1], ap[2], ap[3], round(tv.get(ap[1], float("nan")), 2)),
                burnout=burn, first=first, pre=pre, res=res, d_avail=d_avail, acc_bad=acc_bad,
                alarm_last={k: (v[-1] if v else None) for k, v in raw.items()})


def cases(name):
    B, A = TRUTH[name]
    r20 = lambda t: int(round(t / 20.0)) * 20
    on = {"early": r20(B + 1800), "mid": r20((A + B) / 2), "late": r20(A - 3000), "near": r20(A - 1500)}
    c = []
    for R in (5, 10, 20, 30, 50, 100, 200):
        for k, t in on.items():
            c.append((f"step -{R} m @{k}", [("baro_offset", t, None, -R)]))
    for d in (2, 5, 10, 20, 50, 100, 200):
        for k, t in on.items():
            c.append((f"drift -{d} m/s @{k}", [("baro_drift", t, None, -d)]))
    for k, t in (("mid", on["mid"]), ("late", on["late"]), ("A-2s", r20(A - 2000)), ("A-1s", r20(A - 1000)), ("A-0.5s", r20(A - 500)), ("A", r20(A))):
        c.append((f"baro frozen @{k}", [("baro_frozen", t)]))
    for R, dur in ((60, 60), (150, 200), (500, 1000)):
        for k, t in (("mid", on["mid"]), ("A-2s", r20(A - 2000))):
            c.append((f"false peak +{R} m {dur} ms @{k}", [("baro_offset", t, t + dur, R)]))
    c.append(("baro invalid mid..A+3s", [("baro_invalid", on["mid"], r20(A + 3000))]))
    c.append(("baro invalid late..A-1s", [("baro_invalid", on["late"], r20(A - 1000))]))
    for lab, sp in (("accel stuck 0 @B-1s", ("accel_value", B - 1000, None, 0.0)), ("accel stuck 9.81 @B-1s", ("accel_value", B - 1000, None, 9.80665)),
                    ("accel stuck 80 @B-1s", ("accel_value", B - 1000, None, 80.0)), ("accel held @B-1s", ("accel_hold", B - 1000)),
                    ("accel stuck 0 @mid", ("accel_value", on["mid"], None, 0.0)), ("accel stuck 9.81 @mid", ("accel_value", on["mid"], None, 9.80665)),
                    ("accel held @mid", ("accel_hold", on["mid"])), ("accel invalid @mid", ("accel_invalid", on["mid"]))):
        c.append((lab, [sp]))
    m = on["mid"]
    for dur in (20, 40, 60, 100, 300, 1000):
        c.append((f"REC step -100 m @mid {dur} ms", [("baro_offset", m, m + dur, -100)]))
    for dur in (200, 1000):
        c.append((f"REC drift -50 m/s @mid {dur} ms", [("baro_drift", m, m + dur, -50)]))
    for dur in (40, 100, 300):
        c.append((f"REC step -30 m @near {dur} ms", [("baro_offset", on["near"], on["near"] + dur, -30)]))
    c.append(("REC frozen @A-2s 500 ms", [("baro_frozen", r20(A - 2000), r20(A - 1500))]))
    t5 = r20(A + 500)
    for lab, sp in (("DELAY step -30 m @A+0.5s", [("baro_offset", t5, None, -30)]), ("DELAY step +30 m @A+0.5s", [("baro_offset", t5, None, 30)]),
                    ("DELAY drift -50 m/s @A+0.5s", [("baro_drift", t5, None, -50)]), ("DELAY accel stuck 0 @A+0.5s", [("accel_value", t5, None, 0.0)]),
                    ("DELAY baro frozen @A+0.5s", [("baro_frozen", t5)])):
        c.append((lab, sp))
    for d in (5, 10, 20, 50):
        for k in ("mid", "late"):
            c.append((f"ACCINV@B+1s drift -{d} m/s @{k}", [("accel_invalid", B + 1000), ("baro_drift", on[k], None, -d)]))
    c += [("SIM drift -20 + accel invalid @mid", [("baro_drift", m, None, -20), ("accel_invalid", m)]),
          ("SIM drift -50 + accel invalid @mid", [("baro_drift", m, None, -50), ("accel_invalid", m)]),
          ("SIM frozen + accel 0 @mid", [("baro_frozen", m), ("accel_value", m, None, 0.0)]),
          ("SIM step -100 + accel invalid @mid", [("baro_offset", m, None, -100), ("accel_invalid", m)]),
          ("SIM drift -20 + accel 9.81 @mid", [("baro_drift", m, None, -20), ("accel_value", m, None, 9.80665)]),
          ("SIM baro invalid + accel 0 @late", [("baro_invalid", on["late"]), ("accel_value", on["late"], None, 0.0)]),
          ("SIM frozen + accel held @mid", [("baro_frozen", m), ("accel_hold", m)])]
    for k, t in (("boost", r20(B - 1200)), ("mid", m), ("A-2s", r20(A - 2000)), ("A+0.5s", t5)):
        c.append((f"SPIKE +5000 Pa x1 @{k}", [("baro_pa", t, t + 1, 5000.0)]))
    for k, t in (("mid", m), ("A-2s", r20(A - 2000)), ("A+0.5s", t5)):
        c.append((f"SPIKE -5 m x1 @{k}", [("baro_offset", t, t + 1, -5)]))
        c.append((f"SPIKE +5 m x1 @{k}", [("baro_offset", t, t + 1, 5)]))
    c.append(("SPIKE -5 m x2 @mid", [("baro_offset", m, m + 21, -5)]))
    c.append(("SPIKE -20 m x1 @mid", [("baro_offset", m, m + 1, -20)]))
    return c


HIGH_CASES = ("step -100 m @mid", "step -30 m @near", "drift -20 m/s @mid", "drift -50 m/s @mid", "baro frozen @A-2s",
              "accel stuck 0 @B-1s", "SPIKE +5000 Pa x1 @mid", "SPIKE -5 m x1 @mid")
SCEN = ("ref01", "zero_drag", "low_thrust", "high4k", "high8k")
TRUTH = {}
for _n in SCEN:
    TRUTH[_n] = truth_times(build(_n, True))


def training(job):
    name, seed, ideal, pad = job
    return name in ("ref01", "zero_drag", "low_thrust") and (pad or ideal or seed < 100)


if __name__ == "__main__":
    t0 = time.time()
    hj = [(n, s, False, False) for n in ("ref01", "zero_drag", "low_thrust") for s in range(300)]
    hj += [(n, 0, True, False) for n in SCEN]
    hj += [(n, s, False, False) for n in ("high4k", "high8k") for s in range(100)]
    hj += [("ref01", s, False, True) for s in range(100)] + [("ref01", 0, True, True)]
    with Pool(11) as pool:
        H = pool.map(healthy_job, hj, chunksize=2)
        T = {}
        for h in H:
            if not training(tuple(h["job"])):
                continue
            for sc, m in h["scopes"].items():
                for key, (val, _) in m.items():
                    T[key] = max(T.get(key, 0.0), val)
        T = {k: 1.5 * v for k, v in T.items()}
        print("thresholds (1.5 x training max over ascent..timer+1s):", {k: round(v, 3) for k, v in T.items()}, f"{time.time() - t0:.0f} s")
        fj = []
        for n in ("ref01", "zero_drag", "low_thrust"):
            for label, specs in cases(n):
                fj += [(n, label, specs, seed, T) for seed in range(10)]
        for n in ("high4k", "high8k"):
            for label, specs in cases(n):
                if label in HIGH_CASES:
                    fj += [(n, label, specs, seed, T) for seed in range(10)]
        F = pool.map(fault_job, fj, chunksize=4)
    json.dump(dict(T=T, truth=TRUTH, healthy=H, faults=F), open(OUT, "w", encoding="utf-8"))
    print(f"healthy {len(H)}, fault runs {len(F)}, {time.time() - t0:.0f} s -> {OUT}")
