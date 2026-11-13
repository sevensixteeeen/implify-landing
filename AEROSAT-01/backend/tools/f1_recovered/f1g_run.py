"""F1 final mathematical and policy audit (read-only, in memory; nothing in the repository is written).

Phase 1  healthy flights: per-sample normalised statistics, empirical spread by phase and speed,
         thresholds (1.5 x training maximum), per-flight false alarms, policy replay on healthy flights.
Phase 2  fault runs (drag device disabled, ballistic logging): final monitors, policies A and B at W = 1 s,
         timer fired on the avionics 5 ms grid.
Phase 3  edge cases around the deployment-command tick, ballistic + drag-enabled twins (current code).
"""
import bisect
import collections
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
from f1f_run import build, cases, HIGH_CASES, TRUTH  # noqa: E402
from f1v_run import path2_ticks  # noqa: E402
from aerosat.avionics.estimator import _BARO_EXPONENT  # noqa: E402
from aerosat.core.constants import LAPSE_RATE_KPM  # noqa: E402

SIGMA_P = math.sqrt(3.0 ** 2 + 1.0 / 12.0)      # barometer noise 3 Pa + 1 Pa quantisation (REF-01 sensor values)
SIGMA_A = math.sqrt(0.05 ** 2 + 0.005 ** 2 / 12)  # accelerometer noise + quantisation
N_CAL = 200                                      # accelerometer samples in the 1 s pad calibration
DT = 0.005
KAPPA = 0.5 / 288.15                              # altitude scale uncertainty from thermometer bias sigma 0.5 K
STATS = ["qr3of5", "qD0", "qD1", "qD2"]
W_MS = 1000
OUT = os.path.join(HERE, "f1g_results.json")


def dhdp_of(p, cal):
    k = _BARO_EXPONENT
    return cal.reference_temperature_k / LAPSE_RATE_KPM * k * (p / cal.reference_pressure_pa) ** (k - 1) / cal.reference_pressure_pa


def per_sample(fl, der):
    """(tick, {stat: value}, v_est, sigma_h) per valid barometer sample after calibration."""
    cal, rows = fl["cal"], fl["rows"]
    press = {row[0]: row[6] for row in rows}
    vest = {row[0]: row[10] for row in rows}
    D = {d[0]: d[2] for d in der if d[2] is not None}
    out, hist, prev = [], [], None
    for tick, r, Dv, dD, P, hb, h, v in der:
        if r is None:
            continue
        sh = SIGMA_P * dhdp_of(press[tick], cal)
        if prev is None or tick - prev != L.BARO_DT:
            hist = []
        prev = tick
        hist = (hist + [abs(r) / sh])[-5:]
        s = {"qr1": abs(r) / sh}
        if len(hist) == 5:
            s["qr3of5"] = sorted(hist)[-3]
        a = [D.get(tick - j * 20) for j in range(5)]
        b = [D.get(tick - 200 - j * 20) for j in range(5)]
        if all(x is not None for x in a + b):
            dd = sum(a) / 5 - sum(b) / 5
            t_cal = (tick - fl["cal_ticks"]) / 1000.0
            sv = math.sqrt((SIGMA_A / math.sqrt(N_CAL) * t_cal) ** 2 + SIGMA_A ** 2 * DT * t_cal)   # DR velocity error
            s_dr = 0.2 * sv
            base = sh * math.sqrt(2.0 / 5.0)
            s["dD"] = dd
            s["qD0"] = abs(dd) / base
            s["qD1"] = abs(dd) / math.sqrt(base ** 2 + s_dr ** 2)
            s["qD2"] = abs(dd) / math.sqrt(base ** 2 + s_dr ** 2 + (KAPPA * abs(vest[tick]) * 0.2) ** 2)
            s["signed0"] = dd / base
        out.append((tick, s, vest[tick], sh))
    return out


def ev(fl, name):
    return next((e for e in fl["events"] if e[0] == name), None)


def timer_tick(burn, backup):
    return None if burn is None else burn + int(math.ceil(backup / 5.0)) * 5


def replay(ctx, alarms, policy):
    """Deployment (tick, path) under the proposed state model. policy: 'P0', 'A' or 'B' (W = 1 s)."""
    ap, bo = ctx["apogee"], ctx["burnout"]
    if ap is None:
        return None, "none"
    if ap[2] == "BACKUP_TIMER":
        return ap[1], "timer"
    a, d = ap[1], ap[1] + 1000
    hist = lambda lo, hi: bisect.bisect_right(alarms, hi) > bisect.bisect_left(alarms, lo)
    if policy == "P0":
        na = ap[3] != "UNRELIABLE"
    else:
        na = ap[3] != "UNRELIABLE" and not hist(a - W_MS, d)
    if na:
        return d, "normal"
    tt = ctx["timer"]
    for t in ctx["ready"]:
        if t < d:
            continue
        if tt is not None and t >= tt:
            break
        ok = policy == "P0" or not hist(bo, t)
        if policy == "B" and ctx["acc_bad"] is not None and ctx["acc_bad"] <= t:
            ok = False
        if ok:
            return t, "path2"
    if tt is not None:
        return max(tt, a), "timer"
    return None, "none"


# ------------------------------------------------------------------ phase 1
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
    st = per_sample(fl, der)
    te = fl["truth"]
    dep = ev(fl, "DRAG_DEVICE_DEPLOYED")
    bo = ev(fl, "BURNOUT")
    tt = timer_tick(bo[1] if bo else None, s["avionics"]["backup_deployment_delay_ms"])
    mx = {}
    bins = collections.defaultdict(lambda: [0, 0.0, 0.0])     # (stat, phase or speed bin) -> n, sum, sumsq
    for tick, sv, v, sh in st:
        if not pad and tt is not None and tick > tt + 1000:
            continue
        ph = L.phase_of(tick, te)
        for k in STATS + ["qr1"]:
            if k in sv and sv[k] > mx.get(k, (-1,))[0]:
                mx[k] = (sv[k], tick, ph)
        if "signed0" in sv:
            sb = f"|v| {int(min(abs(v), 239) // 40) * 40}-{int(min(abs(v), 239) // 40) * 40 + 40}"
            for key in (("qD0s", ph), ("qD0s", sb)):
                b = bins[key]; b[0] += 1; b[1] += sv["signed0"]; b[2] += sv["signed0"] ** 2
        b = bins[("qr", ph)]; b[0] += 1; b[1] += sv["qr1"]; b[2] += sv["qr1"] ** 2
    # per-flight signed bias of qD0 at speed > 120 m/s (scale-error signature)
    fast = [sv["signed0"] for tick, sv, v, sh in st if "signed0" in sv and abs(v) > 120]
    slow = [sv["signed0"] for tick, sv, v, sh in st if "signed0" in sv and abs(v) < 40]
    bias = (statistics.fmean(fast) if fast else None, statistics.fmean(slow) if slow else None)
    events_ok = all(n <= 1 for n in collections.Counter(e[0] for e in fl["events"]).values())
    ctx = dict(apogee=ev(fl, "APOGEE"), burnout=bo[1] if bo else None, timer=tt,
               ready=sorted(path2_ticks(fl["rows"], bo[1] if bo else None)), acc_bad=None)
    pol = {p: replay(ctx, [], p) for p in ("P0", "A", "B")}
    return dict(job=job, max=mx, bins={f"{k[0]}|{k[1]}": v for k, v in bins.items()}, bias=bias,
                events=fl["events"], events_unique=events_ok, policies=pol,
                commands=sum(1 for e in fl["events"] if e[0] == "DRAG_DEVICE_DEPLOYED"))


# ------------------------------------------------------------------ phase 2 and 3
def fault_job(job):
    name, label, specs, seed, T, drag, qd = job
    faults = L.Faults(*[L.Fault(*sp) for sp in specs])
    s = build(name, drag)
    B, A = TRUTH[name]
    fl = L.fly(s, seed, fault=faults, stop_tick=(None if drag else A + 25000))
    rows = fl["rows"]
    tv = {row[0]: row[2] for row in rows}
    out = dict(name=name, label=label, seed=seed, onset=faults.onset, drag=drag, specs=specs,
               events=[(e[0], e[1], e[2], e[3], round(tv.get(e[1], float("nan")), 2)) for e in fl["events"]],
               events_unique=all(n <= 1 for n in collections.Counter(e[0] for e in fl["events"]).values()),
               commands=len(fl["commands"]), truth_apogee=fl["truth"].apogee_tick)
    if drag:
        return out
    der, _ = L.derive(fl)
    st = per_sample(fl, der)
    ap, bo = ev(fl, "APOGEE"), ev(fl, "BURNOUT")
    burn = bo[1] if bo else None
    acc_bad = next((row[0] for row in rows if burn is not None and row[0] > burn and row[4] is not None and not row[5]), None)
    ctx = dict(apogee=ap, burnout=burn, timer=timer_tick(burn, s["avionics"]["backup_deployment_delay_ms"]),
               ready=sorted(path2_ticks(rows, burn)), acc_bad=acc_bad)
    rI = [t for t, sv, v, sh in st if sv.get("qr3of5", 0) > T["qr3of5"]]
    res, first, inter, same = {}, {}, {}, {}
    for q in qd:
        rD = [t for t, sv, v, sh in st if sv.get(q, 0) > T[q]]
        al = sorted(set(rI) | set(rD))
        for p in ("A", "B"):
            t, path = replay(ctx, al, p)
            res[f"{q}|{p}"] = (t, path, None if t is None else round(tv.get(t, float("nan")), 2))
        first[q] = next((t for t in rD if t >= faults.onset), None)
        inter[q] = intervals(rD)
        same[q] = len(set(rI) & set(rD))
    t, path = replay(ctx, [], "P0")
    res["P0"] = (t, path, None if t is None else round(tv.get(t, float("nan")), 2))
    first["qr3of5"] = next((t for t in rI if t >= faults.onset), None)
    out.update(apogee=None if ap is None else (ap[1], ap[2], ap[3], round(tv.get(ap[1], float("nan")), 2)),
               burnout=burn, acc_bad=acc_bad, res=res, first=first, pre={"qr3of5": sum(1 for t in rI if t < faults.onset)},
               int_I=intervals(rI), int_D=inter, same_tick=same, timer=ctx["timer"],
               d_avail=sum(1 for t, sv, v, sh in st if t >= faults.onset and "qD0" in sv))
    return out


def intervals(ticks, gap=20):
    out = []
    for t in ticks:
        if out and t - out[-1][1] <= gap:
            out[-1][1] = t
        else:
            out.append([t, t])
    return out[:30]


def command_tick(job):
    """Nominal (unfaulted) ballistic run: detected APOGEE tick + 1000 = the normal-path command tick."""
    name, seed = job
    fl = L.fly(build(name, False), seed, stop_tick=TRUTH[name][1] + 3000)
    ap = ev(fl, "APOGEE")
    return name, seed, None if ap is None else ap[1] + 1000


def lyapunov():
    """Steady-state innovation spread of the fixed-gain filter, in units of barometer noise."""
    Kh, Kv, T_b = 0.24, 0.72, 0.02
    sigma_m = 1.0
    qa = (SIGMA_A / 0.30) ** 2  # accelerometer noise relative to ~0.3 m barometric altitude noise
    P = [[0.0, 0.0], [0.0, 0.0]]
    for _ in range(4000):
        for _ in range(4):          # four 5 ms predictions per barometer sample
            F = [[1, DT], [0, 1]]
            G = [0.5 * DT * DT, DT]
            P = [[sum(F[i][k] * sum(P[k][l] * F[j][l] for l in range(2)) for k in range(2)) + G[i] * G[j] * qa for j in range(2)] for i in range(2)]
        S = P[0][0] + sigma_m ** 2
        # fixed-gain correction: x <- x + K (z - Hx); error e <- (I - K H) e - K n
        K = [Kh, Kv]
        IKH = [[1 - K[0], 0], [-K[1], 1]]
        P = [[sum(IKH[i][k] * sum(P[k][l] * IKH[j][l] for l in range(2)) for k in range(2)) + K[i] * K[j] * sigma_m ** 2 for j in range(2)] for i in range(2)]
    return math.sqrt(S)


if __name__ == "__main__":
    t0 = time.time()
    print("theory: innovation spread / barometer noise =", round(lyapunov(), 4),
          "| dD baro-only spread / barometer noise = sqrt(2/5) =", round(math.sqrt(0.4), 4))
    SCEN = ("ref01", "zero_drag", "low_thrust", "high4k", "high8k")
    hj = [(n, s, False, False) for n in ("ref01", "zero_drag", "low_thrust") for s in range(300)]
    hj += [(n, 0, True, False) for n in SCEN]
    hj += [(n, s, False, False) for n in ("high4k", "high8k") for s in range(100)]
    hj += [("ref01", s, False, True) for s in range(100)] + [("ref01", 0, True, True)]
    with Pool(11) as pool:
        H = pool.map(healthy_job, hj, chunksize=2)
        train = lambda j: j[0] in ("ref01", "zero_drag", "low_thrust") and (j[2] or j[3] or j[1] < 100)
        T = {}
        for h in H:
            if train(h["job"]):
                for k, (v, tick, ph) in h["max"].items():
                    T[k] = max(T.get(k, 0.0), v)
        T = {k: 1.5 * v for k, v in T.items()}
        print("thresholds:", {k: round(v, 3) for k, v in T.items()}, f"{time.time() - t0:.0f} s")
        QD = ["qD0", "qD1", "qD2"]
        fj = []
        for n in ("ref01", "zero_drag", "low_thrust"):
            for label, specs in cases(n):
                fj += [(n, label, specs, seed, T, False, QD) for seed in range(10)]
        for n in ("high4k", "high8k"):
            for label, specs in cases(n):
                if label in HIGH_CASES:
                    fj += [(n, label, specs, seed, T, False, QD) for seed in range(10)]
        F = pool.map(fault_job, fj, chunksize=4)
        print(f"fault runs {len(F)}, {time.time() - t0:.0f} s")
        # phase 3: edge cases around the command tick
        cmd = pool.map(command_tick, [(n, s) for n in ("ref01", "zero_drag", "low_thrust") for s in range(40)])
        ej = []
        for n in ("ref01", "zero_drag", "low_thrust"):
            on_grid = [c for c in cmd if c[0] == n and c[2] is not None and c[2] % 20 == 0][:5]
            off_grid = [c for c in cmd if c[0] == n and c[2] is not None and c[2] % 20 != 0][:5]
            for (_, seed, C) in on_grid + off_grid:
                last_before = (C - 1) // 20 * 20
                first_after = (C // 20 + 1) * 20
                plan = [("EDGE spike -50 m on the command tick" if C % 20 == 0 else None, C),
                        ("EDGE spike -50 m one barometer sample before the command", last_before if C % 20 else C - 20),
                        ("EDGE spike -50 m first barometer sample after the command", first_after),
                        ("EDGE step -30 m starting 100 ms before the command", (C - 100) // 20 * 20)]
                for lab, t in plan:
                    if lab is None:
                        continue
                    spec = [("baro_offset", t, t + 1, -50)] if "spike" in lab else [("baro_offset", t, None, -30)]
                    for drag in (False, True):
                        ej.append((n, lab + f" [C={C}]", spec, seed, T, drag, ["qD0"]))
        E = pool.map(fault_job, ej, chunksize=2)
    json.dump(dict(T=T, healthy=H, faults=F, edge=E, cmd=cmd), open(OUT, "w", encoding="utf-8"))
    print(f"healthy {len(H)}, faults {len(F)}, edge {len(E)}, {time.time() - t0:.0f} s -> {OUT}")
