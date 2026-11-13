"""F1 validation batch (read-only, in memory).

A. Fresh held-out healthy flights, seeds 200-299, three scenarios (drag device enabled, normal flights):
   both candidate monitors' maxima per phase, innovation tail counts, innovation normalised by the
   local barometric noise sigma.
B. Extended fault cases, three scenarios, seeds 0-9, drag device DISABLED ("ballistic logging"):
   truth stays the no-deployment trajectory, so each response policy can be replayed offline up to the
   tick at which it WOULD command deployment. Monitors never feed back into the avionics.
C. Twins of B with the drag device enabled, seed 0, to check that the offline replay of the current
   code (policy P0) reproduces the real deployment tick.
"""
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
from aerosat.avionics.estimator import _BARO_EXPONENT  # noqa: E402
from aerosat.core.constants import LAPSE_RATE_KPM  # noqa: E402

R_THR = 1.5 * 1.4708   # 2.206 m: 1.5 x training maximum (as reported)
D_THR = 1.5 * 0.9312   # 1.397 m
HOLD = 1000            # P1 alarm hold, ms
SIGMA_P = math.sqrt(3.0 ** 2 + 1.0 / 12.0)   # barometer noise + 1 Pa quantisation (truth-side, analysis only)
TRUTH = {"ref01": (6200, 24247), "zero_drag": (6200, 29865), "low_thrust": (19200, 28156)}
OUT = os.path.join(HERE, "f1v_results.json")


def scen(name, drag=True):
    s = L.scenario(name)
    if name == "low_thrust":       # test-only timer for this variant: coast = 28156 - 19200 = 8956 ms
        s["avionics"]["expected_coast_duration_ms"] = 8956
        s["avionics"]["backup_deployment_delay_ms"] = 11000
    s["vehicle"]["drag_device"]["enabled"] = drag
    return s


def alarms(der):
    r_ticks, d_ticks, d_avail = [], [], set()
    for tick, r, D, dD, P, hb, hacc, vacc in der:
        if r is not None and abs(r) > R_THR:
            r_ticks.append(tick)
        if 0.2 in dD:
            d_avail.add(tick)
            if abs(dD[0.2]) > D_THR:
                d_ticks.append(tick)
    return r_ticks, d_ticks, d_avail


def intervals(ticks, gap=40):
    out = []
    for t in ticks:
        if out and t - out[-1][1] <= gap:
            out[-1][1] = t
        else:
            out.append([t, t])
    return out


def path2_ticks(rows, burnout):
    """Ticks at which the barometric-confirmation count is >= 5, replaying _track_barometric_descent."""
    ready, recent, peak, count = set(), [], None, 0
    if burnout is None:
        return ready
    for row in rows:
        tick = row[0]
        if tick <= burnout:
            continue
        present, valid, h = row[6] is not None, row[7], row[8]
        if present and not valid:
            recent, count = [], 0
        elif valid:
            recent = (recent + [h])[-5:]
            if len(recent) == 5:
                med = statistics.median(recent)
                peak = med if peak is None or med > peak else peak
            count = count + 1 if peak is not None and peak - h >= 10.0 else 0
        if count >= 5:
            ready.add(tick)
    return ready


def replay(f, all_alarms, policy):
    """Deployment tick and path under a response policy, from the logged (ballistic) run."""
    ev = {e[0]: e for e in f["events"]}
    ap, bo = ev.get("APOGEE"), ev.get("BURNOUT")
    if ap is None:
        return None, "none"
    burn = bo[1] if bo else None
    timer = burn + f["backup"] if burn is not None else None
    if ap[2] == "BACKUP_TIMER":
        return ap[1], "timer"
    a = ap[1]
    import bisect
    alarm_in = lambda lo, hi: bisect.bisect_right(all_alarms, hi) > bisect.bisect_left(all_alarms, lo)
    reliable = ap[3] != "UNRELIABLE"
    if policy == "P1":
        reliable = reliable and not alarm_in(a - HOLD, a)
    if policy in ("P2", "P3"):
        reliable = reliable and not alarm_in(1000, a + 1000)
    if reliable:
        return a + 1000, "normal"
    ready = f["path2"]
    last = f["rows"][-1][0]
    t = a
    while t <= last:
        p2_ok = policy != "P3" or not alarm_in(1000, t)
        if t >= a + 1000 and t in ready and p2_ok:
            return t, "path2"
        if timer is not None and t >= timer:
            return t, "timer"
        t += 5
    return None, "none"


def fault_job(job):
    name, label, specs, seed, drag = job
    faults = L.Faults(*[L.Fault(*s) for s in specs])
    s = scen(name, drag)
    fl = L.fly(s, seed, fault=faults)
    rows = fl["rows"]
    tv = {row[0]: row[2] for row in rows}
    th = {row[0]: row[1] for row in rows}
    evs = [(e[0], e[1], e[2], e[3], round(tv.get(e[1], float("nan")), 2), round(th.get(e[1], float("nan")), 1)) for e in fl["events"]]
    out = dict(name=name, label=label, seed=seed, drag=drag, onset=faults.onset, events=evs,
               truth_apogee=fl["truth"].apogee_tick)
    if drag:
        return out
    der, _ = L.derive(fl)
    r_t, d_t, d_avail = alarms(der)
    union = sorted(set(r_t) | set(d_t))
    burnout = next((e[1] for e in fl["events"] if e[0] == "BURNOUT"), None)
    ctx = dict(events=fl["events"], rows=rows, backup=s["avionics"]["backup_deployment_delay_ms"],
               path2=path2_ticks(rows, burnout))
    pol = {}
    for p in ("P0", "P1", "P2", "P3"):
        tick, path = replay(ctx, union, p)
        pol[p] = (tick, path, None if tick is None else round(tv.get(tick, float("nan")), 2))
    onset = faults.onset
    end = max((sp[2] for sp in specs if len(sp) > 2 and sp[2] is not None), default=None)
    after = [t for t in d_avail if t >= onset]
    out.update(policies=pol, r_int=intervals([t for t in r_t if t >= onset - 2000])[:20],
               d_int=intervals([t for t in d_t if t >= onset - 2000])[:20],
               first_r=next((t for t in r_t if t >= onset), None), first_d=next((t for t in d_t if t >= onset), None),
               pre_onset_alarms=sum(1 for t in union if t < onset),
               d_avail_after_onset=len(after), fault_end=end)
    return out


def healthy_job(job):
    name, seed = job
    s = scen(name, True)
    fl = L.fly(s, seed)
    te, cal = fl["truth"], fl["cal"]
    der, _ = L.derive(fl)
    press = {row[0]: row[6] for row in fl["rows"]}
    k = _BARO_EXPONENT
    ph_stats = {}
    for tick, r, D, dD, P, hb, hacc, vacc in der:
        ph = L.phase_of(tick, te)
        a = ph_stats.setdefault(ph, dict(n=0, rmax=0.0, dmax=0.0, zmax=0.0, zs=0.0, zss=0.0,
                                         r_over={x: 0 for x in (1.5, 1.8, 2.0, 2.206)},
                                         z_over={x: 0 for x in (3, 4, 4.5, 5, 5.5, 6)}, d_over=0))
        if r is not None:
            p = press[tick]
            dhdp = cal.reference_temperature_k / LAPSE_RATE_KPM * k * (p / cal.reference_pressure_pa) ** (k - 1) / cal.reference_pressure_pa
            z = r / (SIGMA_P * dhdp)
            a["n"] += 1; a["zs"] += z; a["zss"] += z * z
            a["rmax"] = max(a["rmax"], abs(r)); a["zmax"] = max(a["zmax"], abs(z))
            for x in a["r_over"]:
                a["r_over"][x] += abs(r) > x
            for x in a["z_over"]:
                a["z_over"][x] += abs(z) > x
        if 0.2 in dD:
            a["dmax"] = max(a["dmax"], abs(dD[0.2]))
            a["d_over"] += abs(dD[0.2]) > D_THR
    for a in ph_stats.values():
        a["r_over"] = {str(k2): v for k2, v in a["r_over"].items()}
        a["z_over"] = {str(k2): v for k2, v in a["z_over"].items()}
    return dict(name=name, seed=seed, phases=ph_stats, events=fl["events"])


def cases(name):
    B, A = TRUTH[name]
    r20 = lambda t: int(round(t / 20.0)) * 20
    mid, late = r20(B + (A - B) / 2), r20(A - 3000)
    c = []
    for when, tag in ((mid, "mid-coast"), (late, "apogee-3s")):
        for d in (-5, -10, -20, -50):
            c.append((f"drift {d} m/s + accel invalid @{tag}", [("baro_drift", when, None, d), ("accel_invalid", when)]))
    c.append(("accel invalid only @mid-coast", [("accel_invalid", mid)]))
    for lab, spec in (("accel stuck 80", ("accel_value", B - 1000, None, 80.0)), ("accel stuck 0", ("accel_value", B - 1000, None, 0.0)),
                      ("accel stuck 9.81", ("accel_value", B - 1000, None, 9.80665)), ("accel held at last value", ("accel_hold", B - 1000))):
        c.append((f"{lab} @burnout-1s", [spec]))
    for dt in (-2000, -1000, -500, 0, 500, 1000):
        c.append((f"baro frozen @apogee{dt:+d}ms", [("baro_frozen", r20(A + dt))]))
    for dur in (20, 40, 60, 100, 300, 1000, None):
        c.append((f"REC step -100 m @mid-coast for {dur if dur else 'ever'} ms", [("baro_offset", mid, None if dur is None else mid + dur, -100)]))
    for dur in (200, 1000):
        c.append((f"REC drift -50 m/s @mid-coast for {dur} ms", [("baro_drift", mid, mid + dur, -50)]))
    for dur in (40, 100, 300):
        t0 = r20(A - 1500)
        c.append((f"REC step -30 m @apogee-1.5s for {dur} ms", [("baro_offset", t0, t0 + dur, -30)]))
    c += [("SIM baro frozen + accel stuck 0 @mid-coast", [("baro_frozen", mid), ("accel_value", mid, None, 0.0)]),
          ("SIM step -100 m + accel invalid @mid-coast", [("baro_offset", mid, None, -100), ("accel_invalid", mid)]),
          ("SIM drift -20 m/s + accel stuck 9.81 @mid-coast", [("baro_drift", mid, None, -20), ("accel_value", mid, None, 9.80665)]),
          ("SIM baro invalid + accel stuck 0 @apogee-3s", [("baro_invalid", late), ("accel_value", late, None, 0.0)]),
          ("SIM baro frozen + accel held @mid-coast", [("baro_frozen", mid), ("accel_hold", mid)])]
    return c


if __name__ == "__main__":
    jobs_h = [(n, s) for n in TRUTH for s in range(200, 300)]
    jobs_f = []
    for n in TRUTH:
        for label, specs in cases(n):
            jobs_f += [(n, label, specs, seed, False) for seed in range(10)]
            jobs_f.append((n, label, specs, 0, True))
    t0 = time.time()
    with Pool(11) as pool:
        res_f = pool.map(fault_job, jobs_f, chunksize=2)
        res_h = pool.map(healthy_job, jobs_h, chunksize=2)
    json.dump(dict(fault=res_f, healthy=res_h), open(OUT, "w", encoding="utf-8"))
    print(f"fault jobs {len(jobs_f)} (ballistic {sum(1 for j in jobs_f if not j[4])}, drag-enabled twins {sum(1 for j in jobs_f if j[4])}), "
          f"healthy jobs {len(jobs_h)}, {time.time() - t0:.0f} s -> {OUT}")
