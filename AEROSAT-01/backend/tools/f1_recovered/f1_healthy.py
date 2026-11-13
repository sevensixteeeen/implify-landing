"""E1, E3, E4, E5 and the healthy part of E7. In-memory only; writes a JSON summary to the scratchpad."""
import json
import os
import sys
import time
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import f1_lib as L  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "f1_healthy.json")
METRICS = ["r"] + [f"dD{w}" for w in L.WINDOWS_S] + [f"P{w}" for w in L.WINDOWS_S] + ["absD"]


def series(der):
    ser = {m: [] for m in METRICS}
    for tick, r, D, dD, P, hb, hacc, vacc in der:
        if r is not None:
            ser["r"].append((tick, r))
        if D is not None:
            ser["absD"].append((tick, D))
        for w, x in dD.items():
            ser[f"dD{w}"].append((tick, x))
        for w, x in P.items():
            ser[f"P{w}"].append((tick, x))
    return ser


def summarize(seq, te):
    """Per phase: n, sum, sumsq, max|x|, its tick, persistence maxima (n=2,3), lag-1 sums, same-sign run."""
    acc = {}
    window = []
    prev_tick, prev_x, prev_ph = None, None, None
    sign_run = 0
    for tick, x in seq:
        ph = L.phase_of(tick, te)
        a = acc.setdefault(ph, dict(n=0, s=0.0, ss=0.0, max=0.0, max_tick=None, p2=0.0, p3=0.0,
                                    lag_n=0, lag_s=0.0, run=0))
        a["n"] += 1; a["s"] += x; a["ss"] += x * x
        if abs(x) > a["max"]:
            a["max"], a["max_tick"] = abs(x), tick
        contiguous = prev_tick is not None and tick - prev_tick == L.BARO_DT and ph == prev_ph
        if not contiguous:
            window, sign_run = [], 0
        window = (window + [abs(x)])[-3:]
        if len(window) >= 2:
            a["p2"] = max(a["p2"], min(window[-2:]))
        if len(window) >= 3:
            a["p3"] = max(a["p3"], min(window))
        if contiguous:
            a["lag_n"] += 1; a["lag_s"] += x * prev_x
            sign_run = sign_run + 1 if (x > 0) == (prev_x > 0) else 1
        else:
            sign_run = 1
        a["run"] = max(a["run"], sign_run)
        prev_tick, prev_x, prev_ph = tick, x, ph
    return acc


def healthy(job):
    name, seed, ideal, pad_only = job
    scen = L.scenario(name, ignition_ms=120000 if pad_only else None)
    f = L.fly(scen, seed, ideal=ideal, stop_tick=60000 if pad_only else None)
    te = f["truth"]
    der, _ = L.derive(f)
    ser = series(der)
    truth = {row[0]: (row[1], row[2]) for row in f["rows"]}
    dr = {}
    for label, tk in (("apogee", te.apogee_tick), ("touchdown", te.touchdown_tick)):
        if tk is None:
            continue
        near = min((d for d in der if d[6] is not None), key=lambda d: abs(d[0] - tk), default=None)
        if near is not None and near[0] in truth:
            th, tv = truth[near[0]]
            dr[label] = dict(dh=near[6] - th, dv=near[7] - tv, D=near[2])
    acc_runs = L.run_lengths(f["rows"], te, 4)
    baro_runs = L.run_lengths(f["rows"], te, 6)
    quality = sum(1 for row in f["rows"] if row[12] != "NOMINAL" and row[0] >= f["cal_ticks"])
    return dict(job=job, metrics={m: summarize(s, te) for m, s in ser.items()}, dr=dr,
                stuck=dict(accel=acc_runs, baro=baro_runs), events=f["events"],
                truth=dict(liftoff=te.liftoff_tick, burnout=te.burnout_tick, apogee=te.apogee_tick,
                           touchdown=te.touchdown_tick),
                non_nominal_ticks=quality, cal=dict(bias=f["cal"].accel_bias_mps2) if f["cal"] else None)


if __name__ == "__main__":
    jobs = []
    for name in ("ref01", "zero_drag", "low_thrust"):
        jobs += [(name, seed, False, False) for seed in range(0, 200)]
        jobs.append((name, 0, True, False))
    jobs += [("ref01", seed, False, True) for seed in range(100)]
    jobs.append(("ref01", 0, True, True))
    t0 = time.time()
    with Pool(11) as pool:
        results = pool.map(healthy, jobs, chunksize=2)
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(results, fh)
    print(f"{len(results)} flights in {time.time() - t0:.0f} s -> {OUT}")
