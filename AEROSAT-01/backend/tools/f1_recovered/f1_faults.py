"""E2 sweep, E6 replay, E7 injected faults. In-memory only. Thresholds come from f1_healthy.json."""
import json
import os
import sys
import time
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import f1_lib as L  # noqa: E402
from f1_healthy import METRICS, series  # noqa: E402

ASCENT = ("PAD", "BOOST", "COAST", "NEAR_APOGEE")
MARGINS = (1.0, 1.5, 2.0)
PERSIST = (1, 2, 3)
OUT = os.path.join(HERE, "f1_faults.json")


def thresholds():
    """Observed maximum per metric and persistence over all healthy TRAINING flights (seeds 0-99, three
    scenarios, noisy), the ideal flights and the pad-only runs, over the ascent phases (PAD..NEAR_APOGEE)."""
    data = json.load(open(os.path.join(HERE, "f1_healthy.json"), encoding="utf-8"))
    T = {}
    for res in data:
        name, seed, ideal, pad = res["job"]
        if not ideal and seed >= 100:
            continue                      # validation seeds are held out
        for m in METRICS:
            for ph, a in res["metrics"][m].items():
                if ph not in ASCENT:
                    continue
                for p, key in ((1, "max"), (2, "p2"), (3, "p3")):
                    T[(m, p)] = max(T.get((m, p), 0.0), a[key])
    stuck = {}
    for res in data:
        name, seed, ideal, pad = res["job"]
        if ideal or seed >= 100:
            continue
        for sensor in ("accel", "baro"):
            for ph, n in res["stuck"][sensor][0].items():
                if ph in ASCENT and not pad:
                    stuck[sensor] = max(stuck.get(sensor, 0), n)
    return T, stuck


def first_detection(seq, thr, p, start, end):
    """First tick in [start, end] at which p consecutive (20 ms spaced) samples all exceed thr in magnitude.
    Also returns the first such tick before `start` (a false alarm)."""
    run, prev, before, after = 0, None, None, None
    for tick, x in seq:
        if tick > end:
            break
        if abs(x) > thr:
            run = run + 1 if (prev is not None and tick - prev == L.BARO_DT) else 1
        else:
            run = 0
        prev = tick
        if run >= p:
            if tick < start and before is None:
                before = tick
            elif tick >= start:
                after = tick
                break
    return before, after


def stuck_detection(rows, idx, n_needed, start, end):
    prev, run = None, 0
    for row in rows:
        if row[idx] is None or row[0] < 1000:
            continue
        if row[0] > end:
            break
        run = run + 1 if row[idx] == prev else 1
        prev = row[idx]
        if run >= n_needed and row[0] >= start:
            return row[0]
    return None


def run_case(job):
    label, specs, seed, ideal, stop, T, stuck = job
    faults = L.Faults(*[L.Fault(*s) for s in specs])
    f = L.fly(L.scenario("ref01"), seed, ideal=ideal, fault=faults, stop_tick=stop)
    te = f["truth"]
    rows = f["rows"]
    tv = {row[0]: row[2] for row in rows}
    th = {row[0]: row[1] for row in rows}
    onset = faults.onset
    end = (te.apogee_tick + 2000) if te.apogee_tick is not None else stop
    der, _ = L.derive(f)
    ser = series(der)
    det = {}
    for m in METRICS:
        for p in PERSIST:
            for k in MARGINS:
                b, a = first_detection(ser[m], T[(m, p)] * k, p, onset, end)
                det[f"{m}|p{p}|x{k}"] = (b, a)
    for sensor, idx in (("accel", 4), ("baro", 6)):
        for extra in (1, 2):
            n = stuck[sensor] + 1 if extra == 1 else 2 * stuck[sensor]
            det[f"stuck_{sensor}|n{n}"] = (None, stuck_detection(rows, idx, n, onset, end))
    q_changes, last = [], "NOMINAL"
    for row in rows:
        if row[0] >= 1000 and row[12] != last:
            q_changes.append((row[0], row[12])); last = row[12]
    evs = []
    for name, tick, src, q in f["events"]:
        evs.append((name, tick, src, q, round(tv.get(tick, float("nan")), 2), round(th.get(tick, float("nan")), 1)))
    return dict(label=label, specs=specs, seed=seed, ideal=ideal, onset=onset, events=evs,
                commands=[(t, k, round(tv.get(t, float("nan")), 2)) for t, k in f["commands"]],
                quality=q_changes[:12], detections=det,
                truth=dict(apogee=te.apogee_tick, touchdown=te.touchdown_tick, impact=te.impact_velocity_mps))


def cases():
    out = []
    S = 1000
    # ---- E2 sweep, seeds 0-4
    for sign, onsets in ((-1, (5, 8, 12, 15, 18, 21, 23)), (+1, (15, 21, 23))):
        for R in (5, 10, 20, 30, 50, 75, 100, 150, 200):
            for t in onsets:
                out.append(("E2", f"step {sign * R:+d} m @{t}s", [("baro_offset", t * S, None, sign * R)], range(5), 70000))
        for rate in (2, 5, 10, 20, 50, 100, 200):
            for t in onsets:
                out.append(("E2", f"drift {sign * rate:+d} m/s @{t}s", [("baro_drift", t * S, None, sign * rate)], range(5), 70000))
    # ---- E6 replay, seeds 0-9
    E6 = []
    for R in (15, 60, 150):
        for t in (8, 15, 22):
            E6.append((f"step -{R} m @{t}s", [("baro_offset", t * S, None, -R)]))
    for rate in (2, 10, 50, 150):
        for t in (8, 15, 22):
            E6.append((f"drift -{rate} m/s @{t}s", [("baro_drift", t * S, None, -rate)]))
    for t in (5, 10, 15, 22):
        E6.append((f"baro frozen @{t}s", [("baro_frozen", t * S)]))
    E6 += [("false peak +60 m 60 ms @15s", [("baro_offset", 15000, 15060, 60)]),
           ("false peak +150 m 200 ms @15s", [("baro_offset", 15000, 15200, 150)]),
           ("false peak +500 m 1 s @15s", [("baro_offset", 15000, 16000, 500)]),
           ("false peak +60 m 60 ms @22s", [("baro_offset", 22000, 22060, 60)]),
           ("baro invalid 3.9-4.5 s", [("baro_invalid", 3900, 4500)]),
           ("baro invalid 10-26 s", [("baro_invalid", 10000, 26000)]),
           ("baro invalid 20-24.4 s", [("baro_invalid", 20000, 24400)]),
           ("baro invalid from 10 s", [("baro_invalid", 10000)]),
           ("accel stuck 0 @3.5s", [("accel_value", 3500, None, 0.0)]),
           ("accel stuck 0 @4.5s", [("accel_value", 4500, None, 0.0)]),
           ("accel stuck 9.81 @3.5s", [("accel_value", 3500, None, 9.80665)]),
           ("accel stuck 80 @3.5s", [("accel_value", 3500, None, 80.0)]),
           ("accel stuck 9.81 @1.0s (pad)", [("accel_value", 1000, None, 9.80665)]),
           ("REC baro frozen 10-12 s", [("baro_frozen", 10000, 12000)]),
           ("REC drift -50 m/s 15-16 s", [("baro_drift", 15000, 16000, -50)]),
           ("REC step -60 m 15-15.5 s", [("baro_offset", 15000, 15500, -60)]),
           ("REC baro invalid 20-21 s", [("baro_invalid", 20000, 21000)]),
           ("REC accel invalid 3.5-4.5 s", [("accel_invalid", 3500, 4500)]),
           ("REC accel stuck 0 3.5-4.0 s", [("accel_value", 3500, 4000, 0.0)]),
           ("COMBO baro frozen + accel 9.81 @10s", [("baro_frozen", 10000), ("accel_value", 10000, None, 9.80665)]),
           ("COMBO drift -10 + accel invalid @15s", [("baro_drift", 15000, None, -10), ("accel_invalid", 15000)]),
           ("COMBO baro invalid 20-26 + accel 0 @22s", [("baro_invalid", 20000, 26000), ("accel_value", 22000, None, 0.0)]),
           ("COMBO accel 9.81 @3.5 + step -60 @15", [("accel_value", 3500, None, 9.80665), ("baro_offset", 15000, None, -60)])]
    for label, specs in E6:
        out.append(("E6", label, specs, range(10), 80000))
    # ---- E7 injected faults in ideal mode (seed irrelevant)
    for label, specs in (("step -60 m @15s", [("baro_offset", 15000, None, -60)]),
                         ("drift -10 m/s @15s", [("baro_drift", 15000, None, -10)]),
                         ("baro frozen @15s", [("baro_frozen", 15000)]),
                         ("accel stuck 0 @3.5s", [("accel_value", 3500, None, 0.0)]),
                         ("accel stuck 9.81 @3.5s", [("accel_value", 3500, None, 9.80665)]),
                         ("accel stuck 80 @3.5s", [("accel_value", 3500, None, 80.0)])):
        out.append(("E7", label, specs, range(1), 80000))
    return out


if __name__ == "__main__":
    T, stuck = thresholds()
    jobs = []
    for exp, label, specs, seeds, stop in cases():
        for seed in seeds:
            jobs.append(((exp, label), specs, seed, exp == "E7", stop, T, stuck))
    t0 = time.time()
    with Pool(11) as pool:
        results = pool.map(run_case, jobs, chunksize=2)
    json.dump(dict(T={f"{m}|p{p}": v for (m, p), v in T.items()}, stuck=stuck, results=results),
              open(OUT, "w", encoding="utf-8"))
    print(f"{len(results)} fault runs in {time.time() - t0:.0f} s -> {OUT}")
