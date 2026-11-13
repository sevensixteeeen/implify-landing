import collections
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "f1v_results.json"), encoding="utf-8"))
PH = ("PAD", "BOOST", "COAST", "NEAR_APOGEE", "DESCENT", "LANDED")

print("=" * 120)
print("A. Held-out healthy flights, seeds 200-299 (drag device enabled)")
agg = collections.defaultdict(lambda: dict(n=0, rmax=0.0, zmax=0.0, zs=0.0, zss=0.0, dmax=0.0, r_over=collections.Counter(), z_over=collections.Counter(), d_over=0, flights_r=0, flights_d=0))
for h in D["healthy"]:
    for ph, a in h["phases"].items():
        b = agg[(h["name"], ph)]
        b["n"] += a["n"]; b["zs"] += a["zs"]; b["zss"] += a["zss"]
        b["rmax"] = max(b["rmax"], a["rmax"]); b["zmax"] = max(b["zmax"], a["zmax"]); b["dmax"] = max(b["dmax"], a["dmax"])
        b["r_over"].update(a["r_over"]); b["z_over"].update(a["z_over"]); b["d_over"] += a["d_over"]
        b["flights_r"] += a["r_over"]["2.206"] > 0; b["flights_d"] += a["d_over"] > 0
print(f"{'scenario':11}{'phase':12}{'n':>9}{'max|r|':>8}{'r>2.206':>8}{'fl':>4}{'max dD':>8}{'dD>1.397':>9}{'fl':>4}{'sd z':>7}{'max z':>7}  z>4 obs/exp   z>4.5 obs/exp   z>5 obs/exp")
tot = collections.Counter()
for name in ("ref01", "zero_drag", "low_thrust"):
    for ph in PH:
        b = agg.get((name, ph))
        if not b or not b["n"]:
            continue
        sd = math.sqrt(b["zss"] / b["n"] - (b["zs"] / b["n"]) ** 2)
        exp = {x: b["n"] * math.erfc(x / (sd * math.sqrt(2))) for x in (4, 4.5, 5)}
        print(f"{name:11}{ph:12}{b['n']:>9}{b['rmax']:>8.3f}{b['r_over']['2.206']:>8}{b['flights_r']:>4}{b['dmax']:>8.3f}{b['d_over']:>9}{b['flights_d']:>4}{sd:>7.3f}{b['zmax']:>7.2f}"
              f"  {b['z_over']['4']:>4}/{exp[4]:<7.2f}  {b['z_over']['4.5']:>4}/{exp[4.5]:<7.3f}  {b['z_over']['5']:>4}/{exp[5]:<7.4f}")
        if ph != "LANDED":
            tot["r"] += b["r_over"]["2.206"]; tot["d"] += b["d_over"]
print(f"alarm samples before LANDED: r {tot['r']}, dD {tot['d']}  (flights: {len(D['healthy'])})")
ev = collections.Counter((h["name"], e[0], e[2]) for h in D["healthy"] for e in h["events"])
print("events:", dict(ev))

print("=" * 120)
print("B. Extended faults, seeds 0-9, ballistic logging; policies replayed offline")
print("   P0 current code | P1 alarm within 1 s before APOGEE -> UNRELIABLE | P2 any alarm from calibration end to APOGEE+1 s inhibits the normal path (path 2 + timer remain) | P3 as P2 and path 2 also inhibited after any alarm (timer only)")
print("   cell = premature/10 (true v>0 at the command) worst..best true v, paths; det = first alarm latency ms max (missed), [k] = alarm before a premature APOGEE")
F = [r for r in D["fault"] if not r["drag"]]
twins = [r for r in D["fault"] if r["drag"]]
groups = collections.defaultdict(list)
for r in F:
    groups[(r["name"], r["label"])].append(r)
for (name, label), rs in groups.items():
    cells = []
    for p in ("P0", "P1", "P2", "P3"):
        vs = [x["policies"][p][2] for x in rs if x["policies"][p][0] is not None]
        paths = collections.Counter(x["policies"][p][1] for x in rs)
        prem = sum(1 for v in vs if v > 0)
        rng = f"{max(vs):+.0f}..{min(vs):+.0f}" if vs else "none"
        cells.append(f"{p} {prem}/{len(rs)} {rng:>11} {','.join(f'{k[:5]}{n}' for k, n in sorted(paths.items()))}")
    ap = [x for x in rs if any(e[0] == "APOGEE" for e in x["events"])]
    early = [x for x in rs if any(e[0] == "APOGEE" and e[4] > 0 for e in x["events"])]
    lat = []
    for key in ("first_r", "first_d"):
        got = [x[key] - x["onset"] for x in rs if x[key] is not None]
        lat.append(f"{max(got) if got else '-'}({len(rs) - len(got)}m)")
    union = [min(t for t in (x["first_r"], x["first_d"]) if t is not None) if (x["first_r"] or x["first_d"]) else None for x in rs]
    before = sum(1 for x, u in zip(rs, union) if u is not None and any(e[0] == "APOGEE" and e[4] > 0 and u <= e[1] for e in x["events"]))
    air = sum(1 for x in rs if any(e[0] == "TOUCHDOWN" and e[5] > 5 for e in x["events"]))
    noap = len(rs) - len(ap)
    print(f"{name:10} {label:52} earlyAPO {len(early)}/{len(rs)} noAPO {noap} airLND {air} | det r {lat[0]:>10} dD {lat[1]:>10} [{before}] pre {sum(x['pre_onset_alarms'] for x in rs)}")
    print(f"{'':63}" + " | ".join(cells))
print("=" * 120)
print("C. Replay check: P0 offline replay of the ballistic run vs the real deployment of the drag-enabled twin (seed 0)")
ok = bad = 0
for t in twins:
    b = next(x for x in F if x["name"] == t["name"] and x["label"] == t["label"] and x["seed"] == 0)
    real = next((e[1] for e in t["events"] if e[0] == "DRAG_DEVICE_DEPLOYED"), None)
    if real == b["policies"]["P0"][0]:
        ok += 1
    else:
        bad += 1
        print("   MISMATCH", t["name"], t["label"], "real", real, "replay", b["policies"]["P0"])
print(f"   {ok} match, {bad} mismatch, of {len(twins)}")
air = [(t["name"], t["label"]) for t in twins if any(e[0] == "TOUCHDOWN" and e[5] > 5 for e in t["events"])]
print(f"   drag-enabled twins with TOUCHDOWN declared above 5 m: {len(air)} {air[:12]}")
print("=" * 120)
print("D. Recovery: alarm clear time after the fault ends (ms, max over seeds), from alarm intervals")
for (name, label), rs in groups.items():
    if not label.startswith("REC"):
        continue
    clears = []
    for x in rs:
        end = x["fault_end"]
        if end is None:
            continue
        last = max([iv[1] for iv in x["r_int"] + x["d_int"] if iv[0] <= end + 3000] or [None]) if (x["r_int"] or x["d_int"]) else None
        clears.append(None if last is None else last - end)
    got = [c for c in clears if c is not None]
    print(f"   {name:10} {label:52} last alarm relative to fault end: {min(got) if got else '-'}..{max(got) if got else '-'} ms")
