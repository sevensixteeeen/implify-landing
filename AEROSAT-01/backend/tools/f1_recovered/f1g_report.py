import collections
import json
import math
import os
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "f1g_results.json"), encoding="utf-8"))
T, H, F, E = D["T"], D["healthy"], D["faults"], D["edge"]
PH = ("PAD", "BOOST", "COAST", "NEAR_APOGEE", "DESCENT")


def ds(job):
    n, s, ideal, pad = job
    if pad:
        return "pad-ideal" if ideal else "pad 0-99"
    if ideal:
        return "ideal"
    if n.startswith("high"):
        return f"{n} 0-99"
    return "train 0-99" if s < 100 else "held 100-299"


print("A. MEASURED SPREAD, noisy training+held flights (REF-01, zero-drag, low-thrust), within the monitor lifetime")
agg = collections.defaultdict(lambda: [0, 0.0, 0.0])
for h in H:
    if ds(h["job"]) in ("train 0-99", "held 100-299"):
        for k, (n, s, ss) in h["bins"].items():
            a = agg[k]; a[0] += n; a[1] += s; a[2] += ss
for k in sorted(agg):
    n, s, ss = agg[k]
    mean = s / n
    print(f"   {k:28} n={n:>8} mean {mean:+.3f}  sd {math.sqrt(ss / n - mean * mean):.3f}  rms {math.sqrt(ss / n):.3f}")
fast = [h["bias"][0] for h in H if ds(h["job"]) in ("train 0-99", "held 100-299") and h["bias"][0] is not None]
slow = [h["bias"][1] for h in H if ds(h["job"]) in ("train 0-99", "held 100-299") and h["bias"][1] is not None]
print(f"   per-flight mean of signed qD0 at |v|>120 m/s: mean {statistics.fmean(fast):+.3f}, sd {statistics.pstdev(fast):.3f}, max |.| {max(abs(x) for x in fast):.3f} (n={len(fast)})")
print(f"   per-flight mean of signed qD0 at |v|<40 m/s:  mean {statistics.fmean(slow):+.3f}, sd {statistics.pstdev(slow):.3f}, max |.| {max(abs(x) for x in slow):.3f}")

print("\nB. THRESHOLDS (1.5 x training max) and per-flight false alarms; dataset max")
order = ["train 0-99", "held 100-299", "ideal", "pad 0-99", "pad-ideal", "high4k 0-99", "high8k 0-99"]
for k in ("qr1", "qr3of5", "qD0", "qD1", "qD2"):
    line = f"   {k:7} thr {T.get(k, float('nan')):6.3f} |"
    for d in order:
        hs = [h for h in H if ds(h["job"]) == d]
        vals = [h["max"][k][0] for h in hs if k in h["max"]]
        fa = sum(1 for v in vals if v > T.get(k, 1e9))
        line += f" {d}: {fa}/{len(hs)} max {max(vals) if vals else 0:.2f} |"
    print(line)
    held = max(h["max"][k][0] for h in H if ds(h["job"]) == "held 100-299" and k in h["max"])
    print(f"           margin vs held-out {T.get(k, float('nan')) / held:.2f}; where training max occurred: "
          f"{max(((h['max'][k][0], h['job'], h['max'][k][2]) for h in H if ds(h['job']) in ('train 0-99', 'pad 0-99') and k in h['max']))}")
pol_h = collections.Counter((h["policies"]["P0"] == h["policies"]["A"], h["policies"]["P0"] == h["policies"]["B"]) for h in H if not h["job"][3])
print("   healthy flights where policy A / B decision equals the current code:", dict(pol_h))
print("   healthy flights: commands per flight", dict(collections.Counter(h["commands"] for h in H if not h["job"][3])),
      "| all events unique:", all(h["events_unique"] for h in H))

print("\nC. POLICIES A (path 2 kept) and B (path 2 withheld after accelerometer invalid since BURNOUT), W = 1 s, timer on the 5 ms grid")
apo = {r["name"]: None for r in F}
for qd in ("qD0", "qD1", "qD2"):
    for p in ("A", "B"):
        key = f"{qd}|{p}"
        vals = [r["res"][key] for r in F]
        nan = sum(1 for v in vals if v[0] is not None and (v[2] is None or math.isnan(v[2])))
        prem = [r for r in F if r["res"][key][0] is not None and r["res"][key][2] > 0]
        paths = collections.Counter(v[1] for v in vals)
        print(f"   {key:7} ascent deployments {len(prem):>3} (max {max((r['res'][key][2] for r in prem), default=0):+.1f}) | paths {dict(paths)} | missing velocity {nan}")
for p in ("P0", "qD0|A", "qD0|B"):
    by = collections.defaultdict(list)
    delay = []
    for r in F:
        t, path, v = r["res"][p]
        if t is not None:
            by[path].append(v)
            delay.append(t - (r["truth_apogee"] + 1000))
    q = lambda xs, f: sorted(xs)[min(len(xs) - 1, int(f * len(xs)))]
    print(f"   {p:6} velocity at command by path: " + "; ".join(f"{k} n={len(v)} min {min(v):+.1f} p10 {q(v, .1):+.1f} med {q(v, .5):+.1f} p90 {q(v, .9):+.1f} max {max(v):+.1f}" for k, v in sorted(by.items())))
    print(f"          delay vs (true apogee + 1000 ms): min {min(delay)} p10 {q(delay, .1)} median {q(delay, .5)} p90 {q(delay, .9)} max {max(delay)} ms; "
          f"later than +1 s: {sum(1 for x in delay if x > 1000)}")
for p in ("qD0|A", "qD0|B"):
    prem = [r for r in F if r["res"][p][0] is not None and r["res"][p][2] > 0]
    print(f"   residuals under {p}: {len(prem)}")
    for r in prem:
        print(f"      {r['name']:10} {r['label']:42} seed {r['seed']} {r['res'][p]} | first I {None if r['first']['qr3of5'] is None else r['first']['qr3of5'] - r['onset']} ms, D {None if r['first']['qD0'] is None else r['first']['qD0'] - r['onset']} ms, dD samples after onset {r['d_avail']}, accel invalid from {r['acc_bad']}")
    moved = collections.Counter(r["label"].split(" @")[0][:28] for r in F if r["res"]["qD0|A"][1] == "path2" and r["res"]["qD0|B"][1] == "timer")
    if p.endswith("B"):
        print(f"   runs moved from path 2 (A) to the timer (B): {sum(moved.values())} -> {dict(moved)}")
haz = [r for r in F if r["res"]["P0"][0] is not None and r["res"]["P0"][2] > 0]
det = lambda r: any(x is not None and r["apogee"] and x <= r["apogee"][0] for x in (r["first"]["qr3of5"], r["first"]["qD0"]))
print(f"   P0 hazardous runs {len(haz)}; flagged before the false APOGEE by I or D: {sum(det(r) for r in haz)}; pre-onset innovation alarms: {sum(r['pre']['qr3of5'] for r in F)}")
print(f"   runs with innovation and disagreement alarms on the same tick: {sum(1 for r in F if r['same_tick']['qD0'])} of {len(F)}")

print("\nD. CLEARANCE after the fault ends (REC cases): last raw alarm minus fault end (ms), innovation / disagreement")
for name in ("ref01", "zero_drag", "low_thrust"):
    for r_lab in sorted({r["label"] for r in F if r["label"].startswith("REC") and r["name"] == name}):
        rs = [r for r in F if r["label"] == r_lab and r["name"] == name]
        end = max(sp[2] for sp in rs[0]["specs"] if len(sp) > 2 and sp[2] is not None)
        li = [max([iv[1] for iv in r["int_I"] if iv[0] <= end + 2000] or [None], key=lambda x: -10**9 if x is None else x) for r in rs]
        ld = [max([iv[1] for iv in r["int_D"]["qD0"] if iv[0] <= end + 2000] or [None], key=lambda x: -10**9 if x is None else x) for r in rs]
        fi = [x - end for x in li if x is not None]
        fd = [x - end for x in ld if x is not None]
        print(f"   {name:10} {r_lab:34} I {min(fi) if fi else '-'}..{max(fi) if fi else '-'}  D {min(fd) if fd else '-'}..{max(fd) if fd else '-'}")

print("\nE. EDGE CASES (ballistic replay; drag-enabled twin = current code)")
for r in E:
    if r["drag"]:
        continue
    twin = next(x for x in E if x["drag"] and x["name"] == r["name"] and x["label"] == r["label"] and x["seed"] == r["seed"])
    al = (r["int_I"][:2], r["int_D"]["qD0"][:2])
    print(f"   {r['name']:10} seed {r['seed']} {r['label']:66} P0 {r['res']['P0'][:2]} A {r['res']['qD0|A'][:2]} B {r['res']['qD0|B'][:2]} | alarms I {al[0]} D {al[1]} | twin commands {twin['commands']} unique events {twin['events_unique']}")
print("   all drag-enabled twins: commands per run", dict(collections.Counter(x["commands"] for x in E if x["drag"])), "events unique:", all(x["events_unique"] for x in E))
