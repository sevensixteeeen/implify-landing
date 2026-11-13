import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, os.environ.get("F1F_OUT", "f1f_results.json")), encoding="utf-8"))
T, H, F = D["T"], D["healthy"], D["faults"]
sys.path.insert(0, HERE)
from f1f_run import STATS, COMBOS  # noqa: E402
STATS = [s for s in STATS if s in T]
COMBOS = {k: v for k, v in COMBOS.items() if all(x in T for x in v)}
POL = ["QA", "QB1", "QB3", "QB5", "QD", "QC3", "QAacc"]
section = sys.argv[1] if len(sys.argv) > 1 else "all"


def ds(job):
    n, s, ideal, pad = job
    if pad:
        return "pad-ideal" if ideal else "pad 0-99"
    if ideal:
        return "ideal (5 scen)"
    if n.startswith("high"):
        return f"{n} 0-99"
    return "train 0-99" if s < 100 else ("held 100-199" if s < 200 else "held 200-299")


if section in ("all", "healthy"):
    print("THRESHOLDS = 1.5 x training max (scope: calibration end .. timer+1 s)")
    print("   " + "  ".join(f"{k} {T[k]:.3f} (train max {T[k] / 1.5:.3f})" for k in STATS))
    order = ["train 0-99", "held 100-199", "held 200-299", "ideal (5 scen)", "pad 0-99", "pad-ideal", "high4k 0-99", "high8k 0-99"]
    print("\nPER-FLIGHT FALSE ALARMS (flights with any value above threshold) | held-out max / margin")
    mx = collections.defaultdict(float)
    cnt = collections.Counter()
    tot = collections.Counter()
    for h in H:
        d = ds(h["job"])
        tot[d] += 1
        for k in STATS:
            v = max((m[k][0] for m in h["scopes"].values() if k in m), default=0.0)
            mx[(d, k)] = max(mx[(d, k)], v)
            cnt[(d, k)] += v > T[k]
        for c, (a, b) in COMBOS.items():
            va = max((m[a][0] for m in h["scopes"].values() if a in m), default=0.0)
            vb = max((m[b][0] for m in h["scopes"].values() if b in m), default=0.0)
            cnt[(d, c)] += (va > T[a]) or (vb > T[b])
    print(f"{'dataset':16}{'n':>5} | " + " ".join(f"{k:>12}" for k in STATS))
    for d in order:
        print(f"{d:16}{tot[d]:>5} | " + " ".join(f"{cnt[(d, k)]:>3} {mx[(d, k)]:7.3f} " for k in STATS))
    print(f"{'dataset':16}{'n':>5} | " + " ".join(f"{c:>8}" for c in COMBOS))
    for d in order:
        print(f"{d:16}{tot[d]:>5} | " + " ".join(f"{cnt[(d, c)]:>8}" for c in COMBOS))
    held = ["held 100-199", "held 200-299"]
    print("\nmargin threshold / max(held-out 100-299): " + "  ".join(f"{k} {T[k] / max(mx[(d, k)] for d in held):.2f}" for k in STATS))
    print("margin threshold / max(high8k):           " + "  ".join(f"{k} {T[k] / mx[('high8k 0-99', k)]:.2f}" for k in STATS))
    # scope of the maxima
    sc = collections.Counter()
    for h in H:
        if ds(h["job"]).startswith(("held", "train")):
            for s_, m in h["scopes"].items():
                if "dmean" in m and m["dmean"][0] > 1.0:
                    sc[s_] += 1
    print("healthy flights with dmean > 1.0 by scope:", dict(sc))

if section in ("all", "faults"):
    def p0(r):
        return r["res"]["P0"]
    haz = [r for r in F if p0(r)[0] is not None and p0(r)[2] > 0]
    print(f"\nFAULT RUNS {len(F)}; P0 (current code) deploys with true v > 0 in {len(haz)}")
    print("DETECTION per statistic over the P0-hazardous runs: before false APOGEE / before P0 deployment / missed; min margin APOGEE - detection (ms); max latency")
    for k in STATS:
        ba = bd = miss = 0
        margins, lats = [], []
        for r in haz:
            f = r["first"][k]
            if f is None:
                miss += 1
                continue
            lats.append(f - r["onset"])
            if r["apogee"] and f <= r["apogee"][0]:
                ba += 1
                margins.append(r["apogee"][0] - f)
            if f < p0(r)[0]:
                bd += 1
        print(f"   {k:6} {ba:>5} / {bd:>5} / {miss:>4}   min margin {min(margins) if margins else '-':>6}   max latency {max(lats) if lats else '-'}")
    for c, (a, b) in COMBOS.items():
        ba = bd = 0
        for r in haz:
            fs = [x for x in (r["first"][a], r["first"][b]) if x is not None]
            f = min(fs) if fs else None
            if f is not None and r["apogee"] and f <= r["apogee"][0]:
                ba += 1
            if f is not None and f < p0(r)[0]:
                bd += 1
        print(f"   {c:7} before APOGEE {ba}/{len(haz)}, before P0 deploy {bd}/{len(haz)}")
    print("\nPRE-ONSET ALARMS in fault runs (false alarms):", {k: sum(1 for r in F if r["pre"][k]) for k in STATS})

    print("\nPOLICY OUTCOMES over all fault runs (premature = true v > 0 at the command)")
    base_ok = [r for r in F if p0(r)[0] is not None and p0(r)[2] <= 0]
    print(f"{'combo|policy':14}{'premature':>10}{'max v':>7}{'none':>6}{'later>5m/s':>11}{'min v':>7}  paths of premature")
    rows = []
    for c in ["P0"] + [f"{c}|{p}" for c in COMBOS for p in POL]:
        vals = [r["res"][c] for r in F]
        prem = [v for v in vals if v[0] is not None and v[2] > 0]
        none = sum(1 for v in vals if v[0] is None)
        later = sum(1 for r in base_ok if r["res"][c][0] is not None and r["res"][c][2] < p0(r)[2] - 5)
        mn = min(v[2] for v in vals if v[0] is not None)
        rows.append((c, len(prem), max((v[2] for v in prem), default=0), none, later, mn, dict(collections.Counter(v[1] for v in prem))))
    for row in rows:
        print(f"{row[0]:14}{row[1]:>10}{row[2]:>+7.0f}{row[3]:>6}{row[4]:>11}{row[5]:>+7.0f}  {row[6]}")

if section in ("all", "spikes"):
    print("\nSPIKES AND OUTLIERS: per case (10 seeds x scenario): P0 outcome, and per combo|policy the deployment path counts and true v range")
    show = ["Mz1|QA", "Mz1|QB3", "Mz3|QA", "Mz3|QB3", "Mzk|QB3", "Mz1med|QB3", "Mz3med|QB3", "Mz3|QC3"]
    g = collections.defaultdict(list)
    for r in F:
        if r["label"].startswith(("SPIKE", "DELAY", "REC")):
            g[(r["name"], r["label"])].append(r)
    for (n, lab), rs in sorted(g.items()):
        cells = []
        for c in ["P0"] + show:
            vs = [r["res"][c] for r in rs]
            pc = collections.Counter(v[1][:4] for v in vs)
            vv = [v[2] for v in vs if v[0] is not None]
            cells.append(f"{c}:{''.join(f'{k}{x}' for k, x in sorted(pc.items()))}[{max(vv):+.0f}..{min(vv):+.0f}]" if vv else f"{c}:none")
        print(f"{n:10} {lab:30} " + " ".join(cells))

if section in ("all", "residual"):
    which = sys.argv[2:] if len(sys.argv) > 2 else ["Mz3|QB3", "Mz3|QC3", "Mz1|QB3", "Mz1|QA", "Mzk|QB3"]
    for c in which:
        prem = [r for r in F if r["res"][c][0] is not None and r["res"][c][2] > 0]
        print(f"\nRESIDUAL premature under {c}: {len(prem)}")
        g = collections.Counter((r["name"], r["label"], r["res"][c][1]) for r in prem)
        for k, n in sorted(g.items()):
            ex = [r for r in prem if (r["name"], r["label"], r["res"][c][1]) == k]
            vs = [r["res"][c][2] for r in ex]
            a, b = COMBOS[c.split("|")[0]]
            fa = [None if r["first"][a] is None else r["first"][a] - r["onset"] for r in ex]
            fb = [None if r["first"][b] is None else r["first"][b] - r["onset"] for r in ex]
            print(f"   {k[0]:10} {k[1]:40} {k[2]:6} n={n:>2} v {max(vs):+.1f}..{min(vs):+.1f} seeds {[r['seed'] for r in ex]} | first {a} {fa[:4]} {b} {fb[:4]} dD avail {max(r['d_avail'] for r in ex)} acc_bad {ex[0]['acc_bad']}")
    # relevance window evidence
    print("\nRELEVANCE WINDOW: over P0-hazardous runs, APOGEE tick minus LAST alarm before it, and minus FIRST alarm after onset (ms), for Mz3")
    haz = [r for r in F if r["res"]["P0"][0] is not None and r["res"]["P0"][2] > 0 and r["apogee"]]
    gaps = []
    for r in haz:
        fs = [x for x in (r["first"]["z3"], r["first"]["dmean"]) if x is not None]
        if fs and min(fs) <= r["apogee"][0]:
            gaps.append((r["apogee"][0] - min(fs), r["name"], r["label"]))
    gaps.sort(reverse=True)
    print("   largest APOGEE - first alarm:", gaps[:8])
