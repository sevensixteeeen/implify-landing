"""False-alarm counts per candidate threshold: flights whose healthy maximum exceeds threshold, per group and phase scope."""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
H = json.load(open(os.path.join(HERE, "f1_healthy.json"), encoding="utf-8"))
T = json.load(open(os.path.join(HERE, "f1_faults.json"), encoding="utf-8"))["T"]
SCOPES = {"ascent": ("PAD", "BOOST", "COAST", "NEAR_APOGEE"), "descent": ("DESCENT",), "landed": ("LANDED",)}


def grp(job):
    name, seed, ideal, pad = job
    if pad:
        return "pad-ideal" if ideal else "pad-noisy(0-99)"
    if ideal:
        return "ideal(3 scen)"
    return "train(0-99)" if seed < 100 else "VALID(100-199)"


rows = []
for key in ("r|p1", "r|p2", "r|p3", "dD0.2|p1", "dD0.2|p3", "dD0.5|p1", "dD0.5|p3", "dD1.0|p1", "dD2.0|p1", "absD|p1"):
    m, p = key.split("|")
    k = {"p1": "max", "p2": "p2", "p3": "p3"}[p]
    for margin in (1.0, 1.25, 1.5, 2.0):
        thr = T[key] * margin
        counts = {}
        worst = {}
        for res in H:
            g = grp(res["job"])
            for scope, phs in SCOPES.items():
                v = max((res["metrics"][m][ph][k] for ph in phs if ph in res["metrics"][m]), default=0.0)
                c = counts.setdefault((g, scope), [0, 0])
                c[1] += 1 if any(ph in res["metrics"][m] for ph in phs) else 0
                if v > thr:
                    c[0] += 1
                worst[(g, scope)] = max(worst.get((g, scope), 0.0), v)
        line = f"{key:10} x{margin:<4} thr {thr:7.3f} | "
        for g in ("train(0-99)", "VALID(100-199)", "ideal(3 scen)", "pad-noisy(0-99)", "pad-ideal"):
            parts = []
            for scope in ("ascent", "descent", "landed"):
                if counts.get((g, scope), [0, 0])[1]:
                    a, n = counts[(g, scope)]
                    parts.append(f"{scope[0]}{a}/{n}")
            line += f"{g}: {' '.join(parts):24} "
        print(line)
    print()
print("Largest healthy value per group/scope (the margin actually available):")
for key in ("r|p1", "r|p3", "dD0.2|p1", "dD0.5|p1"):
    m, p = key.split("|"); k = {"p1": "max", "p3": "p3"}[p]
    for scope, phs in SCOPES.items():
        vals = {}
        for res in H:
            g = grp(res["job"])
            v = max((res["metrics"][m][ph][k] for ph in phs if ph in res["metrics"][m]), default=None)
            if v is not None:
                vals[g] = max(vals.get(g, 0.0), v)
        print(f"   {key:9} {scope:8} " + "  ".join(f"{g} {v:.3f}" for g, v in vals.items()))
