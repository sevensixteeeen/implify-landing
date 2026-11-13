"""Tables for E1, E3, E4, E5 and the healthy part of E7, from f1_healthy.json."""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
data = json.load(open(os.path.join(HERE, "f1_healthy.json"), encoding="utf-8"))
PH = ("PAD", "BOOST", "COAST", "NEAR_APOGEE", "DESCENT", "LANDED")
ASC = ("PAD", "BOOST", "COAST", "NEAR_APOGEE")


def group(res):
    name, seed, ideal, pad = res["job"]
    if pad:
        return "pad_ideal" if ideal else "pad_noisy"
    if ideal:
        return f"{name}/ideal"
    return f"{name}/{'train' if seed < 100 else 'valid'}"


def pool(metric, sel):
    agg = {}
    for res in data:
        g = group(res)
        if not sel(g):
            continue
        for ph, a in res["metrics"][metric].items():
            b = agg.setdefault(ph, dict(n=0, s=0.0, ss=0.0, max=0.0, where=None, p2=0.0, p3=0.0, lag_n=0, lag_s=0.0, run=0))
            b["n"] += a["n"]; b["s"] += a["s"]; b["ss"] += a["ss"]; b["lag_n"] += a["lag_n"]; b["lag_s"] += a["lag_s"]
            if a["max"] > b["max"]:
                b["max"], b["where"] = a["max"], (res["job"][0], res["job"][1], a["max_tick"])
            b["p2"] = max(b["p2"], a["p2"]); b["p3"] = max(b["p3"], a["p3"]); b["run"] = max(b["run"], a["run"])
    return agg


def stats(b):
    mean = b["s"] / b["n"]
    var = b["ss"] / b["n"] - mean * mean
    sd = math.sqrt(max(var, 0.0))
    rho = ((b["lag_s"] / b["lag_n"]) - mean * mean) / var if b["lag_n"] and var > 0 else float("nan")
    return mean, sd, rho


print("=" * 100)
print("E1  Innovation r = h_baro - h_predicted at every barometer correction (m)")
for scen in ("ref01", "zero_drag", "low_thrust"):
    for split in ("train", "valid"):
        agg = pool("r", lambda g, s=f"{scen}/{split}": g == s)
        print(f"-- {scen} seeds {'0-99' if split == 'train' else '100-199'}")
        print(f"   {'phase':12}{'n':>9}{'mean':>9}{'sd':>8}{'max|r|':>8}{'max/sd':>8}{'p2 max':>8}{'p3 max':>8}{'rho1':>7}{'run':>5}  worst (scenario, seed, tick)")
        for ph in PH:
            if ph in agg:
                b = agg[ph]; mean, sd, rho = stats(b)
                print(f"   {ph:12}{b['n']:>9}{mean:>+9.3f}{sd:>8.3f}{b['max']:>8.3f}{b['max'] / sd:>8.2f}{b['p2']:>8.3f}{b['p3']:>8.3f}{rho:>7.3f}{b['run']:>5}  {b['where']}")
for g in ("ref01/ideal", "zero_drag/ideal", "low_thrust/ideal", "pad_noisy", "pad_ideal"):
    agg = pool("r", lambda x, g=g: x == g)
    print(f"-- {g}: " + ", ".join(f"{ph} sd {stats(b)[1]:.4f} max {b['max']:.4f} mean {stats(b)[0]:+.4f}" for ph, b in agg.items() if ph in PH))

print("=" * 100)
print("E3  Longest run of identical consecutive RAW readings (samples), max over runs in the group")
for sensor in ("baro", "accel"):
    print(f"-- {sensor}")
    for g in ("ref01/train", "zero_drag/train", "low_thrust/train", "ref01/valid", "zero_drag/valid", "low_thrust/valid",
              "ref01/ideal", "zero_drag/ideal", "low_thrust/ideal", "pad_noisy", "pad_ideal"):
        best, pairs, same = {}, {}, {}
        for res in data:
            if group(res) != g:
                continue
            b, p, s = res["stuck"][sensor]
            for ph in b:
                best[ph] = max(best.get(ph, 0), b[ph])
                pairs[ph] = pairs.get(ph, 0) + p.get(ph, 0)
                same[ph] = same.get(ph, 0) + s.get(ph, 0)
        print(f"   {g:18}" + "  ".join(f"{ph[:8]} {best[ph]:>5} ({(same.get(ph, 0) / pairs[ph] if pairs.get(ph) else 0):.3f})" for ph in PH if ph in best))

for title, prefix in (("E4  Windowed dead-reckoning disagreement dD_W = mean5(D) - mean5(D W earlier), D = h_baro - h_accel_only (m)", "dD"),
                      ("E5  Barometric plausibility P_W = quadratic-fit acceleration - mean accel-derived acceleration (m/s^2)", "P")):
    print("=" * 100)
    print(title)
    for w in (0.2, 0.5, 1.0, 2.0):
        m = f"{prefix}{w}"
        print(f"-- W = {w} s")
        for g in ("ref01/train", "zero_drag/train", "low_thrust/train", "ref01/valid", "zero_drag/valid", "low_thrust/valid",
                  "ref01/ideal", "zero_drag/ideal", "low_thrust/ideal", "pad_noisy"):
            agg = pool(m, lambda x, g=g: x == g)
            print(f"   {g:18}" + "  ".join(f"{ph[:7]} sd {stats(b)[1]:6.3f} max {b['max']:7.3f} p3 {b['p3']:7.3f}" for ph, b in agg.items() if ph in ASC + ("DESCENT",)))

print("=" * 100)
print("E4  Absolute disagreement |D| (accumulated dead-reckoning drift + barometric scale error), max per phase (m)")
for g in ("ref01/train", "zero_drag/train", "low_thrust/train", "ref01/valid", "ref01/ideal", "zero_drag/ideal", "low_thrust/ideal", "pad_noisy"):
    agg = pool("absD", lambda x, g=g: x == g)
    print(f"   {g:18}" + "  ".join(f"{ph[:7]} {b['max']:8.2f}" for ph, b in agg.items() if ph in PH))
print("E4  Accelerometer-only dead reckoning error vs truth (h m, v m/s): max |.| and mean over the group")
for g in ("ref01/train", "zero_drag/train", "low_thrust/train", "ref01/ideal", "zero_drag/ideal", "low_thrust/ideal"):
    for ev in ("apogee", "touchdown"):
        vals = [res["dr"][ev] for res in data if group(res) == g and ev in res["dr"]]
        if vals:
            print(f"   {g:18} at {ev:9}: dh max {max(abs(v['dh']) for v in vals):8.2f} mean {sum(v['dh'] for v in vals) / len(vals):+8.2f} | "
                  f"dv max {max(abs(v['dv']) for v in vals):7.3f} mean {sum(v['dv'] for v in vals) / len(vals):+7.3f} | "
                  f"D max {max(abs(v['D']) for v in vals):8.2f}")

print("=" * 100)
print("Healthy regression sanity: event sources and non-NOMINAL ticks")
bad = [(res["job"], res["non_nominal_ticks"]) for res in data if res["non_nominal_ticks"]]
print(f"   flights with any non-NOMINAL estimate tick: {len(bad)} {bad[:5]}")
srcs = {}
for res in data:
    for e in res["events"]:
        srcs[(group(res).split('/')[0], e[0], e[2])] = srcs.get((group(res).split('/')[0], e[0], e[2]), 0) + 1
for k in sorted(srcs):
    print(f"   {k}: {srcs[k]}")
