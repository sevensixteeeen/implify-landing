"""Tables for E2, E6 and the injected-fault part of E7, from f1_faults.json."""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "f1_faults.json"), encoding="utf-8"))
T, STUCK, R = D["T"], D["stuck"], D["results"]
MARGIN = sys.argv[1] if len(sys.argv) > 1 else "1.5"
KEYS = ["r|p1|x1.0", "r|p3|x1.0", f"r|p1|x{MARGIN}", f"r|p3|x{MARGIN}", f"dD0.2|p1|x{MARGIN}", f"dD0.5|p1|x{MARGIN}",
        f"dD1.0|p1|x{MARGIN}", f"dD2.0|p1|x{MARGIN}", f"P0.2|p1|x{MARGIN}", f"P0.5|p1|x{MARGIN}", f"P1.0|p1|x{MARGIN}",
        f"absD|p1|x{MARGIN}"]
SHORT = {k: k.replace("|p1", "").replace("|x", "x") for k in KEYS}

print("Thresholds (observed healthy maxima, ascent phases, training seeds 0-99 + ideal + pad):")
for k, v in sorted(T.items()):
    print(f"   {k:12} {v:9.3f}")
print(f"   stuck run length max (noisy, ascent, in flight): {STUCK}")


def summarize(rs):
    out = dict(n=len(rs), premature=0, deadline=[], dep_v=[], landed_air=0, first_q=[], fp={}, det={}, before={})
    for r in rs:
        ap = next((e for e in r["events"] if e[0] == "APOGEE"), None)
        dep = next((e for e in r["events"] if e[0] == "DRAG_DEVICE_DEPLOYED"), None)
        td = next((e for e in r["events"] if e[0] == "TOUCHDOWN"), None)
        early = ap is not None and ap[4] > 0
        if early:
            out["premature"] += 1
            out["deadline"].append(ap[1] - r["onset"])
        out["dep_v"].append(dep[4] if dep else None)
        if td is not None and td[5] > 5.0:
            out["landed_air"] += 1
        q = next((t for t, s in r["quality"] if t >= r["onset"] and s != "NOMINAL"), None)
        out["first_q"].append(None if q is None else q - r["onset"])
        stuck_keys = [k for k in r["detections"] if k.startswith("stuck_")]
        for k in KEYS + stuck_keys:
            before, after = r["detections"][k]
            kk = k.split("|")[0] + "|" + k.split("|")[1] if k.startswith("stuck_") else k
            if before is not None:
                out["fp"][kk] = out["fp"].get(kk, 0) + 1
            lat = None if after is None else after - r["onset"]
            out["det"].setdefault(kk, []).append(lat)
            if early:
                out["before"].setdefault(kk, 0)
                if after is not None and after < ap[1]:
                    out["before"][kk] += 1
    return out


def fmt_lat(lats):
    got = [x for x in lats if x is not None]
    if not got:
        return "  none"
    miss = len(lats) - len(got)
    return f"{max(got):>6}" + (f"/{miss}m" if miss else "")


def table(exp):
    labels = []
    for r in R:
        if r["label"][0] == exp and r["label"][1] not in labels:
            labels.append(r["label"][1])
    stuck_cols = sorted({k.split("|")[0] + "|" + k.split("|")[1] for r in R for k in r["detections"] if k.startswith("stuck_")})
    cols = KEYS + stuck_cols
    print(f"{'case':40}{'early':>6}{'deadline':>9}{'dep v':>14}{'airLND':>7}{'Qlat':>6} | " + " ".join(f"{SHORT.get(k, k.replace('stuck_', 'st_')):>12}" for k in cols))
    for lab in labels:
        rs = [r for r in R if r["label"] == [exp, lab]]
        s = summarize(rs)
        dv = [v for v in s["dep_v"] if v is not None]
        dep = f"{min(dv):+.0f}..{max(dv):+.0f}" if dv else "none"
        if len(dv) < len(s["dep_v"]):
            dep += f" ({len(s['dep_v']) - len(dv)} no)"
        ql = [x for x in s["first_q"] if x is not None]
        cells = []
        for k in cols:
            c = fmt_lat(s["det"].get(k, []))
            if s["premature"]:
                c += f"[{s['before'].get(k, 0)}]"
            if s["fp"].get(k):
                c += f"F{s['fp'][k]}"
            cells.append(f"{c:>12}")
        print(f"{lab:40}{s['premature']:>3}/{s['n']:<2}{(min(s['deadline']) if s['deadline'] else '-'):>9}{dep:>14}{s['landed_air']:>7}"
              f"{(max(ql) if ql else '-'):>6} | " + " ".join(cells))


for exp, title in (("E2", "E2 barometer fault sweep, REF-01 seeds 0-4"), ("E6", "E6 fault replay, REF-01 seeds 0-9"),
                   ("E7", "E7 injected faults, ideal mode")):
    print("=" * 160)
    print(title)
    print("early = premature APOGEE (true v > 0) / runs; deadline = min ms from onset to that APOGEE; dep v = true velocity at deploy command;")
    print("airLND = TOUCHDOWN declared above 5 m true altitude; Qlat = max ms from onset to first non-NOMINAL quality;")
    print("detector cells: max latency ms over runs (/Nm = N runs missed), [k] = runs detected before the premature APOGEE, Fk = false alarms before onset")
    table(exp)
