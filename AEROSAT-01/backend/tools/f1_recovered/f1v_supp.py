import sys, json, collections
sys.path.insert(0, r'C:\Users\Harshit\AppData\Local\Temp\claude\e2d2b9e520211c21\c4cd98e5-d312-4cd6-a1fc-df3bdfbadf6b\scratchpad')
from multiprocessing import Pool
import f1v_run as V
def jobs():
    out = []
    for n, (B, A) in V.TRUTH.items():
        r20 = lambda t: int(round(t / 20.0)) * 20
        mid, late = r20(B + (A - B) / 2), r20(A - 3000)
        for when, tag in ((mid, 'mid-coast'), (late, 'apogee-3s')):
            for d in (-20, -50, -100, -200):
                specs = [('accel_invalid', B + 1000), ('baro_drift', when, None, d)]
                out += [(n, f'accel invalid @burnout+1s, drift {d} m/s @{tag}', specs, s, False) for s in range(10)]
    return out
if __name__ == '__main__':
    with Pool(11) as p:
        res = p.map(V.fault_job, jobs(), chunksize=2)
    g = collections.defaultdict(list)
    for r in res: g[(r['name'], r['label'])].append(r)
    print('runs', len(res))
    for (n, lab), rs in g.items():
        cells = []
        for pol in ('P0', 'P2', 'P3'):
            vs = [x['policies'][pol][2] for x in rs if x['policies'][pol][0] is not None]
            paths = collections.Counter(x['policies'][pol][1] for x in rs)
            cells.append(f"{pol} {sum(v > 0 for v in vs)}/{len(rs)} {max(vs):+.0f}..{min(vs):+.0f} {dict(paths)}" if vs else f"{pol} none")
        ap = [next((e for e in x['events'] if e[0] == 'APOGEE'), None) for x in rs]
        fr = [x['first_r'] for x in rs]
        before = sum(1 for x, a in zip(rs, ap) if x['first_r'] is not None and a is not None and x['first_r'] <= a[1])
        lat = [f - x['onset'] for f, x in zip(fr, rs) if f is not None]
        print(f"{n:10} {lab:50} APOGEE src/q {collections.Counter((a[2][:6], a[3][:5]) for a in ap if a)} | r first {min(lat) if lat else '-'}..{max(lat) if lat else '-'} ms, before APOGEE {before}/10, dD avail {max(x['d_avail_after_onset'] for x in rs)} | " + ' | '.join(cells))
