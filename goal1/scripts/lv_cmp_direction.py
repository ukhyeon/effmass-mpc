"""lv_cmp_direction.py <tag> <L|R> - live cmp: per motion (seed 0 of each mode), the approach direction of the robot to the hand and forearm of one arm
(cos of the vector shape -> nearest robot sphere with the forearm axis elbow -> wrist, at steps with a gap < 15 cm, every 3rd step), the zone
thickness and mu there (b1 log), and the arm pose (elbow bend, shoulder to hand). Writes compare/direction_<side>.json."""
import sys, json, numpy as np
sys.path.insert(0, '/home/robotics/effmass_mpc/goal1/scripts')
import lv_cmp_analyze as A, b2a_analyze as BA, shape_metrics as SM
C = f'/home/robotics/effmass_mpc/goal1/live/{sys.argv[1]}/compare'; cfg = json.load(open(f'{C}/runs.json')); an = json.load(open(f'{C}/analysis.json'))
H = A.human(cfg); out = {}
SIDE = sys.argv[2]; HS, FS, US = (17, 15, 13) if SIDE == 'L' else (18, 16, 14)
for mode in ('part', 'uniform'):
    R = A.run_metrics(cfg['runs'][mode][0], H, H['F'])
    for e in an['intervals']:
        if e['t0_s'] is None: continue
        s = np.arange(int(e['t0_s'] * 60), min(int(e['t1_s'] * 60), R['n']))
        rows = []
        for i in s[::3]:
            f = min(int((i + 1) // 2), H['F'] - 1); Cs = BA.RI.compute(R['Q'][i])[0]
            for p, segs in (('hand', (HS,)), ('forearm', (FS,))):
                ks = [k for k in range(len(H['rad'])) if H['rad'][k] > 0 and H['seg'][k] in segs]
                best = None
                for k in ks:
                    a, b = H['P0'][f][k], H['P1'][f][k]; d = SM.segdist(Cs, a, b) - H['rad'][k] - BA.RS; j = int(np.argmin(d))
                    if best is None or d[j] < best[0]: best = (d[j], k, j, a, b)
                g, k, j, a, b = best
                if g > 0.15: continue
                fa = [kk for kk in range(len(H['rad'])) if H['seg'][kk] == FS and H['rad'][kk] > 0][0]
                ax = H['P1'][f][fa] - H['P0'][f][fa]; ax /= np.linalg.norm(ax)          # forearm axis, elbow -> wrist
                ab = b - a; t = np.clip((Cs[j] - a) @ ab / max(ab @ ab, 1e-12), 0, 1); v = Cs[j] - (a + t * ab); v /= np.linalg.norm(v)
                el = H['P0'][f][fa]; pass
                eta, mu = A.eta_of(R, i, k, int(H['seg'][k]))
                rows.append((p, g, float(v @ ax), eta, mu))
        for p in ('hand', 'forearm'):
            r = [x for x in rows if x[0] == p]
            if r:
                g = np.array([x[1] for x in r]); c = np.array([x[2] for x in r]); et = np.array([x[3] for x in r]); mu = np.array([x[4] for x in r])
                out[f"{mode}/{e['name']}/{p}"] = dict(n=len(r), gap_min_cm=round(100 * g.min(), 1), cos_axis_median=round(float(np.median(c)), 2),
                    share_front=round(float((c > 0.7).mean()), 2), eta_cm_median=round(100 * float(np.median(et)), 1), eta_cm_max=round(100 * float(et.max()), 1), mu_median=round(float(np.median(mu)), 2))
# right arm extension per motion (shoulder joint ~ upper arm p0 to hand distal), median
ua = [k for k in range(len(H['rad'])) if H['seg'][k] == US][0]; hd = [k for k in range(len(H['rad'])) if H['seg'][k] == HS]
for e in an['intervals']:
    if e['t0_s'] is None: continue
    fs = range(int(e['t0_s'] * 30), int(e['t1_s'] * 30))
    L = [max(np.linalg.norm(H['P1'][f][k] - H['P0'][f][ua]) for k in hd) for f in fs]
    el = [float(np.degrees(np.arccos(np.clip(((H['P1'][f][ua] - H['P0'][f][ua]) / np.linalg.norm(H['P1'][f][ua] - H['P0'][f][ua])) @ ((H['P1'][f][ua + 2] - H['P0'][f][ua + 2]) / np.linalg.norm(H['P1'][f][ua + 2] - H['P0'][f][ua + 2])), -1, 1)))) for f in fs]
    out[f"pose/{e['name']}"] = dict(shoulder_to_hand_m=round(float(np.median(L)), 3), elbow_bend_deg=round(float(np.median(el)), 0))
json.dump(out, open(f'{C}/direction_{SIDE}.json', 'w'), indent=1)
for k, v in out.items(): print(k, v)
