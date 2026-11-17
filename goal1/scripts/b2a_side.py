"""b2a_side.py - which side the robot passes the hand on (from saved records; g1_run_human.sh py).
At the shape-based hand minimum inside the pass window (TCP within +-0.12 m of the hand COM along y): nearest robot sphere centre c, nearest
point on the v2 hand shape surface p; n = (c - p)/|c - p|; side = largest of up (n_z), down (-n_z), front (n . a, a = arm axis toward the robot),
side (|n_y|). Also u (nearest definition) vs frontal (-a) angle, mu at that pair (B1 record), and the maximum TCP height over the hand top in the window.
Usage: b2a_side.py <tag_prefix> <out.json>   (tags <prefix>_<path>_<cond>_s<seed>)"""
import json, os, re, csv, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
pref, outp = sys.argv[1], sys.argv[2]
HY = json.load(open(f"{BA.G}/lines.json"))["hand_com"][1]; HTOP = max(max(BA.P0[k][2], BA.P1[k][2]) + BA.RAD[k] for k in BA.HAND)
res = []
for f in sorted(os.listdir(BA.O)):
    m = re.match(rf"{re.escape(pref)}_(\w+?)_(off|iso|massA|uni10|unim)_s(\d)_b1\.npz$", f)
    if not m:
        continue
    path, cond, seed = m.group(1), m.group(2), int(m.group(3)); tag = f[:-7]
    r = BA.result(tag); rows = list(csv.DictReader(open(r["steps_csv"]))); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows])
    B = np.load(f"{BA.O}/{f}")["rows"]; tcp = B[:, 8:11] - np.array([0, 0, 0.1034]); w = np.abs(tcp[:, 1] - HY) <= 0.12
    D = B[:, 12:36][:, BA.HAND]; Dw = np.where(w[:, None], D, np.inf); i, jj = np.unravel_index(int(np.argmin(Dw)), Dw.shape); k = BA.HAND[jj]
    s = int(B[i, 36 + k]); c = BA.RI.compute(Q[i])[0][s]
    ab = BA.P1[k] - BA.P0[k]; t = np.clip(((c - BA.P0[k]) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = BA.P0[k] + t * ab; n = (c - q) / np.linalg.norm(c - q)
    comp = {"up": n[2], "down": -n[2], "front": float(n @ BA.AX), "side": abs(n[1])}; side = max(comp, key=comp.get)
    u, _, _ = BA.shape_u(c); ang = float(np.degrees(np.arccos(np.clip(u @ (-BA.AX), -1, 1))))
    res.append(dict(tag=tag, path=path, cond=cond, seed=seed, side=side, comp={k_: round(float(v), 3) for k_, v in comp.items()}, u_vs_front_deg=ang, mu=float(B[i, 60 + k]),
                    sphere=BA.RI.sphere_link[s], gap_shape=float(D[i, jj]), step=int(i), tcp_max_above_hand_top=float(tcp[w, 2].max() - HTOP) if w.any() else None))
json.dump(res, open(outp, "w"), indent=1)
for p in sorted({r["path"] for r in res}):
    for c in ("off", "iso", "massA", "uni10", "unim"):
        rs = [r for r in res if r["path"] == p and r["cond"] == c]
        if rs:
            cnt = {}
            for r in rs: cnt[r["side"]] = cnt.get(r["side"], 0) + 1
            print(f"{p:10s} {c:6s} n {len(rs)} sides {cnt}  u_vs_front mean {np.mean([r['u_vs_front_deg'] for r in rs]):5.1f} [{min(r['u_vs_front_deg'] for r in rs):5.1f},{max(r['u_vs_front_deg'] for r in rs):5.1f}]"
                  f"  mu mean {np.mean([r['mu'] for r in rs]):.2f} [{min(r['mu'] for r in rs):.2f},{max(r['mu'] for r in rs):.2f}]  TCP max above hand top {np.mean([r['tcp_max_above_hand_top'] for r in rs])*100:5.1f} cm")
