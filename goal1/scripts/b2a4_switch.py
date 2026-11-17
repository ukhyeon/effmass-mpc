"""b2a4_switch.py - offline: straight front lines at hand height (IK grid b2a4_ik.json, |dy| <= 0.12), for TCP x 0.38 .. 0.46: minimum shape gap to the
v2 hand shapes over the line, nearest robot sphere, u (nearest definition) vs frontal angle (system python via g1_run_human.sh py)."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import shape_metrics as SM
IK = json.load(open(f"{BA.G}/b2a4_ik.json")); dy = np.array(IK["dy"]); out = []
for key in [f"{x:.2f}" for x in np.arange(0.38, 0.465, 0.01)]:
    d = IK["front"][key]; best = None
    for i in np.nonzero(np.abs(dy) <= 0.12)[0]:
        if not d["success"][i]:
            continue
        C = BA.RI.compute(np.array(d["q"][i]))[0]
        G_ = np.stack([SM.segdist(C, BA.P0[k], BA.P1[k]) - BA.RAD[k] for k in BA.HAND], 1) - BA.RS[:, None]; j, kk = np.unravel_index(np.argmin(G_), G_.shape)
        if best is None or G_[j, kk] < best[0]:
            best = (float(G_[j, kk]), int(j), C[j], float(dy[i]))
    g, j, c, y = best; u, _, _ = BA.shape_u(c)
    # second candidate: nearest finger sphere vs nearest hand-body sphere separately at that pose
    out.append(dict(tcp_x=float(key), min_gap=g, sphere=BA.RI.sphere_link[j], dy=y, u_vs_front_deg=float(np.degrees(np.arccos(np.clip(u @ (-BA.AX), -1, 1))))))
    print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in out[-1].items()}))
json.dump(out, open(f"{BA.G}/b2a4_switch.json", "w"), indent=1)
