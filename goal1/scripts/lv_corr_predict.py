"""lv_corr_predict.py <tag> <motion> <ver> <mode> dL=a,b,... [dlat=..] [dz=..] - live3m corrected paths: gaps per pass predicted for a changed
path distance L (and lateral offset / height) from the robot records of an existing version's runs (mode off: the robot record does not
depend on the human; exact for dL and dlat, where only the base moves; for dz only a guide, the path height in the robot frame changes).
The human is translated by -(dL d_h + dlat u + dz z) in the base frame. Prints per value the per-pass gaps and angles."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lv_corr_lib as CL
import lv_corr_analyze as AN
tag, q, ver, mode = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]; kv = {x.split("=")[0]: [float(v) for v in x.split("=")[1].split(",")] for x in sys.argv[5:]}
T = f"{CL.GG}/live/{tag}/corrected/cal/m{q}{ver}"; P = json.load(open(f"{T}/path.json")); H = CL.Human(tag); d = np.array(P["d"]); dh = np.array([d[0], d[1], 0.0]); dh /= np.linalg.norm(dh); u = np.array(P["u"])
for dL in kv.get("dL", [0.0]):
    for dlat in kv.get("dlat", [0.0]):
        for dz in kv.get("dz", [0.0]):
            sh = -(dL * dh + dlat * u + np.array([0, 0, dz])); txt = []; allg = []
            for D in ("AB", "BA"):
                t = f"corr_{tag}_m{q}{ver}_{D}_{mode}"
                if os.path.exists(f"{CL.O}/{t}.log"):
                    o = AN.one(t, P, D, H, dh=sh); txt.append(D + ": " + " ".join(f"[{p['dir']} {100 * p['gap']:.1f}cm {p['angle_deg']:.0f}deg seg{p['seg']}/{p['link']}]" for p in o["passes"])); allg += [p["gap"] for p in o["passes"]]
            print(f"dL {dL:+.3f} dlat {dlat:+.3f} dz {dz:+.3f}: range {100 * min(allg):.1f}-{100 * max(allg):.1f} | " + " | ".join(txt), flush=True)
