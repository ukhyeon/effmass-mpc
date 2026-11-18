"""lv_tip_static.py <tag> <ik2_json> - live cmp tip: static gap of the IK poses along each candidate line to the target (fingertip hand shape /
elbow spheres, as lv_tip_analyze.py) of the representative posture (compare_tip/rep_<motion>.npz, moved by -shift), gap metric of the
live cmp tables (robot sphere + 0.02 buffer). Prints per job: IK success range, min gap over |s| <= 0.10 m, and over the whole line."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import b2a_analyze as BA
import shape_metrics as SM
_tg, _sub = (sys.argv[1].split("@") + ["compare_tip"])[:2]; T = os.path.expanduser(f"~/effmass_mpc/goal1/live/{_tg}/{_sub}"); P = {m["name"]: m for m in json.load(open(f"{T}/paths.json"))["motions"]}   # "tag@folder"
D = json.load(open(sys.argv[2])); s = np.array(D["s"]); res = []
for jb in D["jobs"]:
    m = P[jb["motion"]]; z = np.load(f"{T}/rep_{jb['motion'].replace(' ', '')}.npz"); sh = np.array(list(jb["shift"]) + [0.0])
    p0, p1, rad, seg = z["p0"] - sh, z["p1"] - sh, z["rad"], z["seg"]; su, sf = (13, 15) if m["side"] == "L" else (14, 16)
    fa = [k for k in range(len(rad)) if seg[k] == sf and rad[k] > 0][0]; ua = [k for k in range(len(rad)) if seg[k] == su and rad[k] > 0][0]
    ok = np.array(jb["success"], bool); Q = np.array(jb["q"]); g = np.full(len(s), np.nan)
    for i in np.nonzero(ok)[0]:
        C = BA.RI.compute(Q[i])[0]
        if m["target"] == "fingertip":
            k = m["target_shape"]; g[i] = float((SM.segdist(C, p0[k], p1[k]) - rad[k] - BA.RS).min())
        else:
            g[i] = min(float((np.linalg.norm(C - p0[fa], axis=1) - rad[fa] - BA.RS).min()), float((np.linalg.norm(C - p1[ua], axis=1) - rad[ua] - BA.RS).min()))
    nearc = np.abs(s) <= 0.10 + 1e-9
    r = dict(motion=jb["motion"], offset=jb["offset"], shift=jb["shift"], dz=jb.get("dz", 0.0), n_ok=int(ok.sum()), all_ok=bool(ok.all()),
             gap_centre_cm=None if np.all(np.isnan(g[nearc])) else round(100 * float(np.nanmin(g[nearc])), 1), gap_line_cm=None if np.all(np.isnan(g)) else round(100 * float(np.nanmin(g)), 1))
    res.append(r); print(r, flush=True)
json.dump(res, open(sys.argv[2].replace(".json", "_static.json"), "w"), indent=1)
