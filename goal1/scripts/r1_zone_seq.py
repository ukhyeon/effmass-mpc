"""r1_zone_seq.py - zone surfaces along the R1 retract motion (system python3 + scikit-image): retract s = 0, 0.03, ..., 0.33 m; for each s the frame of
human_r1_reach_v025.npz whose s_retract is closest; uniform (eta 0.065) and ours (gain k 0.10); same grid as v1_zone.py, box moved with the hand COM.
Output goal1/zone/r1_seq/<cond>_<i>.npz and index.json (display only)."""
import json, os, sys
import numpy as np
from skimage.measure import marching_cubes
sys.path.insert(0, os.path.dirname(__file__))
import v2_zone_lib as ZL
H = os.path.expanduser("~"); HF = f"{H}/effmass_mpc/goal1/data/human_r1_reach_v025.npz"; OUT = f"{H}/effmass_mpc/goal1/zone/r1_seq"; os.makedirs(OUT, exist_ok=True)
sr = np.load(HF)["s_retract"]; S = np.round(np.arange(0, 0.331, 0.03), 3); idx = {"s": S.tolist(), "files": {"uniform": [], "ours": []}}
for i, sv in enumerate(S):
    f = int(np.argmin(np.abs(sr - sv))); D = ZL.load_frame(HF, f); c0 = D["COM"][18]
    lo = c0 - 0.35; hi = c0 + 0.35; hi[0] = max(hi[0], max(D["P0"][k][0] for k in D["SH"] if D["SEG"][k] == 16) + 0.05)
    xs, ys, zs = [np.arange(lo[a], hi[a] + 1e-9, 0.006) for a in range(3)]; X, Y, Z = np.meshgrid(xs, ys, zs, indexing="ij"); P = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    for cond, kk in (("uniform", 0.0), ("ours", 0.10)):
        fld, _, _ = ZL.field(D, P, "ours", kk); vt, fc, _, _ = marching_cubes(fld.reshape(X.shape), level=0.0, spacing=(0.006,) * 3); vt = vt + lo; _, mu, _ = ZL.field(D, vt, "ours", kk)
        fn = f"{OUT}/{cond}_{i:02d}.npz"; np.savez(fn, verts=vt.astype(np.float32), faces=fc.astype(np.int32), mu=mu.astype(np.float32), s=sv, frame=f); idx["files"][cond].append(fn)
    print(i, sv, f, flush=True)
json.dump(idx, open(f"{OUT}/index.json", "w"), indent=1)
