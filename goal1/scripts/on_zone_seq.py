"""on_zone_seq.py <human npz> <out dir> <n keys> - zone surfaces (display only, same as r1_zone_seq.py) for any precomputed human file: key values =
n evenly spaced values of the file's s_retract (path coordinate; for a static file one key, frame 0); per key the closest frame; uniform (eta 0.065)
and ours (gain k 0.10); grid 6 mm in a box around the hand COM (+-0.35 m, extended to the forearm). Output <out>/<cond>_<i>.npz, index.json
(with "human" = the file, used by g1_viz --viz-zone-seq), and <out>/<cond>_static.npz for a static file (for --viz-zone)."""
import json, os, sys
import numpy as np
from skimage.measure import marching_cubes
sys.path.insert(0, os.path.dirname(__file__))
import v2_zone_lib as ZL
HF, OUT, NK = sys.argv[1], sys.argv[2], int(sys.argv[3]); os.makedirs(OUT, exist_ok=True)
z = np.load(HF); sr = z["s_retract"] if "s_retract" in z.files else np.zeros(len(z["p0"]))
S = np.round(np.linspace(sr.min(), sr.max(), NK), 3) if NK > 1 else np.array([float(sr[0])]); idx = {"s": S.tolist(), "files": {"uniform": [], "ours": []}, "human": HF}
for i, sv in enumerate(S):
    f = int(np.argmin(np.abs(sr - sv))); D = ZL.load_frame(HF, f); c0 = D["COM"][18]; fore = np.array([D["P0"][k] for k in D["SH"] if D["SEG"][k] == 16] + [D["P1"][k] for k in D["SH"] if D["SEG"][k] == 16])
    lo = np.minimum(c0 - 0.35, fore.min(0) - 0.12); hi = np.maximum(c0 + 0.35, fore.max(0) + 0.12)
    xs, ys, zs = [np.arange(lo[a], hi[a] + 1e-9, 0.006) for a in range(3)]; X, Y, Z = np.meshgrid(xs, ys, zs, indexing="ij"); P = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    for cond, kk in (("uniform", 0.0), ("ours", 0.10)):
        fld, _, _ = ZL.field(D, P, "ours", kk); vt, fc, _, _ = marching_cubes(fld.reshape(X.shape), level=0.0, spacing=(0.006,) * 3); vt = vt + lo; _, mu, _ = ZL.field(D, vt, "ours", kk)
        fn = f"{OUT}/{cond}_{i:02d}.npz" if NK > 1 else f"{OUT}/{cond}_static.npz"
        np.savez(fn, verts=vt.astype(np.float32), faces=fc.astype(np.int32), mu=mu.astype(np.float32), s=sv, frame=f); idx["files"][cond].append(fn)
    print(i, sv, f, flush=True)
json.dump(idx, open(f"{OUT}/index.json", "w"), indent=1)
