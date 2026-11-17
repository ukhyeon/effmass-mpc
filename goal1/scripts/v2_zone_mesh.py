"""v2_zone_mesh.py <k> [k ...] - ours zone surface for other k_gain values (system python3 + scikit-image), same grid and definitions as v1_zone.py.
Output goal1/zone/zone_gk<k*100>_mesh.npz (world frame, vertex mu)."""
import os, sys
import numpy as np
from skimage.measure import marching_cubes
sys.path.insert(0, os.path.dirname(__file__))
import v2_zone_lib as ZL
D = ZL.load(os.path.expanduser("~/effmass_mpc/goal1/data/human_b2a_reach_v2.npz")); c0 = D["COM"][18]
lo = c0 - 0.35; hi = c0 + 0.35; hi[0] = max(hi[0], max(D["P0"][k][0] for k in D["SH"] if D["SEG"][k] == 16) + 0.05)
xs, ys, zs = [np.arange(lo[i], hi[i] + 1e-9, 0.005) for i in range(3)]; X, Y, Z = np.meshgrid(xs, ys, zs, indexing="ij"); P = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
for k in [float(x) for x in sys.argv[1:]]:
    f, _, _ = ZL.field(D, P, "ours", k); vt, fc, _, _ = marching_cubes(f.reshape(X.shape), level=0.0, spacing=(0.005, 0.005, 0.005)); vt = vt + lo; _, mu, _ = ZL.field(D, vt, "ours", k)
    out = os.path.expanduser(f"~/effmass_mpc/goal1/zone/zone_gk{int(round(k * 100)):03d}_mesh.npz")
    np.savez(out, verts=vt.astype(np.float32), faces=fc.astype(np.int32), mu=mu.astype(np.float32), note=f"ours zone, k_gain {k}"); print(out, len(vt))
