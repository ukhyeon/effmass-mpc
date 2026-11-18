"""pr_zone_mesh.py - (part) proposed (per-part reference) zone surface of the B2a reach posture, same grid, shapes and definitions as
v2_zone_mesh.py / v1_zone.py (right-arm v2 shapes, robot point mass 4.6 kg), except mu_ref = mu_iso of each shape's segment
(1 / (1/m_iso + 1/m_r), goal1/data/iso_15066.json). Output goal1/part_regen/zone_part_mesh.npz (world frame of scene g2, vertex mu, eta)."""
import os, sys
import numpy as np
from skimage.measure import marching_cubes
sys.path.insert(0, os.path.dirname(__file__))
import v2_zone_lib as ZL
import g1_meff_gpu as G
MISO = G.iso_mass_19(); K = 0.10


def field_part(D, P):
    fb = np.full(len(P), np.inf); mub = np.zeros(len(P)); eb = np.zeros(len(P))
    for s in D["SH"]:
        a, b = D["P0"][s], D["P1"][s]; ab = b - a; tt = np.clip(((P - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = a + tt[:, None] * ab; v = q - P; n = np.linalg.norm(v, axis=1)
        d = n - D["RAD"][s]; u = v / np.maximum(n, 1e-9)[:, None]; g = D["SEG"][s]
        mh = 1.0 / np.maximum(np.einsum("ni,ij,nj->n", u, D["LAM"][g], u), 1e-12); mu = 1.0 / (1.0 / mh + 1.0 / ZL.MROB)
        ref = 1.0 / (1.0 / MISO[g] + 1.0 / ZL.MROB)
        eta = np.clip(ZL.DB + K * np.maximum(np.sqrt(mu / ref) - 1, 0), ZL.DB, ZL.EM)
        f = d - eta; m = f < fb; fb[m] = f[m]; mub[m] = mu[m]; eb[m] = eta[m]
    return fb, mub, eb


D = ZL.load(os.path.expanduser("~/effmass_mpc/goal1/data/human_b2a_reach_v2.npz")); c0 = D["COM"][18]
lo = c0 - 0.35; hi = c0 + 0.35; hi[0] = max(hi[0], max(D["P0"][k][0] for k in D["SH"] if D["SEG"][k] == 16) + 0.05)
xs, ys, zs = [np.arange(lo[i], hi[i] + 1e-9, 0.005) for i in range(3)]; X, Y, Z = np.meshgrid(xs, ys, zs, indexing="ij"); P = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
f, _, _ = field_part(D, P); vt, fc, _, _ = marching_cubes(f.reshape(X.shape), level=0.0, spacing=(0.005, 0.005, 0.005)); vt = vt + lo; _, mu, eta = field_part(D, vt)
out = os.path.expanduser("~/effmass_mpc/goal1/part_regen/zone_part_mesh.npz")
np.savez(out, verts=vt.astype(np.float32), faces=fc.astype(np.int32), mu=mu.astype(np.float32), eta=eta.astype(np.float32), note="proposed (per-part reference) zone, k_gain 0.10")
print(out, len(vt), "eta range %.3f-%.3f" % (eta.min(), eta.max()))
