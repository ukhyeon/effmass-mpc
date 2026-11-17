"""b2a_shape_check.py - shape check on the B2a static pose (system python3 -B, numpy/scipy/matplotlib only; report only, no shape change).
Under-cover: per segment, distance of mesh vertices outside the segment's capsules (p95 over outside vertices, outside fraction).
Over-cover: per capsule, area-uniform surface samples; keep samples outside the mesh (generalized winding number < 0.5) and measure
distance to the mesh surface (exact point-triangle distance over KD-tree candidate triangles). Median / p95 / fraction outside.
Right hand by direction: support distance of capsule union (22, 23) minus support of right-hand mesh vertices along tip (arm axis),
top (+z), outward side (person's right, horizontal); full hand and a local window (+-2 cm along the arm axis around the hand COM).
Torso alternatives are evaluated only as proposals. Figures: side and top orthographic views (mesh + capsule outlines, right-hand zoom)."""
import json, os, sys
import numpy as np
from scipy.spatial import cKDTree
H = os.path.expanduser("~"); D = f"{H}/effmass_mpc/goal1/data"; OUT = f"{H}/effmass_mpc/goal1/b2a"
S = np.load(f"{D}/human_b2a_reach.npz"); M = np.load(f"{D}/human_b2a_reach_verts.npz")
V = M["verts"][0].astype(np.float64); F = M["faces"]; P0, P1, RAD, SSEG = S["p0"][0].astype(float), S["p1"][0].astype(float), S["shape_radius"].astype(float), S["shape_segment"]
NAMES = list(S["segment_names"]); LAB = np.load(f"{OUT}/vertex_segment.npy")
rep = json.load(open(f"{OUT}/pose_report.json"))
TRI = V[F]                                                   # (F,3,3)


def segdist(P, a, b):
    ab = b - a; L2 = float(ab @ ab); t = np.clip(((P - a) @ ab) / max(L2, 1e-12), 0, 1)
    return np.linalg.norm(P - (a + t[:, None] * ab), axis=1)


def winding(Q, chunk=256):
    out = np.empty(len(Q))
    for i in range(0, len(Q), chunk):
        q = Q[i:i + chunk]; A = TRI[None, :, 0] - q[:, None]; B = TRI[None, :, 1] - q[:, None]; C = TRI[None, :, 2] - q[:, None]
        a, b, c = np.linalg.norm(A, axis=2), np.linalg.norm(B, axis=2), np.linalg.norm(C, axis=2)
        num = np.einsum("nfi,nfi->nf", A, np.cross(B, C))
        den = a * b * c + np.einsum("nfi,nfi->nf", A, B) * c + np.einsum("nfi,nfi->nf", B, C) * a + np.einsum("nfi,nfi->nf", C, A) * b
        out[i:i + chunk] = np.arctan2(num, den).sum(1) / (2 * np.pi)
    return out


VT = [[] for _ in range(len(V))]
for fi, f in enumerate(F):
    for v in f:
        VT[v].append(fi)
KD = cKDTree(V)


def pt_tri(p, T):
    # exact closest-point distance from p to triangles T (n,3,3) (Ericson, vectorized)
    a, b, c = T[:, 0], T[:, 1], T[:, 2]; ab, ac, ap = b - a, c - a, p - a
    d1, d2 = (ab * ap).sum(1), (ac * ap).sum(1); bp = p - b; d3, d4 = (ab * bp).sum(1), (ac * bp).sum(1); cp = p - c; d5, d6 = (ab * cp).sum(1), (ac * cp).sum(1)
    va = d3 * d6 - d5 * d4; vb = d5 * d2 - d1 * d6; vc = d1 * d4 - d3 * d2
    den = np.where(np.abs(va + vb + vc) < 1e-20, 1e-20, va + vb + vc); v = vb / den; w = vc / den; q = a + ab * v[:, None] + ac * w[:, None]
    m = (d1 <= 0) & (d2 <= 0); q[m] = a[m]
    m2 = (d3 >= 0) & (d4 <= d3); q[m2] = b[m2]
    m3 = (vc <= 0) & (d1 >= 0) & (d3 <= 0); t = d1 / np.where(np.abs(d1 - d3) < 1e-20, 1e-20, d1 - d3); q[m3 & ~m & ~m2] = (a + ab * t[:, None])[m3 & ~m & ~m2]
    m4 = (d6 >= 0) & (d5 <= d6); q[m4 & ~m & ~m2 & ~m3] = c[m4 & ~m & ~m2 & ~m3]
    m5 = (vb <= 0) & (d2 >= 0) & (d6 <= 0); t = d2 / np.where(np.abs(d2 - d6) < 1e-20, 1e-20, d2 - d6); mm = m5 & ~m & ~m2 & ~m3 & ~m4; q[mm] = (a + ac * t[:, None])[mm]
    m6 = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0); t = (d4 - d3) / np.where(np.abs((d4 - d3) + (d5 - d6)) < 1e-20, 1e-20, (d4 - d3) + (d5 - d6))
    mm = m6 & ~m & ~m2 & ~m3 & ~m4 & ~m5; q[mm] = (b + (c - b) * t[:, None])[mm]
    return np.linalg.norm(q - p, axis=1).min()


def mesh_dist(Q, k=24):
    _, idx = KD.query(Q, k=k)
    return np.array([pt_tri(q, TRI[sorted({t for v in ii for t in VT[v]})]) for q, ii in zip(Q, idx)])


def capsule_samples(a, b, r, n, rng):
    L = np.linalg.norm(b - a); u = (b - a) / max(L, 1e-12); e1 = np.cross(u, [1, 0, 0]) if abs(u[0]) < 0.9 else np.cross(u, [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(u, e1)
    Acyl, Asph = 2 * np.pi * r * L, 4 * np.pi * r * r; nc = int(round(n * Acyl / (Acyl + Asph))); ns = n - nc
    th = rng.uniform(0, 2 * np.pi, nc); t = rng.uniform(0, L, nc)
    cyl = a + np.outer(t, u) + r * (np.outer(np.cos(th), e1) + np.outer(np.sin(th), e2))
    d = rng.normal(size=(ns, 3)); d /= np.linalg.norm(d, axis=1, keepdims=True); side = (d @ u) >= 0
    sph = np.where(side[:, None], b, a) + r * d
    return np.vstack([cyl, sph])


def under(P, caps):
    ex = np.min(np.stack([segdist(P, a, b) - r for a, b, r in caps]), 0); o = ex > 0
    return dict(outside_frac=float(o.mean()), outside_p95_m=float(np.percentile(ex[o], 95)) if o.any() else 0.0)


def over(caps, n=1500, seed=0):
    rng = np.random.default_rng(seed); Q = np.vstack([capsule_samples(a, b, r, n, rng) for a, b, r in caps])
    # drop samples inside another capsule of the same set (internal surfaces)
    keep = np.ones(len(Q), bool)
    for a, b, r in caps:
        keep &= ~(segdist(Q, a, b) < r - 1e-9)
    Q = Q[keep]; w = winding(Q); out = w < 0.5; d = mesh_dist(Q[out]) if out.any() else np.zeros(0)
    return dict(n=int(len(Q)), outside_frac=float(out.mean()), median_m=float(np.median(d)) if len(d) else 0.0, p95_m=float(np.percentile(d, 95)) if len(d) else 0.0)


if __name__ == "__main__":  # main check
    res = {"winding_check": dict(mesh_vertex_count=int(len(V)), faces=int(len(F)), euler_char=int(len(V) - len(set(map(tuple, np.sort(np.vstack([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), 1).tolist()))) + len(F)),
                              w_far=float(winding(np.array([[10.0, 10, 10]]))[0]), w_pelvis_center=float(winding(V[LAB == 0].mean(0)[None])[0]))}
    seg = {}
    for s in range(19):
        ks = np.nonzero(SSEG == s)[0]; caps = [(P0[k], P1[k], RAD[k]) for k in ks]
        seg[NAMES[s]] = dict(shapes=ks.tolist(), radius=[round(RAD[k], 4) for k in ks], under=under(V[LAB == s], caps), over=over(caps, seed=s))
        print(NAMES[s], seg[NAMES[s]], flush=True)
    res["by_segment"] = seg
    # right hand by direction
    sh, el, wr, com = (np.array(rep[k]) for k in ("shoulder", "elbow", "wrist", "hand_com"))
    ax = wr - el; ax /= np.linalg.norm(ax); up = np.array([0, 0, 1.0]); fwd = np.array(rep["forward_node_world"]); right = np.cross(fwd, up); right /= np.linalg.norm(right)
    HV = V[LAB == 18]; caps = [(P0[k], P1[k], RAD[k]) for k in (22, 23)]
    def cap_support(v, win=None):
        best = -1e9
        for a, b, r in caps:
            if win is None:
                best = max(best, max(a @ v, b @ v) + r)
            else:   # support restricted to axial window: sample the axis densely
                t = np.linspace(0, 1, 2001); pts = a + np.outer(t, b - a); m = np.abs((pts - com) @ ax) <= win
                if m.any():
                    best = max(best, (pts[m] @ v).max() + r)
        return best
    dirs = {"tip (arm axis)": ax, "top (+z, back of hand)": up, "outward side (person right)": right}
    res["right_hand_direction"] = {}
    for nm, v in dirs.items():
        full = cap_support(v) - (HV @ v).max()
        loc_mask = np.abs((HV - com) @ ax) <= 0.02
        loc = cap_support(v, 0.02) - (HV[loc_mask] @ v).max() if nm != "tip (arm axis)" else None
        res["right_hand_direction"][nm] = dict(overhang_full_m=float(full), overhang_local_pm2cm_m=None if loc is None else float(loc),
                                               real_gap_if_capsule_gap_3cm_m=float(0.03 + (full if loc is None else loc)))
    json.dump(res, open(f"{OUT}/shape_check.json", "w"), indent=1)
    print(json.dumps(res["right_hand_direction"], indent=1)); print(res["winding_check"])
