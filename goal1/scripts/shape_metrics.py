"""shape_metrics.py - under-cover / over-cover of a capsule set against the node SMPL-H mesh (system python3, numpy/scipy only).
under(seg)  : p95 over the segment's vertices of max(0, distance outside the union of ALL shapes); also the outside fraction.
over(shape) : area-uniform samples on the capsule surface (internal parts inside another capsule of the same segment removed);
              distance outside the mesh (0 if inside the mesh); median and p95 over all samples, fraction outside;
              'intr' variant also counts samples inside the mesh but in another segment's region (distance to the nearest own vertex)."""
import numpy as np
from scipy.spatial import cKDTree


def segdist(P, a, b):
    ab = b - a; L2 = float(ab @ ab)
    t = np.clip(((P - a) @ ab) / L2, 0, 1) if L2 > 1e-12 else np.zeros(len(P))
    return np.linalg.norm(P - (a + t[:, None] * ab), axis=1)


def winding(Q, TRI, chunk=256):
    out = np.empty(len(Q))
    for i in range(0, len(Q), chunk):
        q = Q[i:i + chunk]; A = TRI[None, :, 0] - q[:, None]; B = TRI[None, :, 1] - q[:, None]; C = TRI[None, :, 2] - q[:, None]
        a, b, c = np.linalg.norm(A, axis=2), np.linalg.norm(B, axis=2), np.linalg.norm(C, axis=2)
        num = np.einsum("nfi,nfi->nf", A, np.cross(B, C))
        den = a * b * c + np.einsum("nfi,nfi->nf", A, B) * c + np.einsum("nfi,nfi->nf", B, C) * a + np.einsum("nfi,nfi->nf", C, A) * b
        out[i:i + chunk] = np.arctan2(num, den).sum(1) / (2 * np.pi)
    return out


def pt_tri_batch(P, T):
    a, b, c = T[..., 0, :], T[..., 1, :], T[..., 2, :]; p = P[:, None, :]
    ab, ac, ap = b - a, c - a, p - a; d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
    bp = p - b; d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1); cp = p - c; d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
    va = d3 * d6 - d5 * d4; vb = d5 * d2 - d1 * d6; vc = d1 * d4 - d3 * d2
    den = va + vb + vc; den = np.where(np.abs(den) < 1e-30, 1e-30, den)
    q = a + ab * (vb / den)[..., None] + ac * (vc / den)[..., None]
    sd = lambda x, y: np.where(np.abs(x - y) < 1e-30, 1e-30, x - y)
    cands = [(d1 <= 0) & (d2 <= 0), (d3 >= 0) & (d4 <= d3), (vc <= 0) & (d1 >= 0) & (d3 <= 0), (d6 >= 0) & (d5 <= d6), (vb <= 0) & (d2 >= 0) & (d6 <= 0),
             (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)]
    vals = [a, b, a + ab * (d1 / sd(d1, d3))[..., None], c, a + ac * (d2 / sd(d2, d6))[..., None],
            b + (c - b) * ((d4 - d3) / np.where(np.abs((d4 - d3) + (d5 - d6)) < 1e-30, 1e-30, (d4 - d3) + (d5 - d6)))[..., None]]
    done = np.zeros(d1.shape, bool)
    for m, v in zip(cands, vals):
        mm = m & ~done; q = np.where(mm[..., None], v, q); done |= mm
    return np.linalg.norm(q - p, axis=-1).min(1)


class Mesh:
    def __init__(self, V, F, lab):
        self.V = np.asarray(V, float); self.F = F; self.TRI = self.V[F]; self.lab = lab; self.KD = cKDTree(self.V)
        self.VT = [[] for _ in range(len(V))]
        for fi, f in enumerate(F):
            for v in f:
                self.VT[v].append(fi)
        self.KDS = {s: cKDTree(self.V[lab == s]) for s in np.unique(lab[lab >= 0])}

    def dist(self, P, k=12, m=64):
        """unsigned distance to the mesh surface (exact over KD-tree candidate triangles)"""
        P = np.asarray(P, float); out = np.empty(len(P))
        _, idx = self.KD.query(P, k=k)
        for i0 in range(0, len(P), 4096):
            ii = idx[i0:i0 + 4096]; cand = np.zeros((len(ii), m), np.int64)
            for j, row in enumerate(ii):
                t = sorted({x for v in row for x in self.VT[v]})[:m]; cand[j, :len(t)] = t; cand[j, len(t):] = t[0]
            out[i0:i0 + 4096] = pt_tri_batch(P[i0:i0 + 4096], self.TRI[cand])
        return out

    def inside(self, P):
        return winding(np.asarray(P, float), self.TRI) >= 0.5


def cap_samples(a, b, r, n, rng):
    d = b - a; L = np.linalg.norm(d); u = d / L if L > 1e-9 else np.array([1.0, 0, 0])
    e1 = np.cross(u, [1, 0, 0]) if abs(u[0]) < 0.9 else np.cross(u, [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(u, e1)
    Ac, As = 2 * np.pi * r * L, 4 * np.pi * r * r; nc = int(round(n * Ac / (Ac + As))); ns = n - nc
    th = rng.uniform(0, 2 * np.pi, nc); t = rng.uniform(0, L, nc)
    cyl = a + np.outer(t, u) + r * (np.outer(np.cos(th), e1) + np.outer(np.sin(th), e2))
    dd = rng.normal(size=(ns, 3)); dd /= np.linalg.norm(dd, axis=1, keepdims=True)
    return np.vstack([cyl, np.where(((dd @ u) >= 0)[:, None], b, a) + r * dd])


def under(P, caps):
    ex = np.min(np.stack([segdist(P, a, b) - r for a, b, r in caps if r > 0]), 0)
    return dict(p95_m=float(np.percentile(np.maximum(ex, 0), 95)), outside_frac=float((ex > 0).mean()), max_m=float(ex.max()))


def over(mesh, caps, own, n=800, seed=0):
    caps = [c for c in caps if c[2] > 0]
    if not caps:
        return None
    rng = np.random.default_rng(seed); Q = np.vstack([cap_samples(a, b, r, n, rng) for a, b, r in caps]); keep = np.ones(len(Q), bool)
    for a, b, r in caps:
        keep &= ~(segdist(Q, a, b) < r - 1e-9)
    Q = Q[keep]; ins = mesh.inside(Q); d = np.zeros(len(Q)); d[~ins] = mesh.dist(Q[~ins]) if (~ins).any() else 0
    di = d.copy()
    if ins.any():
        _, iv = mesh.KD.query(Q[ins]); intr = ~np.isin(mesh.lab[iv], own)
        if intr.any():
            x = np.min(np.stack([mesh.KDS[o].query(Q[ins][intr])[0] for o in own if o in mesh.KDS]), 0); t = di[ins]; t[intr] = x; di[ins] = t
    return dict(n=int(len(Q)), outside_frac=float((~ins).mean()), median_m=float(np.median(d)), p95_m=float(np.percentile(d, 95)),
                median_out_only_m=float(np.median(d[~ins])) if (~ins).any() else 0.0, median_with_intrusion_m=float(np.median(di)))
