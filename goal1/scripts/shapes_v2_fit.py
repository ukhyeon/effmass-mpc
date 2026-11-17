"""shapes_v2_fit.py - fit collision shapes v2 (run: g1_run_human.sh py shapes_v2_fit.py). Shapes stay plain capsules in segment-local
coordinates (p_world = J_s + R_s @ p_local, same convention as human_shapes.ShapeSet), so the node pose path and the GPU kernel are unchanged.
Fit cloud: vertices of the B2a static pose and replay frames 0/300/600/900/1261, mapped into each segment's local frame and merged.
Layout: pelvis/abdomen/chest one left-right capsule each (chest stretched to shoulder width, no shoulder shapes); hands: palm row + finger row
capsules along the palm width + thumb; head = v1 PCA capsule; limbs/neck/feet = v1 axis with end points moved in by r.
Radius rule: r_over = largest r with over-cover median <= 0.5 cm (all surface samples, inside = 0, static-pose mesh); r_under = smallest r with
under-cover p95 <= 2.0 cm (fit cloud, own vertices). r = r_over if r_over >= r_under else r_under (under-cover has priority).
Hands: (r_palm, r_finger) = smallest sum with under p95 <= 2 cm and fingertip under p95 <= 1 cm. Output goal1/data/shapes_v2.json (21 + 3 inactive)."""
import json, os, sys
import numpy as np, torch
torch.set_num_threads(1)
from effmass_human.human_inertia_core import HumanInertiaCore
from effmass_human.utils import batch_roteulerSMPL
from effmass_human import human_shapes as HS
sys.path.insert(0, os.path.dirname(__file__))
H = os.path.expanduser("~"); D = f"{H}/effmass_mpc/goal1/data"
core = HumanInertiaCore("cpu"); core.set_betas(np.zeros(10, np.float32))
with torch.inference_mode():
    rz, _ = batch_roteulerSMPL(torch.zeros((1, 72))); out = core.smplh.forward(betas=core.betas, rotmat=rz)
VREST = out.vertices[0].numpy().astype(float); J24 = out.joint_smpl_shaped[0].numpy().astype(float); RJ19 = core.rest_joints_19[0].numpy().astype(float)
J52, WMAX = HS._hand_detail(VREST)
LAB = np.load(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy"); FACES = np.asarray(core.smplh.faces, np.int64)
# frames: static + replay
S = np.load(f"{D}/human_b2a_reach.npz"); SV = np.load(f"{D}/human_b2a_reach_verts.npz")["verts"]
R = np.load(f"{D}/human_precomputed_1781524848.npz"); RV = np.load(f"{D}/human_precomputed_1781524848_verts.npz", mmap_mode="r")["verts"]
FR = [("static", S["rot"][0], S["jprox"][0], SV[0])] + [(f"replay{f}", R["rot"][f], R["jprox"][f], np.asarray(RV[f])) for f in (0, 300, 600, 900, 1261)]
FR = [(n, rot.astype(float), J.astype(float), V.astype(float)) for n, rot, J, V in FR]


def local(s, ids=None):
    """vertices of segment s (or given vertex ids) in segment-local coordinates, all fit frames merged"""
    ids = np.nonzero(LAB == s)[0] if ids is None else ids
    return np.vstack([(V[ids] - J[s]) @ rot[s] for _, rot, J, V in FR])


def segdist(P, a, b):
    ab = b - a; L2 = float(ab @ ab); t = np.clip(((P - a) @ ab) / max(L2, 1e-12), 0, 1) if L2 > 1e-12 else np.zeros(len(P))
    return np.linalg.norm(P - (a + t[:, None] * ab), axis=1)


def under_p95(P, caps):
    ex = np.min(np.stack([segdist(P, a, b) - r for a, b, r in caps]), 0)
    return float(np.percentile(np.maximum(ex, 0), 95))


# ---- over-cover on the static mesh (winding number) ----
_, ROT0, J0, V0 = FR[0]; TRI0 = V0[FACES]


def winding(Q, TRI, chunk=256):
    out = np.empty(len(Q))
    for i in range(0, len(Q), chunk):
        q = Q[i:i + chunk]; A = TRI[None, :, 0] - q[:, None]; B = TRI[None, :, 1] - q[:, None]; C = TRI[None, :, 2] - q[:, None]
        a, b, c = np.linalg.norm(A, axis=2), np.linalg.norm(B, axis=2), np.linalg.norm(C, axis=2)
        num = np.einsum("nfi,nfi->nf", A, np.cross(B, C))
        den = a * b * c + np.einsum("nfi,nfi->nf", A, B) * c + np.einsum("nfi,nfi->nf", B, C) * a + np.einsum("nfi,nfi->nf", C, A) * b
        out[i:i + chunk] = np.arctan2(num, den).sum(1) / (2 * np.pi)
    return out


def cap_samples(a, b, r, n, rng):
    d = b - a; L = np.linalg.norm(d)
    u = d / L if L > 1e-9 else np.array([1.0, 0, 0])
    e1 = np.cross(u, [1, 0, 0]) if abs(u[0]) < 0.9 else np.cross(u, [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(u, e1)
    Ac, As = 2 * np.pi * r * L, 4 * np.pi * r * r; nc = int(round(n * Ac / (Ac + As))); ns = n - nc
    th = rng.uniform(0, 2 * np.pi, nc); t = rng.uniform(0, L, nc)
    cyl = a + np.outer(t, u) + r * (np.outer(np.cos(th), e1) + np.outer(np.sin(th), e2))
    dd = rng.normal(size=(ns, 3)); dd /= np.linalg.norm(dd, axis=1, keepdims=True)
    return np.vstack([cyl, np.where(((dd @ u) >= 0)[:, None], b, a) + r * dd])


from scipy.spatial import cKDTree


def mesh_tools(V):
    TRI = V[FACES]; VT = [[] for _ in range(len(V))]
    for fi, f in enumerate(FACES):
        for v in f:
            VT[v].append(fi)
    return TRI, VT, cKDTree(V)


def pt_tri_batch(P, T):
    """exact distance from points P (n,3) to triangles T (n,m,3,3), returns min over m (Ericson closest point, vectorized)"""
    a, b, c = T[..., 0, :], T[..., 1, :], T[..., 2, :]; p = P[:, None, :]
    ab, ac, ap = b - a, c - a, p - a; d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
    bp = p - b; d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1); cp = p - c; d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
    va = d3 * d6 - d5 * d4; vb = d5 * d2 - d1 * d6; vc = d1 * d4 - d3 * d2
    den = va + vb + vc; den = np.where(np.abs(den) < 1e-30, 1e-30, den)
    q = a + ab * (vb / den)[..., None] + ac * (vc / den)[..., None]
    sd = lambda x, y: np.where(np.abs(x - y) < 1e-30, 1e-30, x - y)
    cands = [(d1 <= 0) & (d2 <= 0), (d3 >= 0) & (d4 <= d3), (vc <= 0) & (d1 >= 0) & (d3 <= 0), (d6 >= 0) & (d5 <= d6), (vb <= 0) & (d2 >= 0) & (d6 <= 0),
             (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)]
    vals = [a, b, a + ab * (d1 / sd(d1, d3))[..., None], c, a + ac * (d2 / sd(d2, d6))[..., None], b + (c - b) * ((d4 - d3) / np.where(np.abs((d4 - d3) + (d5 - d6)) < 1e-30, 1e-30, (d4 - d3) + (d5 - d6)))[..., None]]
    done = np.zeros(d1.shape, bool)
    for m, v in zip(cands, vals):
        mm = m & ~done; q = np.where(mm[..., None], v, q); done |= mm
    return np.linalg.norm(q - p, axis=-1).min(1)


def mesh_dist(P, tools, k=12, m=64):
    TRI, VT, KD = tools; _, idx = KD.query(P, k=k)
    cand = np.zeros((len(P), m), np.int64)
    for i, ii in enumerate(idx):
        t = sorted({x for v in ii for x in VT[v]})[:m]; cand[i, :len(t)] = t; cand[i, len(t):] = t[0]
    return pt_tri_batch(P, TRI[cand])


TOOLS0 = mesh_tools(V0)


def over_stats(caps_world, TRI, tools, n=600, seed=0, others=()):
    rng = np.random.default_rng(seed); Q = np.vstack([cap_samples(a, b, r, n, rng) for a, b, r in caps_world])
    keep = np.ones(len(Q), bool)
    for a, b, r in list(caps_world) + list(others):
        keep &= ~(segdist(Q, a, b) < r - 1e-9)
    # samples inside another capsule of the set are internal surfaces
    Q = Q[keep]
    if not len(Q):
        return dict(n=0, outside_frac=0.0, median_all_m=0.0, p95_all_m=0.0, median_out_m=0.0)
    w = winding(Q, TRI); out = w < 0.5; d = np.zeros(len(Q))
    if out.any():
        d[out] = mesh_dist(Q[out], tools)
    return dict(n=int(len(Q)), outside_frac=float(out.mean()), median_all_m=float(np.median(d)), p95_all_m=float(np.percentile(d, 95)),
                median_out_m=float(np.median(d[out])) if out.any() else 0.0)


def to_world(s, caps_local, rot, J):
    return [(J[s] + rot[s] @ a, J[s] + rot[s] @ b, r) for a, b, r in caps_local]


# ---- under-cover against the union of all current shapes, per frame in world coordinates ----
VID = {s: np.nonzero(LAB == s)[0] for s in range(19)}


def under_union(s_list, cur, vid=None):
    """cur: dict seg -> list of local capsules. p95 over fit frames of max(0, distance outside the union) for vertices of segments s_list"""
    ex = []
    for _, rot, J, V in FR:
        caps = [c for t, cl in cur.items() for c in to_world(t, cl, rot, J)]
        P = np.vstack([V[VID[s] if vid is None else vid] for s in s_list])
        ex.append(np.min(np.stack([segdist(P, a, b) - r for a, b, r in caps]), 0))
    return float(np.percentile(np.maximum(np.concatenate(ex), 0), 95))


# ---- over-cover on the static mesh: outside the mesh, or inside the mesh but in another segment's region (intrusion) ----
KDV = cKDTree(V0); KDS = {s: cKDTree(V0[VID[s]]) for s in range(19)}


def over_intr(s, caps_local, own, n=400, seed=0):
    caps = to_world(s, caps_local, ROT0, J0); rng = np.random.default_rng(seed)
    Q = np.vstack([cap_samples(a, b, r, n, rng) for a, b, r in caps]); keep = np.ones(len(Q), bool)
    for a, b, r in caps:
        keep &= ~(segdist(Q, a, b) < r - 1e-9)
    Q = Q[keep]; w = winding(Q, TRI0); out = w < 0.5; d = np.zeros(len(Q))
    if out.any():
        d[out] = mesh_dist(Q[out], TOOLS0)
    ins = ~out
    if ins.any():
        _, iv = KDV.query(Q[ins]); lab = LAB[iv]; intr = ~np.isin(lab, own)
        if intr.any():
            di = np.min(np.stack([KDS[o].query(Q[ins][intr])[0] for o in own]), 0); dd = d[ins]; dd[intr] = di; d[ins] = dd
    return float(np.median(d))


def pick_r(make, s, own, cur, lo=0.004, hi=0.25, it=10):
    def und(r):
        c = dict(cur); c[s] = make(r); return under_union(own, c)
    if und(hi) > 0.02:
        r_under = hi
    else:
        a, b = lo, hi
        for _ in range(it + 2):
            m = 0.5 * (a + b); (b := m) if und(m) <= 0.02 else (a := m)
        r_under = b
    if over_intr(s, make(lo), own) > 0.005:
        r_over = 0.0
    else:
        a, b = lo, hi
        for _ in range(it):
            m = 0.5 * (a + b); (a := m) if over_intr(s, make(m), own) <= 0.005 else (b := m)
        r_over = a
    return (r_over if r_over >= r_under else r_under), r_over, r_under


v1 = core.shapes
CUR = {t: [(v1.p0[k].copy(), v1.p1[k].copy(), float(v1.radius[k])) for k in np.nonzero(v1.segs == t)[0]] for t in range(19)}   # start: v1
MAKERS = {}
for s in (1, 2, 4, 5, 6, 7, 9, 13, 14, 15, 16):
    k = np.nonzero(v1.segs == s)[0][0]; a0, b0 = v1.p0[k].copy(), v1.p1[k].copy()
    if HS.SPEC[s][0] == "tip":   # foot: axis parallel to ankle -> toe, moved to pass through the foot vertex centroid (fit cloud)
        u = (b0 - a0) / np.linalg.norm(b0 - a0); P = local(s); c = P.mean(0); c = c - ((c - a0) @ u) * u
        pr = (P - a0) @ u; a_end, b_end = c + u * pr.min(), c + u * pr.max()
    else:
        a_end, b_end = a0, b0
    u = (b_end - a_end) / np.linalg.norm(b_end - a_end); L = float(np.linalg.norm(b_end - a_end))

    def mk(r, a_end=a_end, b_end=b_end, u=u, L=L):
        if 2 * r >= L:
            m = 0.5 * (a_end + b_end); return [(m, m, r)]
        return [(a_end + u * r, b_end - u * r, r)]
    MAKERS[s] = ("limb-inset" + (" (foot axis through vertex centroid)" if HS.SPEC[s][0] == "tip" else ""), mk, [s])
for s, extra in ((0, ()), (3, ()), (8, (10, 11))):
    P = local(s); yc = 0.5 * (np.percentile(P[:, 1], 2) + np.percentile(P[:, 1], 98)); zc = 0.5 * (np.percentile(P[:, 2], 2) + np.percentile(P[:, 2], 98))
    Px = np.vstack([P] + [np.vstack([(V[LAB == e] - J[s]) @ rot[s] for _, rot, J, V in FR]) for e in extra]) if extra else P
    xlo, xhi = np.percentile(Px[:, 0], 1), np.percentile(Px[:, 0], 99)

    def mk(r, yc=yc, zc=zc, xlo=xlo, xhi=xhi):
        a = np.array([min(xlo + r, 0.5 * (xlo + xhi)), yc, zc]); b = np.array([max(xhi - r, 0.5 * (xlo + xhi)), yc, zc]); return [(a, b, r)]
    MAKERS[s] = ("torso left-right" + (" (to shoulder width)" if extra else ""), mk, [s] + list(extra))
for t in (10, 11):
    CUR[t] = []          # no shoulder shapes in v2
CUR[12] = CUR[12]        # head: v1
INFO = {}
for ps in range(2):      # two passes so that each shape sees its fitted neighbours
    for s in (0, 3, 8, 1, 2, 4, 5, 6, 7, 9, 13, 14, 15, 16):
        kind, mk, own = MAKERS[s]; r, ro, ru = pick_r(mk, s, own, CUR); CUR[s] = mk(r)
        INFO[f"{s}"] = dict(kind=kind, r=r, r_over=ro, r_under=ru, length=float(np.linalg.norm(CUR[s][0][1] - CUR[s][0][0])), own_vertices=own)
    print("pass", ps, {k: round(v["r"], 4) for k, v in INFO.items()}, flush=True)
INFO["12"] = dict(kind="v1 head", r=float(CUR[12][0][2]))
# ---- hands: palm row + finger row (along the palm width) + thumb; search (palm row position, r_palm, r_finger) ----
for s in (17, 18):
    wj, th = HS.THUMB[s]; ids = VID[s]; is_th = np.isin(WMAX[ids], th)
    Pn = local(s, ids[~is_th]); hj = 22 if s == 17 else 23; u = J24[hj] - J24[wj]; u /= np.linalg.norm(u)
    C = Pn - Pn.mean(0); w_, v_ = np.linalg.eigh(np.cov(C.T)); n = v_[:, 0]; n = n - (n @ u) * u; n /= np.linalg.norm(n); wv = np.cross(u, n)
    pu, pw, pn = Pn @ u, Pn @ wv, Pn @ n
    mcp = np.mean([(J52[c] - J24[wj]) @ u for c in HS.FINGER_CHAINS[s][:4]]); palm = pu < mcp; fing = ~palm; tipu = np.percentile(pu[fing], 99.5)
    np_c = np.median(pn[palm]); nf_c = np.median(pn[fing])
    wpl, wph = np.percentile(pw[palm], 2), np.percentile(pw[palm], 98); wfl, wfh = np.percentile(pw[fing], 2), np.percentile(pw[fing], 98)
    k = np.nonzero(v1.segs == s)[0][1]; ta, tb = v1.p0[k].copy(), v1.p1[k].copy(); tu = (tb - ta) / np.linalg.norm(tb - ta); tb_end = tb + tu * v1.radius[k]; tL = float(np.linalg.norm(tb_end - ta))
    tip_ids = ids[~is_th][np.repeat(True, len(pu) // len(FR) * 0 + 1)] if False else None

    def caps(up_c, rp, rf, rt):
        cp = (u * up_c + n * np_c + wv * min(wpl + rp, 0.5 * (wpl + wph)), u * up_c + n * np_c + wv * max(wph - rp, 0.5 * (wpl + wph)), rp)
        uf = tipu - rf
        cf = (u * uf + n * nf_c + wv * min(wfl + rf, 0.5 * (wfl + wfh)), u * uf + n * nf_c + wv * max(wfh - rf, 0.5 * (wfl + wfh)), rf)
        ct = (ta + tu * min(rt, 0.5 * tL), tb_end - tu * min(rt, 0.5 * tL), rt)
        return [cp, cf, ct]
    # fingertip vertices: non-thumb hand vertices within 2 cm of the most distal point along u (rest-pose ids, same in every frame)
    pu_rest = ((VREST[ids[~is_th]] - J24[wj]) @ u); tip_vid = ids[~is_th][pu_rest >= np.percentile(pu_rest, 99.5) - 0.02]
    best = None; rt = None
    for r_t in np.arange(0.006, 0.0301, 0.002):
        c = dict(CUR); c[s] = caps(0.05, 0.03, 0.03, r_t)
        if under_union([s], c, vid=ids[is_th]) <= 0.02:
            rt = float(r_t); break
    rt = 0.03 if rt is None else rt
    for up_c in np.arange(0.03, 0.1001, 0.01):
        for rp in np.arange(0.010, 0.0501, 0.0025):
            for rf in np.arange(0.008, 0.0501, 0.0025):
                if best is not None and rp + rf >= best[1] + best[2]:
                    break
                c = dict(CUR); c[s] = caps(up_c, rp, rf, rt)
                if under_union([s], c) <= 0.02 and under_union([s], c, vid=tip_vid) <= 0.01:
                    best = (float(up_c), float(rp), float(rf)); break
    CUR[s] = caps(*best, rt)
    INFO[f"{s}"] = dict(kind="hand palm row + finger row + thumb", palm_row_u=best[0], r_palm=best[1], r_finger=best[2], r_thumb=rt, knuckle_u=float(mcp), tip_u=float(tipu),
                        r_over_palm=over_intr(s, CUR[s][:1], [s]), axes_local=dict(u=u.tolist(), width=wv.tolist(), normal=n.tolist()))
    print("hand", s, INFO[f"{s}"]["r_palm"], INFO[f"{s}"]["r_finger"], rt, best[0], flush=True)
SHAPES = [(t, c) for t in range(19) for c in CUR[t]]
SHAPES.sort(key=lambda x: x[0])
OUT = dict(version="v2", n_active=len(SHAPES), n_padded=24,
           shapes=[dict(segment=int(s), p0_local=[float(x) for x in a], p1_local=[float(x) for x in b], radius=float(r)) for s, (a, b, r) in SHAPES],
           inactive=dict(segment=0, p0_local=[0.0, -1000.0, 0.0], p1_local=[0.0, -1000.0, 0.0], radius=0.0, note="padding to the fixed GPU size 24; 1 km away, radius 0"),
           info=INFO, fit_frames=[n for n, *_ in FR], rule=__doc__)
json.dump(OUT, open(f"{D}/shapes_v2.json", "w"), indent=1)
print("n_active", len(SHAPES)); [print(k, v) for k, v in INFO.items()]
