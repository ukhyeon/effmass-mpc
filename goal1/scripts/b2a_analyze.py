"""b2a_analyze.py - B2a analysis from saved run records only (run with g1_run_human.sh py; no new runs).
Per run: shape-based gaps (B1 rows: hand / forearm / torso), mesh-based hand gap (static node SMPL-H mesh, scene g2 transform, MPC sphere
radius yml + 0.02), nearest robot sphere, u (nearest definition, COM fallback < 1 mm) vs the arm axis, m_h, m_r, mu, eta; pass time
(arrival at B), EE path length, command change, infeasible, nearest-u fallback counts (executed and rollout), rollout cost shares per segment.
Contact risk (relative, m/s): risk_t = P_t * v_n * sqrt(mu / 2.0), P_t = P(mesh gap <= 0) under a rigid translation of the hand mesh by an
isotropic Gaussian error (sigma 0.02, 0.04 m, 1000 samples, the same samples for the same seed number in every condition), gap per sphere
linearised in the translation (g_s - n_s . delta, n_s = gradient of the signed distance); v_n = approach speed of the nearest sphere along u_mesh
(positive when approaching); mu = nearest-definition mu at that sphere. Max and sum over the run.
Usage: b2a_analyze.py runs | check_linear <tag> | summary"""
import json, os, re, csv, sys
import numpy as np
from scipy.spatial import cKDTree
from scipy.stats import binomtest
sys.path.insert(0, os.path.dirname(__file__))
import shape_metrics as SM
from effmass_robot.robot_inertia_np import RobotInertiaNP
H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; G = f"{H}/effmass_mpc/goal1/b2a/g2"; OUT = f"{H}/effmass_mpc/goal1/b2a"
tf = json.load(open(f"{G}/temp_transform.json")); Rm, tv = np.array(tf["R"]), np.array(tf["t"]); LN = json.load(open(f"{G}/lines.json")); AX = np.array(LN["arm_axis"])
Z = np.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_v2.npz"); SEG = Z["shape_segment"]; RAD = Z["shape_radius"].astype(float)
P0 = Z["p0"][0].astype(float) @ Rm.T + tv; P1 = Z["p1"][0].astype(float) @ Rm.T + tv; COMS = Z["com"][0].astype(float) @ Rm.T + tv
LH = Rm @ Z["inv"][0][18].astype(float) @ Rm.T
HAND = [k for k in range(24) if SEG[k] == 18 and RAD[k] > 0]; FORE = [k for k in range(24) if SEG[k] in (15, 16) and RAD[k] > 0]; TORSO = [k for k in range(24) if SEG[k] in (0, 3, 8) and RAD[k] > 0]
MV = np.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_verts.npz"); V = MV["verts"][0].astype(float) @ Rm.T + tv; F = MV["faces"]
LAB = np.load(f"{OUT}/vertex_segment.npy"); TRI = V[F]
tl = np.array([np.bincount(LAB[f][LAB[f] >= 0], minlength=19).argmax() if (LAB[f] >= 0).any() else -1 for f in F])
HV = np.nonzero(LAB == 18)[0]; HTRI = np.nonzero(tl == 18)[0]; KDH = cKDTree(V[HV]); VT = {}
for fi in HTRI:
    for v in F[fi]:
        VT.setdefault(v, []).append(fi)
RI = RobotInertiaNP(rotor=True); RS = RI.sphere_radius + 0.02
A = dict(eta0=0.10, muref=2.0, p=0.5, emin=0.05, emax=0.20); M_ISO_HAND = 0.6
eta_A = lambda mu: np.clip(A["eta0"] * (np.asarray(mu) / A["muref"]) ** A["p"], A["emin"], A["emax"])
CONDS = ["off", "uni10", "unim", "iso", "massA"]; PATHS = ["front", "top"]; SIGMAS = (0.02, 0.04); NS = 1000


def closest_pts(P, T):
    """closest points on triangles T (n,m,3,3) to points P (n,3): returns (dist (n,), point (n,3))"""
    a, b, c = T[..., 0, :], T[..., 1, :], T[..., 2, :]; p = P[:, None, :]
    ab, ac, ap = b - a, c - a, p - a; d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1); bp = p - b; d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1); cp = p - c; d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
    va = d3 * d6 - d5 * d4; vb = d5 * d2 - d1 * d6; vc = d1 * d4 - d3 * d2; den = va + vb + vc; den = np.where(np.abs(den) < 1e-30, 1e-30, den)
    q = a + ab * (vb / den)[..., None] + ac * (vc / den)[..., None]; sd = lambda x, y: np.where(np.abs(x - y) < 1e-30, 1e-30, x - y)
    cands = [(d1 <= 0) & (d2 <= 0), (d3 >= 0) & (d4 <= d3), (vc <= 0) & (d1 >= 0) & (d3 <= 0), (d6 >= 0) & (d5 <= d6), (vb <= 0) & (d2 >= 0) & (d6 <= 0), (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)]
    vals = [a, b, a + ab * (d1 / sd(d1, d3))[..., None], c, a + ac * (d2 / sd(d2, d6))[..., None], b + (c - b) * ((d4 - d3) / np.where(np.abs((d4 - d3) + (d5 - d6)) < 1e-30, 1e-30, (d4 - d3) + (d5 - d6)))[..., None]]
    done = np.zeros(d1.shape, bool)
    for m, v in zip(cands, vals):
        mm = m & ~done; q = np.where(mm[..., None], v, q); done |= mm
    d = np.linalg.norm(q - p, axis=-1); j = d.argmin(1); return d[np.arange(len(P)), j], q[np.arange(len(P)), j]


def hand_mesh_signed(C, k=12, m=48):
    """signed distance (negative inside the body mesh) from points C to the right-hand mesh surface, and the closest points"""
    _, idx = KDH.query(C, k=k); cand = np.zeros((len(C), m), np.int64)
    for i, row in enumerate(idx):
        t = sorted({x for v in HV[row] for x in VT.get(v, [])})[:m]; cand[i, :len(t)] = t; cand[i, len(t):] = t[0]
    d, q = closest_pts(C, TRI[cand]); close = d < 0.12
    if close.any():
        ins = SM.winding(C[close], TRI) >= 0.5; dd = d[close]; dd[ins] = -dd[ins]; d[close] = dd
    return d, q


def shape_u(c):
    """nearest-definition u from point c to the right-hand shapes (COM fallback within 1 mm of the axis); returns u, shape index, axis point"""
    best = None
    for k in HAND:
        ab = P1[k] - P0[k]; t = np.clip(((c - P0[k]) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = P0[k] + t * ab; d = np.linalg.norm(c - q) - RAD[k]
        if best is None or d < best[0]:
            best = (d, k, q)
    _, k, q = best; tgt = COMS[18] if np.linalg.norm(q - c) < 1e-3 else q; u = (tgt - c) / np.linalg.norm(tgt - c); return u, k, q


def deltas(seed, n_steps, sigma):
    return np.random.default_rng(1000 + seed).normal(size=(n_steps, NS, 3)) * sigma      # same samples for the same seed number in every condition


def result(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read(); return json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))


def run(tag, path, cond, seed):
    r = result(tag); rows = list(csv.DictReader(open(r["steps_csv"]))); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows]); n = len(Q)
    B = np.load(f"{O}/{tag}_b1.npz")["rows"]; Dsh = B[:, 12:36]; SMx = B[:, 36:60].astype(int); MU = B[:, 60:84]; MH = B[:, 84:108]; MR = B[:, 108:132]
    eta0 = float(np.load(f"{O}/{tag}_b1.npz")["cost_par"][1])
    out = dict(tag=tag, path=path, cond=cond, seed=seed, steps=n)
    # shape-based
    for g, ks in (("hand", HAND), ("forearm", FORE), ("torso", TORSO)):
        sub = Dsh[:, ks]; i, jj = np.unravel_index(int(np.argmin(sub)), sub.shape); k = ks[jj]
        out[f"{g}_shape_gap"] = float(sub[i, jj]); out[f"{g}_shape_step"] = int(i)
        if g == "hand":
            s = int(SMx[i, k]); C = RI.compute(Q[i])[0]; u, _, _ = shape_u(C[s]); mu = float(MU[i, k]); mr = float(MR[i, k])
            mui = 1 / (1 / M_ISO_HAND + (1 / mr if np.isfinite(mr) and mr > 0 else 0))
            out.update(hand_shape_sphere=RI.sphere_link[s], hand_shape_u_vs_front_deg=float(np.degrees(np.arccos(np.clip(u @ (-AX), -1, 1)))), hand_m_h=float(MH[i, k]), hand_m_r=mr,
                       hand_mu=mu, hand_eta=float(eta_A(mui)) if cond == "iso" else float(eta_A(mu)) if cond == "massA" else (eta0 if cond in ("uni10", "unim") else None), hand_mu_iso=mui)
    # mesh-based + risk
    Cs = np.stack([RI.compute(q)[0] for q in Q]); Lrs = None
    gmin = np.full(n, np.inf); jmin = np.zeros(n, int); pmin = np.zeros((n, 3)); per = []
    for i in range(n):
        d, qq = hand_mesh_signed(Cs[i]); g = d - RS; j = int(np.argmin(g)); gmin[i] = g[j]; jmin[i] = j; pmin[i] = qq[j]; per.append((g, d, qq))
    i0 = int(np.argmin(gmin)); out.update(hand_mesh_gap=float(gmin[i0]), hand_mesh_step=i0, hand_mesh_sphere=RI.sphere_link[jmin[i0]], hand_mesh_steps_negative=int((gmin < 0).sum()))
    vel = np.gradient(Cs, 1 / 60.0, axis=0)
    vn = np.zeros(n); mu_t = np.zeros(n); risk = {s: np.zeros(n) for s in SIGMAS}; P_t = {s: np.zeros(n) for s in SIGMAS}
    for i in range(n):
        g, d, qq = per[i]; j = jmin[i]; c = Cs[i, j]; um = (qq[j] - c); um = um / max(np.linalg.norm(um), 1e-12) * (1 if d[j] >= 0 else -1)
        vn[i] = max(0.0, float(vel[i, j] @ um)); u, _, _ = shape_u(c); Lr = RI.compute(Q[i])[1][j]; mu_t[i] = 1 / (u @ LH @ u + u @ Lr @ u)
        near = np.nonzero(g < 0.2)[0]
        if len(near):
            nrm = Cs[i, near] - qq[near]; nrm = nrm / np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12) * np.where(d[near] >= 0, 1, -1)[:, None]
            for s in SIGMAS:
                dl = deltas(seed, n, s)[i]; gap = g[near][None, :] - dl @ nrm.T           # (NS, n_near)
                P_t[s][i] = float((gap.min(1) <= 0).mean())
        for s in SIGMAS:
            risk[s][i] = P_t[s][i] * vn[i] * np.sqrt(mu_t[i] / A["muref"])
    for s in SIGMAS:
        out[f"risk_max_s{int(s*100):02d}"] = float(risk[s].max()); out[f"risk_sum_s{int(s*100):02d}"] = float(risk[s].sum()); out[f"P_max_s{int(s*100):02d}"] = float(P_t[s].max())
    out["vn_at_mesh_min"] = float(vn[i0]); out["mu_at_mesh_min"] = float(mu_t[i0])
    # task
    arr = r.get("arrival_list", []); out["pass_time_s"] = float((arr[0]["step"] + 1) / 60.0) if arr else None
    ee = B[:, 8:11]; out["ee_path_m"] = float(np.linalg.norm(np.diff(ee, axis=0), axis=1).sum()); dq = np.linalg.norm(np.diff(B[:, 1:8], axis=0), axis=1)
    out.update(dq_mean=float(dq.mean()), dq_p95=float(np.percentile(dq, 95)), infeasible=int(r["infeasible_steps"]), fb_exec=int(B[:, 132].sum()) if B.shape[1] > 132 else None,
               loop_ms_mean=float(r["loop_ms_mean"]), solver_ms_mean=float(r["solver_ms_mean"]))
    ca = f"{O}/{tag}_costagg.npz"
    if os.path.exists(ca):
        z = np.load(ca); nn = z["n"].sum(0); ss = z["s"].sum(0); sg = z["shape_segment"]; tot = nn.sum()
        segn = {int(x): float(nn[sg == x].sum()) for x in np.unique(sg)}
        out.update(rollout_cost_n=float(tot), rollout_share_by_segment={str(k): (v / tot if tot else 0.0) for k, v in segn.items() if v > 0},
                   rollout_cost_sum_by_segment={str(int(x)): float(ss[sg == x].sum()) for x in np.unique(sg) if ss[sg == x].sum() > 0}, fb_rollout=float(z["fb"].sum()))
    np.savez(f"{O}/{tag}_b2a_series.npz", gmin=gmin, jmin=jmin, vn=vn, mu=mu_t, **{f"P_s{int(s*100):02d}": P_t[s] for s in SIGMAS}, **{f"risk_s{int(s*100):02d}": risk[s] for s in SIGMAS})
    return out


def check_linear(tag, seed, n_steps=8):
    """exact vs linearised P on the steps with the largest linearised P (sigma 0.02)"""
    r = result(tag); rows = list(csv.DictReader(open(r["steps_csv"]))); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows])
    z = np.load(f"{O}/{tag}_b2a_series.npz"); idx = np.argsort(-z["P_s02"])[:n_steps]; res = []
    for i in idx:
        C = RI.compute(Q[i])[0]; d, qq = hand_mesh_signed(C); g = d - RS; near = np.nonzero(g < 0.2)[0]; dl = deltas(seed, len(Q), 0.02)[i]
        hit = np.zeros(NS, bool)
        for j in near:   # exact: distance from c - delta to the mesh, all samples
            dd, _ = hand_mesh_signed(C[j][None] - dl); hit |= (dd - RS[j]) <= 0
        res.append(dict(step=int(i), P_linear=float(z["P_s02"][i]), P_exact=float(hit.mean())))
    return res


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "runs":
        allr = []
        for p in PATHS:
            for c in CONDS:
                for s in range(8):
                    t = f"b2a_{p}_{c}_s{s}"
                    if os.path.exists(f"{O}/{t}_b1.npz"):
                        allr.append(run(t, p, c, s)); print(t, round(allr[-1]["hand_shape_gap"] * 100, 2), round(allr[-1]["hand_mesh_gap"] * 100, 2), allr[-1]["pass_time_s"], flush=True)
        json.dump(allr, open(f"{O}/b2a_runs.json", "w"), indent=1)
    elif cmd == "check_linear":
        print(json.dumps(check_linear(sys.argv[2], int(sys.argv[3])), indent=1))
