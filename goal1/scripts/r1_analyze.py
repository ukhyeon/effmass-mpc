"""r1_analyze.py - R1 measurements from saved runs only (system python via g1_run_human.sh py). Robot state at step i = executed joint q of the steps
csv; human at the same simulation time = frame clip(floor((i + 1) dt 30), F - 1) of the precomputed moving-hand file (PrecomputedHumanLink rule).
Hand gap: shape-based (v2 hand capsules, MPC sphere radius yml + 0.02, as the cost's d) and mesh-based (per-frame hand mesh of the node body model,
signed by the winding number of the arm mesh). Passes = legs between goal switches (steps csv goal_switched). Per pass at its minimum shape gap: time, hand
state, approach speed of the fingertip toward the closest robot sphere (> 0 approaching), u vs frontal (static arm axis), mu (hand inertia of that frame,
robot inverse inertia at q), eta (gain formula with the run's k), the shape with the largest cost at the executed state (not the rollout aggregate), pass
time, side. Per run: penetration steps, near-human sphere speed p99, realtime factor, solver mean / p99, captures, infeasible, completed passes, stalls
(TCP < 0.05 m/s for >= 0.5 s within 0.4 m of a human shape). Outputs r1_runs.json, r1_passes.json, r1_summary.json."""
import json, os, re, sys, csv
import numpy as np
from scipy.spatial import cKDTree
from scipy.stats import linregress
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import shape_metrics as SM
O = BA.O; H = os.path.expanduser("~"); DD = f"{H}/effmass_mpc/goal1/data"; Rm, tv = BA.Rm, BA.tv; AX = BA.AX / np.linalg.norm(BA.AX)
K = {"gk000": 0.0, "gain": 0.10}; SPEEDS = ("v025", "v050", "v100"); LABV = np.load(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy")
HB, HT = BA.RI.body_of["fr3v2_1_hand"]; W_COST = 1e5


def human(v):
    z = np.load(f"{DD}/human_r1_reach_{v}.npz"); a = np.load(f"{DD}/human_r1_reach_{v}_armverts.npz")
    D = dict(p0=z["p0"].astype(float) @ Rm.T + tv, p1=z["p1"].astype(float) @ Rm.T + tv, inv=np.einsum("ij,fsjk,lk->fsil", Rm, z["inv"].astype(float), Rm), tip=z["tip"].astype(float) @ Rm.T + tv,
             phase=z["phase"], rad=z["shape_radius"].astype(float), seg=z["shape_segment"], F=len(z["p0"]), av=a["verts"], vid=a["vid"], fl=a["faces_local"])
    D["hand"] = [k for k in range(24) if D["seg"][k] == 18 and D["rad"][k] > 0]; D["all"] = [k for k in range(24) if D["rad"][k] > 0]
    hv = np.nonzero(LABV[D["vid"]] == 18)[0]; D["hv"] = hv; hl = np.zeros(len(D["vid"]), bool); hl[hv] = True; D["hf"] = D["fl"][hl[D["fl"]].all(1)]
    tipv = np.gradient(D["tip"], 1 / 30.0, axis=0); D["tipv"] = tipv; return D


def mesh_gap(D, f, C, RS):
    """signed distance (negative inside) of points C to the hand mesh of frame f, minus RS"""
    V = D["av"][f].astype(float) @ Rm.T + tv; T = V[D["hf"]]; kd = cKDTree(V[D["hv"]]); _, idx = kd.query(C, k=10)
    vt = {}
    for fi, tri in enumerate(D["hf"]):
        for vv in tri:
            vt.setdefault(vv, []).append(fi)
    out = np.empty(len(C))
    for n, row in enumerate(idx):
        cand = sorted({x for vv in D["hv"][row] for x in vt.get(vv, [])})[:60]; d, _ = BA.closest_pts(C[n][None], T[cand][None]); out[n] = d[0]
    close = out < 0.05
    if close.any():
        ins = SM.winding(C[close], V[D["fl"]]) >= 0.5; o2 = out[close]; o2[ins] = -o2[ins]; out[close] = o2
    return out - RS


def one(job):
    v, c, s, suffix = job; tag = f"r1_{v}_{c}_s{s}{suffix}"; D = human(v); k = K[c]
    log = open(f"{O}/{tag}.log", errors="replace").read(); r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1])); rows = list(csv.DictReader(open(r["steps_csv"])))
    Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows]); n = len(Q); sw = np.array([int(float(x["goal_switched"])) for x in rows]); solv = np.array([float(x["solver_ms"]) for x in rows])
    fr = np.minimum(np.floor((np.arange(n) + 1) / 60.0 * 30 + 1e-6).astype(int), D["F"] - 1)
    RL = [BA.RI.compute(q) for q in Q]; C = np.stack([x[0] for x in RL]); TCP = np.stack([(BA.RI._fk(q)[HB + 1] @ HT) @ np.array([0, 0, 0.1034, 1.0]) for q in Q])[:, :3]
    def gaps(i, ks):
        f = fr[i]; return np.stack([SM.segdist(C[i], D["p0"][f][kk], D["p1"][f][kk]) - D["rad"][kk] for kk in ks], 1) - BA.RS[:, None]
    gh = np.zeros(n); ga = np.zeros(n); sph = np.zeros(n, int); shp = np.zeros(n, int)
    for i in range(n):
        G = gaps(i, D["hand"]); j, kk = np.unravel_index(np.argmin(G), G.shape); gh[i] = G[j, kk]; sph[i] = j; shp[i] = D["hand"][kk]; ga[i] = gaps(i, D["all"]).min()
    gm = np.full(n, np.nan)
    for i in np.nonzero(gh < 0.12)[0]:
        Gs = gaps(i, D["hand"]).min(1); cand = np.nonzero(Gs < 0.12)[0]; gm[i] = mesh_gap(D, fr[i], C[i][cand], BA.RS[cand]).min()
    vsph = np.linalg.norm(np.diff(C, axis=0), axis=-1) * 60; near = np.stack([gaps(i, D["all"]).min(1) for i in range(1, n)]) <= 0.4; vnear = vsph[near]
    vt = np.r_[0, np.linalg.norm(np.diff(TCP, axis=0), axis=1) * 60]; slow = (vt < 0.05) & (ga < 0.4); stalls = []; i = 0
    while i < n:
        if slow[i]:
            j = i
            while j < n and slow[j]:
                j += 1
            if j - i >= 30:
                stalls.append((i, j - i))
            i = j
        else:
            i += 1
    swi = list(np.nonzero(sw)[0]); starts = [0] + [x + 1 for x in swi[:-1]]; passes = []
    for pi, (a, b) in enumerate(zip(starts, swi)):
        seg_ = slice(a, b + 1); i0 = a + int(np.argmin(gh[seg_])); f = fr[i0]; j = sph[i0]; kk = shp[i0]; cpos = C[i0, j]
        dtip = cpos - D["tip"][f]; vapp = float(D["tipv"][f] @ (dtip / np.linalg.norm(dtip)))
        a_, b_ = D["p0"][f][kk], D["p1"][f][kk]; ab = b_ - a_; tt = np.clip(((cpos - a_) @ ab) / max(ab @ ab, 1e-12), 0, 1); qp = a_ + tt * ab; uu = (qp - cpos) / np.linalg.norm(qp - cpos)
        LH = D["inv"][f][18]; Lr = RL[i0][1][j]; mu = 1 / (uu @ LH @ uu + uu @ Lr @ uu); eta = float(np.clip(0.065 + k * max(np.sqrt(mu / 0.53) - 1, 0), 0.065, 0.25))
        best = (0, None)
        for js in range(len(C[i0])):
            for ks in D["all"]:
                a2, b2 = D["p0"][f][ks], D["p1"][f][ks]; ab2 = b2 - a2; t2 = np.clip(((C[i0, js] - a2) @ ab2) / max(ab2 @ ab2, 1e-12), 0, 1); q2 = a2 + t2 * ab2
                dd = np.linalg.norm(q2 - C[i0, js]) - D["rad"][ks] - BA.RS[js]
                if dd >= 0.25:
                    continue
                u2 = (q2 - C[i0, js]) / max(np.linalg.norm(q2 - C[i0, js]), 1e-9); m2 = 1 / (u2 @ D["inv"][f][D["seg"][ks]] @ u2 + u2 @ RL[i0][1][js] @ u2)
                e2 = float(np.clip(0.065 + k * max(np.sqrt(m2 / 0.53) - 1, 0), 0.065, 0.25)); cst = (0.5 * e2 - dd) if dd < 0 else ((e2 - dd) ** 2 * 0.5 / e2 if dd < e2 else 0.0)
                if cst * W_COST > best[0]:
                    best = (cst * W_COST, int(D["seg"][ks]))
        nrm = (cpos - qp) / np.linalg.norm(cpos - qp); comp = {"up": nrm[2], "down": -nrm[2], "front": float(nrm @ AX), "side": abs(nrm[1])}; side = max(comp, key=comp.get)
        passes.append(dict(tag=tag, speed=v, cond=c, seed=s, pass_idx=pi, start=a, end=b, t_min=float((i0 + 1) / 60), gap_shape=float(gh[i0]),
                           gap_mesh=float(np.nanmin(gm[seg_])) if np.isfinite(gm[seg_]).any() else None, state=str(D["phase"][f]), v_approach=vapp,
                           u_vs_front_deg=float(np.degrees(np.arccos(np.clip(uu @ (-AX), -1, 1)))), mu=float(mu), eta=eta, cost_shape_seg=best[1], cost_at_min=best[0],
                           pass_time_s=float((b - a + 1) / 60), side=side if side != "side" else "other", reached=bool(float(rows[b]["pos_err_m"]) < 0.01), direction="A->B" if pi % 2 == 0 else "B->A"))
    caps = r["graph_captures_by_stage"]; legs = np.diff(np.r_[-1, swi]) / 60.0
    run = dict(tag=tag, speed=v, cond=c, seed=s, steps=n, realtime_factor=r["realtime_factor"], solver_ms_mean=float(solv[10:].mean()), solver_ms_p99=float(np.percentile(solv[10:], 99)),
               captures=caps, recapture=int(sum(vv for kk_, vv in caps.items() if kk_ != "warmup")), infeasible=int(r["infeasible_steps"]), passes_completed=len(swi),
               round_trip_s=float(2 * legs.mean()) if len(legs) else None, pen_shape_hand=int((gh < 0).sum()), pen_shape_all=int((ga < 0).sum()), pen_mesh_hand=int(np.nansum(gm < 0)),
               v_near_p99=float(np.percentile(vnear, 99)) if len(vnear) else None, stalls=len(stalls), stall_time_s=float(sum(l for _, l in stalls) / 60),
               stall_list=[dict(start_s=a / 60, dur_s=l / 60, at_goal=bool(min(np.linalg.norm(TCP[a + l - 1] - np.array(g)[:3] + np.array([0, 0, 0.1034])) for g in json.load(open(f"{BA.G}/scene_b2a4_front_0.38h1.json"))["goals_wxyz"]) < 0.03)) for a, l in stalls])
    np.savez(f"{O}/{tag}_r1_series.npz", gap_shape=gh, gap_all=ga, gap_mesh=gm, frame=fr, tcp=TCP, switch=sw)
    return run, passes


if __name__ == "__main__":
    from multiprocessing import Pool
    jobs = [(v, c, s, "") for v in SPEEDS for c in K for s in (0, 1, 2) if os.path.exists(f"{O}/r1_{v}_{c}_s{s}.log")]
    with Pool(9) as p:
        res = p.map(one, jobs)
    runs = [x[0] for x in res]; passes = [pp for x in res for pp in x[1]]
    _cv = lambda o: o.item() if hasattr(o, "item") else str(o)
    json.dump(runs, open(f"{O}/r1_runs.json", "w"), indent=1, default=_cv); json.dump(passes, open(f"{O}/r1_passes.json", "w"), indent=1, default=_cv)
    for r_ in runs:
        print(r_["tag"], "rtf", r_["realtime_factor"], "inf", r_["infeasible"], "recap", r_["recapture"], "passes", r_["passes_completed"], "RT %.2f" % r_["round_trip_s"], "pen", r_["pen_shape_hand"], r_["pen_mesh_hand"],
              "v99 %.3f" % r_["v_near_p99"], "stalls", r_["stalls"], round(r_["stall_time_s"], 2))
