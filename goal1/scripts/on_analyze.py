"""on_analyze.py - overnight offline measurements from saved runs only (generalised from r1_analyze.py; same gap, mesh, mu, eta and cost-shape rules).
Robot state at step i = executed joint q of the steps csv; TRUE human at the same simulation time = frame clip(floor((i + 1) dt 30), F - 1) of the
precomputed file named in the job (perturbations such as lead, lag, jitter or dropout change only what the controller saw, never the measured human).
Job dict: tag, human (file base name under goal1/data, needs <base>.npz and <base>_armverts.npz), k (gain, None = off), scene (scene json),
target_segs (segments whose shapes / mesh define the measured gap, default [18] = right hand; WP3 uses [16, 18] = forearm and hand),
ref_label (human phase label whose onsets are the reference event, default "hold"), plus free labels (group, speed, cond, seed, variant).
Per pass (leg between goal switches; single-pass runs: one leg ending at arrival or run end): minimum hand shape gap and its time, mesh gap, hand state,
approach speed, u vs front, mu, eta, deficit = eta - gap at the minimum, cost shape at the executed state (not the rollout aggregate), side, pass time,
reached; crossing time t_cross = first step of the leg where the TCP projection on the A->B line passes the projection of the reference fingertip
(fingertip at the first reference-event frame), phase = t_cross - nearest reference-event onset (s, negative = robot crossed before the event).
Per run: penetration steps (shape hand, shape all, mesh hand), near-human sphere speed p99, realtime factor, solver mean / p99, recapture, infeasible,
completed passes, stalls (TCP < 0.05 m/s for >= 0.5 s within 0.4 m of a human shape; at_goal flag), command roughness (RMS of joint acceleration and jerk
from the executed joint commands, 60 Hz finite differences).
Usage: on_analyze.py <out_prefix> <jobs.json> [workers]  -> <out_prefix>_runs.json, <out_prefix>_passes.json"""
import json, os, re, sys, csv
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import shape_metrics as SM
import r1_analyze as R1A
O = BA.O; H = os.path.expanduser("~"); DD = f"{H}/effmass_mpc/goal1/data"; Rm, tv = BA.Rm, BA.tv; AX = BA.AX / np.linalg.norm(BA.AX)
HB, HT = BA.RI.body_of["fr3v2_1_hand"]; W_COST = 1e5; DT = 1 / 60.0
_HC = {}


def human(base, segs=(18,)):
    key = (base, tuple(segs))
    if key in _HC:
        return _HC[key]
    z = np.load(f"{DD}/{base}.npz"); a = np.load(f"{DD}/{base}_armverts.npz")
    D = dict(p0=z["p0"].astype(float) @ Rm.T + tv, p1=z["p1"].astype(float) @ Rm.T + tv, inv=np.einsum("ij,fsjk,lk->fsil", Rm, z["inv"].astype(float), Rm), tip=z["tip"].astype(float) @ Rm.T + tv,
             phase=z["phase"], rad=z["shape_radius"].astype(float), seg=z["shape_segment"], F=len(z["p0"]), av=a["verts"], vid=a["vid"], fl=a["faces_local"])
    D["hand"] = [k for k in range(24) if D["seg"][k] in segs and D["rad"][k] > 0]; D["all"] = [k for k in range(24) if D["rad"][k] > 0]
    hv = np.nonzero(np.isin(R1A.LABV[D["vid"]], segs))[0]; D["hv"] = hv; hl = np.zeros(len(D["vid"]), bool); hl[hv] = True; D["hf"] = D["fl"][hl[D["fl"]].all(1)]
    D["tipv"] = np.gradient(D["tip"], 1 / 30.0, axis=0) if D["F"] > 1 else np.zeros_like(D["tip"]); _HC[key] = D; return D


def eta_of(mu, k):
    return float(np.clip(0.065 + k * max(np.sqrt(mu / 0.53) - 1, 0), 0.065, 0.25)) if k is not None else None


def one(job):
    tag = job["tag"]; D = human(job["human"], tuple(job.get("target_segs", (18,)))); k = job.get("k"); kk_ = 0.0 if k is None else k; sc = json.load(open(job["scene"])); ref = job.get("ref_label", "hold")
    log = open(f"{O}/{tag}.log", errors="replace").read(); r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1])); rows = list(csv.DictReader(open(r["steps_csv"])))
    Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows]); n = len(Q); sw = np.array([int(float(x.get("goal_switched", 0) or 0)) for x in rows]); solv = np.array([float(x["solver_ms"]) for x in rows])
    perr = np.array([float(x["pos_err_m"]) for x in rows])
    fr = np.minimum(np.floor((np.arange(n) + 1) / 60.0 * 30 + 1e-6).astype(int), D["F"] - 1)
    RL = [BA.RI.compute(q) for q in Q]; C = np.stack([x[0] for x in RL]); TCP = np.stack([(BA.RI._fk(q)[HB + 1] @ HT) @ np.array([0, 0, 0.1034, 1.0]) for q in Q])[:, :3]
    def gaps(i, ks):
        f = fr[i]; return np.stack([SM.segdist(C[i], D["p0"][f][kk], D["p1"][f][kk]) - D["rad"][kk] for kk in ks], 1) - BA.RS[:, None]
    gh = np.zeros(n); ga = np.zeros(n); sph = np.zeros(n, int); shp = np.zeros(n, int)
    for i in range(n):
        Gm = gaps(i, D["hand"]); j, kq = np.unravel_index(np.argmin(Gm), Gm.shape); gh[i] = Gm[j, kq]; sph[i] = j; shp[i] = D["hand"][kq]; ga[i] = gaps(i, D["all"]).min()
    gm = np.full(n, np.nan)
    for i in np.nonzero(gh < 0.12)[0]:
        Gs = gaps(i, D["hand"]).min(1); cand = np.nonzero(Gs < 0.12)[0]; gm[i] = R1A.mesh_gap(D, fr[i], C[i][cand], BA.RS[cand]).min()
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
    goals = [np.array(g)[:3] - np.array([0, 0, 0.1034]) for g in sc["goals_wxyz"]]
    stall_list = [dict(start_s=a / 60, dur_s=l / 60, at_goal=bool(min(np.linalg.norm(TCP[a + l - 1] - g) for g in goals) < 0.03)) for a, l in stalls]
    # reference events (onsets of ref label) and the crossing line
    lab = np.asarray(D["phase"]).astype(str); on = np.nonzero((lab[1:] == ref) & (lab[:-1] != ref))[0] + 1
    if lab[0] == ref:
        on = np.r_[0, on]
    t_on = on / 30.0; A_ = np.array(sc["tcp_A"]); B_ = np.array(sc["tcp_B"]); dAB = (B_ - A_) / np.linalg.norm(B_ - A_)
    tipref = D["tip"][on[0]] if len(on) else D["tip"][0]; sref = float((tipref - A_) @ dAB); sT = (TCP - A_) @ dAB
    swi = list(np.nonzero(sw)[0])
    if job.get("single_pass") or not swi:
        arr = np.nonzero(perr < 0.01)[0]; end = int(arr[0]) if len(arr) else n - 1; legs = [(0, end)]
    else:
        starts = [0] + [x + 1 for x in swi[:-1]]; legs = list(zip(starts, swi))
    passes = []
    for pi, (a, b) in enumerate(legs):
        seg_ = slice(a, b + 1); i0 = a + int(np.argmin(gh[seg_])); f = fr[i0]; j = sph[i0]; kq = shp[i0]; cpos = C[i0, j]
        dtip = cpos - D["tip"][f]; vapp = float(D["tipv"][f] @ (dtip / np.linalg.norm(dtip)))
        a_, b_ = D["p0"][f][kq], D["p1"][f][kq]; ab = b_ - a_; tt = np.clip(((cpos - a_) @ ab) / max(ab @ ab, 1e-12), 0, 1); qp = a_ + tt * ab; uu = (qp - cpos) / np.linalg.norm(qp - cpos)
        LH = D["inv"][f][D["seg"][kq]]; Lr = RL[i0][1][j]; mu = 1 / (uu @ LH @ uu + uu @ Lr @ uu); eta = eta_of(mu, k)
        best = (0, None)
        for js in range(len(C[i0])):
            for ks in D["all"]:
                a2, b2 = D["p0"][f][ks], D["p1"][f][ks]; ab2 = b2 - a2; t2 = np.clip(((C[i0, js] - a2) @ ab2) / max(ab2 @ ab2, 1e-12), 0, 1); q2 = a2 + t2 * ab2
                dd = np.linalg.norm(q2 - C[i0, js]) - D["rad"][ks] - BA.RS[js]
                if dd >= 0.25:
                    continue
                u2 = (q2 - C[i0, js]) / max(np.linalg.norm(q2 - C[i0, js]), 1e-9); m2 = 1 / (u2 @ D["inv"][f][D["seg"][ks]] @ u2 + u2 @ RL[i0][1][js] @ u2)
                e2 = eta_of(m2, kk_); cst = (0.5 * e2 - dd) if dd < 0 else ((e2 - dd) ** 2 * 0.5 / e2 if dd < e2 else 0.0)
                if cst * W_COST > best[0]:
                    best = (cst * W_COST, int(D["seg"][ks]))
        nrm = (cpos - qp) / np.linalg.norm(cpos - qp); comp = {"up": nrm[2], "down": -nrm[2], "front": float(nrm @ AX), "side": abs(nrm[1])}; side = max(comp, key=comp.get)
        ss = sT[a:b + 1] - sref; cr = np.nonzero(np.sign(ss[1:]) != np.sign(ss[:-1]))[0]; t_cross = float((a + cr[0] + 1 + 1) / 60) if len(cr) else None
        if t_cross is not None and len(t_on):
            dphi = t_cross - t_on; ph = float(dphi[np.argmin(np.abs(dphi))])
        else:
            ph = None
        w0, w1 = (max(a, int(t_cross * 60) - 1 - 45), min(b, int(t_cross * 60) - 1 + 45)) if t_cross is not None else (a, b)
        gw = float(gh[w0:w1 + 1].min())
        passes.append(dict(job, pass_idx=pi, start=a, end=b, t_min=float((i0 + 1) / 60), gap_shape=float(gh[i0]), gap_mesh=float(np.nanmin(gm[seg_])) if np.isfinite(gm[seg_]).any() else None,
                           state=str(D["phase"][f]), v_approach=vapp, min_shape=int(kq), min_seg=int(D["seg"][kq]), u_vs_front_deg=float(np.degrees(np.arccos(np.clip(uu @ (-AX), -1, 1)))), mu=float(mu), eta=eta,
                           deficit=(eta - float(gh[i0])) if eta is not None else None, cost_shape_seg=best[1], cost_at_min=best[0], pass_time_s=float((b - a + 1) / 60),
                           side=side if side != "side" else "other", reached=bool(perr[b] < 0.01), t_cross=t_cross, phase=ph, gap_window=gw,
                           direction="A->B" if pi % 2 == 0 else "B->A"))
    caps = r["graph_captures_by_stage"]; legs_s = np.diff(np.r_[-1, swi]) / 60.0
    qd = np.diff(Q, axis=0) / DT; qdd = np.diff(qd, axis=0) / DT; qddd = np.diff(qdd, axis=0) / DT
    run = dict(job, steps=n, realtime_factor=r["realtime_factor"], solver_ms_mean=float(solv[10:].mean()), solver_ms_p99=float(np.percentile(solv[10:], 99)),
               captures=caps, recapture=int(sum(vv for kk2, vv in caps.items() if kk2 != "warmup")), infeasible=int(r["infeasible_steps"]), passes_completed=len(swi),
               round_trip_s=float(2 * legs_s.mean()) if len(swi) else None, pen_shape_hand=int((gh < 0).sum()), pen_shape_all=int((ga < 0).sum()), pen_mesh_hand=int(np.nansum(gm < 0)),
               min_gap_shape=float(gh.min()), min_gap_mesh=float(np.nanmin(gm)) if np.isfinite(gm).any() else None,
               v_near_p99=float(np.percentile(vnear, 99)) if len(vnear) else None, stalls=len(stalls), stall_time_s=float(sum(l for _, l in stalls) / 60),
               stalls_not_at_goal=sum(1 for s_ in stall_list if not s_["at_goal"]), stall_time_not_at_goal_s=float(sum(s_["dur_s"] for s_ in stall_list if not s_["at_goal"])),
               stall_list=stall_list, acc_rms=float(np.sqrt((qdd[10:] ** 2).mean())), jerk_rms=float(np.sqrt((qddd[10:] ** 2).mean())),
               arrived=bool((perr < 0.01).any()), arrival_s=float((np.nonzero(perr < 0.01)[0][0] + 1) / 60) if (perr < 0.01).any() else None)
    np.savez(f"{O}/{tag}_on_series.npz", gap_shape=gh, gap_all=ga, gap_mesh=gm, frame=fr, tcp=TCP, switch=sw)
    return run, passes


def cv(o):
    return o.item() if hasattr(o, "item") else str(o)


if __name__ == "__main__":
    from multiprocessing import Pool
    pref, jf = sys.argv[1], sys.argv[2]; nw = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    jobs = [j for j in json.load(open(jf)) if os.path.exists(f"{O}/{j['tag']}.log")]
    with Pool(min(nw, len(jobs))) as p:
        res = p.map(one, jobs)
    runs = [x[0] for x in res]; passes = [pp for x in res for pp in x[1]]
    json.dump(runs, open(f"{pref}_runs.json", "w"), indent=1, default=cv); json.dump(passes, open(f"{pref}_passes.json", "w"), indent=1, default=cv)
    for r_ in runs:
        print(r_["tag"], "rtf %.3f" % r_["realtime_factor"], "inf", r_["infeasible"], "recap", r_["recapture"], "passes", r_["passes_completed"], "min %.3f" % r_["min_gap_shape"],
              "pen", r_["pen_shape_hand"], r_["pen_mesh_hand"], "v99 %.3f" % (r_["v_near_p99"] or -1), "stalls", r_["stalls"], r_["stalls_not_at_goal"])
