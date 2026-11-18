"""pr_gaps.py - (part) per-run gaps by body group with the pass-section rule (steps with the TCP within 0.10 m of the scene's tcp_A /
tcp_B left out), for any precomputed run (static or moving): human frame at step i = clip(floor((i + 1) / 60 * 30), F - 1).

  gap(group) = min over pass-section steps of (segment distance robot sphere - shape) - shape radius - (MPC sphere radius + 0.02)
  groups: hand (18), forearm_hand (16, 18), torso (0, 3, 8), head (12), all
  single pass: side of the closest hand approach (up / down / front / side, as r1_analyze), pass time (first arrival).
Usage as a module: one(tag, human_npz, scene_json, frame_json) -> dict. CLI: pr_gaps.py <out.json> <jobs.json> (list of [tag, human, scene]).
"""
import csv
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import b2a_analyze as BA
import shape_metrics as SM
import g1_human_link as L

O = BA.O; GG = os.path.expanduser("~/effmass_mpc/goal1"); DD = f"{GG}/data"
GROUPS = {"hand": (18,), "forearm_hand": (16, 18), "torso": (0, 3, 8), "head": (12,)}
HB, HT = BA.RI.body_of["fr3v2_1_hand"]; AX = BA.AX / np.linalg.norm(BA.AX); EE = 0.10


def one(tag, human, scene, frame=f"{GG}/b2a/g2/temp_transform.json"):
    log = open(f"{O}/{tag}.log", errors="replace").read()
    r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1])); rows = list(csv.DictReader(open(r["steps_csv"])))
    Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows]); n = len(Q)
    z = np.load(human if human.endswith(".npz") else f"{DD}/{human}.npz"); F = len(z["p0"]); R, t = L.load_frame(frame)
    P0 = z["p0"].reshape(F, -1, 3).astype(float) @ R.T + t; P1 = z["p1"].reshape(F, -1, 3).astype(float) @ R.T + t
    rad = z["shape_radius"].astype(float); seg = z["shape_segment"]
    fr = np.minimum(np.floor((np.arange(n) + 1) / 60.0 * 30 + 1e-6).astype(int), F - 1)
    sc = json.load(open(scene)); A_, B_ = np.array(sc["tcp_A"]), np.array(sc["tcp_B"])
    C = np.stack([BA.RI.compute(q)[0] for q in Q]); TCP = np.stack([(BA.RI._fk(q)[HB + 1] @ HT) @ np.array([0, 0, 0.1034, 1.0]) for q in Q])[:, :3]
    sec = (np.linalg.norm(TCP - A_, axis=1) > EE) & (np.linalg.norm(TCP - B_, axis=1) > EE)
    act = [k for k in range(len(rad)) if rad[k] > 0]
    G = np.full((n, len(rad)), np.inf); J = np.zeros((n, len(rad)), int)
    for i in range(n):
        f = fr[i]
        for k in act:
            d = SM.segdist(C[i], P0[f][k], P1[f][k]) - rad[k] - BA.RS; J[i, k] = int(np.argmin(d)); G[i, k] = d[J[i, k]]
    out = dict(tag=tag, steps=n, section_steps=int(sec.sum()), realtime_factor=r["realtime_factor"], solver_ms_mean=r["solver_ms_mean"],
               infeasible=int(r["infeasible_steps"]), arrivals=int(r["arrivals"]))
    for name, gs in list(GROUPS.items()) + [("all", tuple(range(19)))]:
        ks = [k for k in act if seg[k] in gs]
        if not ks:
            continue
        g = G[:, ks].min(1)
        out[f"gap_{name}"] = float(g[sec].min()) if sec.any() else None
        out[f"gap_{name}_all_steps"] = float(g.min())
        out[f"pen_{name}"] = int((g < 0).sum())
    # side of the closest hand approach and pass time (single pass)
    hk = [k for k in act if seg[k] == 18]
    if hk and sec.any():
        g = G[:, hk]; ii = np.where(sec)[0]; i0 = ii[int(np.argmin(g[ii].min(1)))]; kk = hk[int(np.argmin(g[i0]))]; j = J[i0, kk]; f = fr[i0]
        a, b = P0[f][kk], P1[f][kk]; ab = b - a; tt = np.clip(((C[i0, j] - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); qp = a + tt * ab
        nrm = (C[i0, j] - qp) / max(np.linalg.norm(C[i0, j] - qp), 1e-9)
        comp = {"up": nrm[2], "down": -nrm[2], "front": float(nrm @ AX), "side": abs(nrm[1])}
        out.update(side=max(comp, key=comp.get), t_min_hand_s=float((i0 + 1) / 60))
    arr = r.get("arrival_list") or []
    out["pass_time_s"] = float((arr[0]["step"] + 1) / 60) if arr else None
    return out


if __name__ == "__main__":
    from multiprocessing import Pool
    jobs = json.load(open(sys.argv[2]))
    jobs = [j for j in jobs if os.path.exists(f"{O}/{j[0]}.log")]
    with Pool(min(10, len(jobs))) as p:
        res = p.starmap(one, [tuple(j) for j in jobs])
    json.dump(res, open(sys.argv[1], "w"), indent=1)
    for x in res:
        print(x["tag"], {k: (round(v * 100, 2) if isinstance(v, float) and k.startswith("gap") else v) for k, v in x.items() if k.startswith("gap_") and not k.endswith("all_steps")})
