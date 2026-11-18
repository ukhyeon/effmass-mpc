"""pr_live_attr.py <tag> <human npz> <frame> <mode global|part> - (part) which body part produced the human cost when the robot stopped
or did not reach its goal, in a replay of a live recording (deterministic path, frame rule clip(floor((i + 1) / 60 * 30), F - 1)).

Stop moments: steps where the TCP speed < 0.05 m/s over >= 30 steps (0.5 s) while the TCP is > 0.03 m from both goals ("stalls not at
a goal"), plus the steps of a leg that never reached its goal before the run ended. At each such step the executed state's cost is
recomputed offline with the run's own rule (kernel rule: per robot sphere max over shapes of c(d, eta), c = 0.5 eta - d (d < 0),
(eta - d)^2 / (2 eta) (0 <= d < eta); u = sphere -> nearest point on the shape; m_r from the robot inverse inertia of that sphere;
global: mu_ref 0.53; part: mu_iso of the shape's segment). The step's cost is split by the segment group of the shape that gives the
largest cost (argmax over spheres and shapes): hand, forearm, upper_arm, torso, head, other.
Writes <tag>_attr.json in goal1/part_regen.
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
import g1_meff_gpu as G

O = BA.O; OUT = os.path.expanduser("~/effmass_mpc/goal1/part_regen"); DB, K, EM = 0.065, 0.10, 0.25
GRP = {17: "hand", 18: "hand", 15: "forearm", 16: "forearm", 13: "upper_arm", 14: "upper_arm", 0: "torso", 3: "torso", 8: "torso", 12: "head"}
INV_ISO = 1.0 / G.iso_mass_19()
HB, HT = BA.RI.body_of["fr3v2_1_hand"]


def main(tag, human, frame, mode):
    log = open(f"{O}/{tag}.log", errors="replace").read()
    r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1])); rows = list(csv.DictReader(open(r["steps_csv"])))
    Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows]); n = len(Q)
    perr = np.array([float(x["pos_err_m"]) for x in rows]); sw = np.array([int(float(x.get("goal_switched", 0) or 0)) for x in rows])
    z = np.load(human); F = len(z["p0"]); R, t = L.load_frame(frame)
    P0 = z["p0"].reshape(F, -1, 3).astype(float) @ R.T + t; P1 = z["p1"].reshape(F, -1, 3).astype(float) @ R.T + t
    INV = np.einsum("ij,fsjk,lk->fsil", R, z["inv"].reshape(F, 19, 3, 3).astype(float), R)
    rad = z["shape_radius"].astype(float); seg = z["shape_segment"]; act = [k for k in range(len(rad)) if rad[k] > 0]
    fr = np.minimum(np.floor((np.arange(n) + 1) / 60.0 * 30 + 1e-6).astype(int), F - 1)
    TCP = np.stack([(BA.RI._fk(q)[HB + 1] @ HT) @ np.array([0, 0, 0.1034, 1.0]) for q in Q])[:, :3]
    v = np.r_[0, np.linalg.norm(np.diff(TCP, axis=0), axis=1) * 60]
    slow = (v < 0.05) & (perr > 0.03)
    stop = np.zeros(n, bool); i = 0
    while i < n:
        if slow[i]:
            j = i
            while j < n and slow[j]:
                j += 1
            if j - i >= 30:
                stop[i:j] = True
            i = j
        else:
            i += 1
    last_sw = np.nonzero(sw)[0]; unreached_from = int(last_sw[-1] + 1) if len(last_sw) else 0
    unreached = np.zeros(n, bool)
    if perr[-1] > 0.03:
        unreached[unreached_from:] = True
    sel = np.nonzero(stop | unreached)[0]
    acc = {g: 0.0 for g in ("hand", "forearm", "upper_arm", "torso", "head", "other")}; cnt = dict(acc); nz = 0
    for i in sel:
        C, Lr, _ = BA.RI.compute(Q[i]); f = fr[i]; best = (0.0, None)
        for k in act:
            a, b = P0[f][k], P1[f][k]; ab = b - a; tt = np.clip(((C - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = a + tt[:, None] * ab
            vv = q - C; nn = np.linalg.norm(vv, axis=1); d = nn - rad[k] - BA.RS
            m = d < EM
            if not m.any():
                continue
            u = vv[m] / np.maximum(nn[m], 1e-9)[:, None]
            ah = np.einsum("ni,ij,nj->n", u, INV[f][seg[k]], u); ar = np.einsum("ni,nij,nj->n", u, Lr[m], u); mu = 1 / (ah + ar)
            if mode == "part":
                eta = np.clip(DB + K * np.maximum(np.sqrt(mu * (INV_ISO[seg[k]] + ar)) - 1, 0), DB, EM)
            else:
                eta = np.clip(DB + K * np.maximum(np.sqrt(mu / 0.53) - 1, 0), DB, EM)
            dd = d[m]; c = np.where(dd < 0, 0.5 * eta - dd, np.where(dd < eta, (eta - dd) ** 2 * 0.5 / eta, 0.0))
            j = int(np.argmax(c))
            if c[j] > best[0]:
                best = (float(c[j]), int(seg[k]))
        if best[1] is not None:
            g = GRP.get(best[1], "other"); acc[g] += best[0]; cnt[g] += 1; nz += 1
    tot = sum(acc.values())
    res = dict(tag=tag, mode=mode, steps=n, stop_steps=int(stop.sum()), unreached_steps=int(unreached.sum()), steps_with_cost=nz,
               arrivals=int(r["arrivals"]), cost_share={g: (acc[g] / tot if tot else 0.0) for g in acc}, argmax_steps=cnt)
    json.dump(res, open(f"{OUT}/{tag}_attr.json", "w"), indent=1)
    print(json.dumps(res))


if __name__ == "__main__":
    main(*sys.argv[1:5])
