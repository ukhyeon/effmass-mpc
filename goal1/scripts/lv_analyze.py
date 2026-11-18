"""lv_analyze.py - live prep: shape-based hand gap and robot speed near the human for one run, with the human frame per step given
either by the deterministic rule (precomputed runs) or by the wall-clock replay log (ROS-path runs).

  python3 lv_analyze.py <tag> <human npz> <frame file> [--ros]
    human npz : precomputed file (node world), shapes from its p0/p1/shape_radius (as r1_analyze.human)
    frame file: node world -> Isaac transform (temp_transform.json or live_frame.yml, g1_human_link.load_frame)
    default   : step i -> frame clip(floor((i + 1) / 60 * 30), F - 1)  (PrecomputedHumanLink rule, same as r1_analyze.py)
    --ros     : step i -> the replay frame at the wall time of step i, floor((t - t0) * 30) (TRUE human at that time, also in gaps).
                Wall time of steps: cumulative loop_ms anchored to the Isaac pump times of new messages (<tag>_human_inertia_rec.npz
                'upd', in step order of h_new == 1); replay log <tag>_replay.npz (frame, wall_ns).
Metrics as r1_analyze.py: min hand gap (shape, MPC sphere radius + 0.02), penetration steps, v_near_p99 (robot sphere speed when
within 0.4 m of any human shape), passes (goal switches), also the age of the human data at the MPC (ROS: wall - input stamp).
Writes <tag>_lv_analysis.json.
"""
import argparse
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

O = BA.O


def load_run(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read()
    r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))
    rows = list(csv.DictReader(open(r["steps_csv"])))
    return r, rows


def ros_frames(tag, rows, F):
    rec = np.load(f"{O}/{tag}_human_inertia_rec.npz"); upd = rec["upd"]          # (n, st, trecv, cnt, time_ns, ...)
    rp = np.load(f"{O}/{tag}_replay.npz"); pf, pw, ps = rp["frame"], rp["wall_ns"], rp["stamp_ns"]
    loop = np.array([float(x["loop_ms"]) for x in rows]) / 1000.0
    hnew = np.array([int(float(x["h_new"])) for x in rows])
    t_end = np.cumsum(loop)                                                       # step end, relative
    inew = np.nonzero(hnew)[0]
    m = min(len(inew), len(upd))
    off = np.median(upd[:m, 4] / 1e9 - t_end[inew[:m]])                          # anchor: pump time of the new message ~ step end
    wall0 = t_end + off
    # wall time per step: linear between the pump times of new messages (about every 2 steps); outside, cumulative loop_ms
    wall = np.interp(np.arange(len(rows)), inew[:m], upd[:m, 4] / 1e9)
    wall[:inew[0]] = wall0[:inew[0]] + (wall[inew[0]] - wall0[inew[0]]); wall[inew[m - 1] + 1:] = wall0[inew[m - 1] + 1:] + (wall[inew[m - 1]] - wall0[inew[m - 1]])
    # TRUE frame at the step's wall time from the replay clock (frame = floor((t - t0) * 30)), also inside deliberate message gaps
    fr = np.floor((wall - int(rp["t0_ns"]) / 1e9) * 30.0 + 1e-9).astype(int)
    # age of the delivered data: input stamp of the last pumped message vs step time
    st = np.full(len(rows), np.nan); k = -1
    for i in range(len(rows)):
        if hnew[i] and k + 1 < len(upd):
            k += 1
        if k >= 0:
            st[i] = upd[k, 1] / 1e9
    age = wall - st
    # delivered frame (what the MPC used) from the input stamp -> replay frame
    dfr = np.full(len(rows), -1)
    sidx = {int(s): int(f) for s, f in zip(ps, pf)}
    k = -1
    for i in range(len(rows)):
        if hnew[i] and k + 1 < len(upd):
            k += 1
        if k >= 0:
            dfr[i] = sidx.get(int(upd[k, 1]), -1)
    return np.clip(fr, 0, F - 1), age, dfr, float(np.median(np.abs(wall0 - wall)))     # loop_ms-only clock vs anchored clock


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag"); ap.add_argument("human"); ap.add_argument("frame")
    ap.add_argument("--ros", action="store_true")
    a = ap.parse_args()
    R, t = L.load_frame(a.frame)
    z = np.load(a.human); F = len(z["p0"])
    P0 = z["p0"].reshape(F, -1, 3).astype(float) @ R.T + t; P1 = z["p1"].reshape(F, -1, 3).astype(float) @ R.T + t
    rad = z["shape_radius"].astype(float); seg = z["shape_segment"]
    hand = [k for k in range(len(rad)) if seg[k] == 18 and rad[k] > 0]; allk = [k for k in range(len(rad)) if rad[k] > 0]
    r, rows = load_run(a.tag)
    Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows]); n = len(Q)
    sw = np.array([int(float(x["goal_switched"])) for x in rows])
    extra = {}
    if a.ros:
        fr, age, dfr, anchor_err = ros_frames(a.tag, rows, F)
        ok = np.isfinite(age)
        extra = dict(age_ms_median=float(np.median(age[ok]) * 1000), age_ms_p95=float(np.percentile(age[ok], 95) * 1000),
                     frame_lag_median=float(np.median((fr - dfr)[dfr >= 0])), anchor_err_ms=anchor_err * 1000)
    else:
        fr = np.minimum(np.floor((np.arange(n) + 1) / 60.0 * 30 + 1e-6).astype(int), F - 1)
    C = np.stack([BA.RI.compute(q)[0] for q in Q])

    def gaps(i, ks):
        f = fr[i]
        return np.stack([SM.segdist(C[i], P0[f][kk], P1[f][kk]) - rad[kk] for kk in ks], 1) - BA.RS[:, None]
    gh = np.array([gaps(i, hand).min() for i in range(n)]); ga_s = [gaps(i, allk).min(1) for i in range(n)]; ga = np.array([g.min() for g in ga_s])
    vsph = np.linalg.norm(np.diff(C, axis=0), axis=-1) * 60; near = np.stack(ga_s[1:]) <= 0.4; vnear = vsph[near]
    swi = list(np.nonzero(sw)[0]); starts = [0] + [x + 1 for x in swi[:-1]]
    pass_min = [float(gh[a_:b_ + 1].min()) for a_, b_ in zip(starts, swi)]
    res = dict(tag=a.tag, human=a.human, frame=a.frame, ros=a.ros, steps=n, realtime_factor=r["realtime_factor"], solver_ms_mean=r["solver_ms_mean"],
               infeasible=int(r["infeasible_steps"]), arrivals=int(r["arrivals"]), passes_completed=len(swi), min_gap_shape=float(gh.min()),
               min_gap_all=float(ga.min()), pen_shape_hand=int((gh < 0).sum()), pen_shape_all=int((ga < 0).sum()),
               v_near_p99=float(np.percentile(vnear, 99)) if len(vnear) else None, pass_min_gap=pass_min, **extra)
    json.dump(res, open(f"{O}/{a.tag}_lv_analysis.json", "w"), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != "pass_min_gap"}))


if __name__ == "__main__":
    main()
