"""lv_tip_analyze.py <tag> <runs_json> <out_json> - live cmp tip: per motion and run, inside the motion core (the 2 s pads left out):
  target gap: motions with target fingertip = the fingertip hand shape of compare_tip (target_shape); elbow = min of the sphere at the
     forearm capsule end p0 (forearm radius) and the sphere at the upper-arm capsule end p1 (upper-arm radius)
     gap = robot sphere surface incl. the 0.02 m buffer (as all live cmp tables) to that surface; also the hand-group and arm gaps
  side of the closest target approach: argmax of front (along the arm axis), up, down of the unit vector target -> nearest robot sphere
  zone thickness eta at that moment (b1 log of the run: mu, m_r of the robot sphere nearest to the target shape; uniform k 0 = eta0)
  only the steps whose human frame is a held frame of the motion (scenes.json hold_frames) when present;
  per pass (between arrivals) the min target gap over its held steps, and their median;
  overlap steps (arm gap < 0), arrivals, pass time (median step gap to the previous arrival for arrivals in the core, s)
runs_json: {"motion 1": {"off": [tag], "uniform": [tags], "part": [tags]}, ...}; scenes from compare_tip/scenes.json.
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
import lv_cmp_analyze as CA

O = BA.O; GG = os.path.expanduser("~/effmass_mpc/goal1")


def one(tag, m):
    z = np.load(m["human"]); F = len(z["p0"]); R, t = L.load_frame(m["frame_file"])
    P0 = z["p0"].reshape(F, -1, 3).astype(float) @ R.T + t; P1 = z["p1"].reshape(F, -1, 3).astype(float) @ R.T + t
    rad = z["shape_radius"].astype(float); seg = z["shape_segment"].astype(int)
    side = m["side"]; su, sf, sh = (13, 15, 17) if side == "L" else (14, 16, 18)
    ks = lambda s: [k for k in range(len(rad)) if seg[k] == s and rad[k] > 0]; fa, ua = ks(sf)[0], ks(su)[0]
    log = open(f"{O}/{tag}.log", errors="replace").read()
    r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1])); rows = list(csv.DictReader(open(r["steps_csv"])))
    Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows]); n = len(Q)
    fr = np.minimum(np.floor((np.arange(n) + 1) / 2 + 1e-6).astype(int), F - 1)
    c0, c1 = m["core_s"]; core = np.nonzero((fr >= int(round(c0 * 30))) & (fr <= int(round(c1 * 30))))[0]
    if m.get("hold_frames"):   # (hold) only steps whose human frame holds the representative posture
        core = np.nonzero(np.isin(fr, np.array(m["hold_frames"])))[0]
    ax = np.array(m["axis"]); arm = [k for k in range(len(rad)) if rad[k] > 0 and seg[k] in (13, 14, 15, 16, 17, 18)]
    gt = np.full(n, np.inf); gh = np.full(n, np.inf); ga = np.full(n, np.inf); near = [None] * n
    for i in core:
        C = BA.RI.compute(Q[i])[0]; f = fr[i]
        if m["target"] == "fingertip":
            k = m["target_shape"]; a, b = P0[f][k], P1[f][k]; d = SM.segdist(C, a, b) - rad[k] - BA.RS; j = int(np.argmin(d)); gt[i] = d[j]
            ab = b - a; tt = np.clip((C[j] - a) @ ab / max(ab @ ab, 1e-12), 0, 1); near[i] = (C[j] - (a + tt * ab), k, j)
        else:
            best = None
            for k, p in ((fa, P0[f][fa]), (ua, P1[f][ua])):
                d = np.linalg.norm(C - p, axis=1) - rad[k] - BA.RS; j = int(np.argmin(d))
                if best is None or d[j] < best[0]:
                    best = (d[j], k, j, p)
            gt[i] = best[0]; near[i] = (C[best[2]] - best[3], best[1], best[2])
        gh[i] = min(float((SM.segdist(C, P0[f][k], P1[f][k]) - rad[k] - BA.RS).min()) for k in ks(sh))
        ga[i] = min(float((SM.segdist(C, P0[f][k], P1[f][k]) - rad[k] - BA.RS).min()) for k in arm)
    i0 = int(core[int(np.argmin(gt[core]))]); v, kt, _ = near[i0]; v = v / max(np.linalg.norm(v), 1e-9)
    comp = {"front": float(v @ ax), "up": float(v[2]), "down": float(-v[2])}
    eta = mu = None
    if os.path.exists(f"{O}/{tag}_b1.npz"):
        b1 = np.load(f"{O}/{tag}_b1.npz"); rw = b1["rows"]; nq = rw.shape[1] - 1 - 3 - 1 - 120 - 1; o = 1 + nq + 3 + 1
        RR = dict(par=b1["cost_par"], mode=int(b1["cost_mode"]), MU=rw[:, o + 48:o + 72], MR=rw[:, o + 96:o + 120], step_b1=rw[:, 0].astype(int))
        eta, mu = CA.eta_of(RR, i0, kt, int(seg[kt]))
    arr = [int(a["step"]) for a in (r.get("arrival_list") or [])]; s0, s1 = int(core[0]), int(core[-1])
    pt = [(a - max(b for b in arr if b < a)) / 60.0 for a in arr if s0 <= a <= s1 and any(b < a for b in arr)]
    cuts = [0] + arr + [n]; pg = []   # per pass (between arrivals): min target gap over its hold steps
    for p_, q_ in zip(cuts[:-1], cuts[1:]):
        g = gt[p_:q_]
        if np.isfinite(g).any():
            pg.append(float(np.min(g[np.isfinite(g)])))
    return dict(tag=tag, steps=n, target_gap=float(gt[core].min()), pass_gaps=pg, pass_gap_median=float(np.median(pg)) if pg else None, n_passes=len(pg), hand_gap=float(gh[core].min()), arm_gap=float(ga[core].min()), t_min_s=(i0 + 1) / 60.0,
                side=max(comp, key=comp.get), side_comp=comp, eta_at_min=eta, mu_at_min=mu, overlap_steps=int((ga[core] < 0).sum()),
                arrivals_core=len([a for a in arr if s0 <= a <= s1]), arrivals=len(arr), pass_time_s=float(np.median(pt)) if pt else None,
                infeasible=int(r["infeasible_steps"]), rtf=r["realtime_factor"], target_gap_ts=[None if not np.isfinite(x) else float(x) for x in gt])


def main(tag, runs_json, out_json):
    tag, sub = (tag.split("@") + ["compare_tip"])[:2]; T = f"{GG}/live/{tag}/{sub}"; S = {m["name"]: m for m in json.load(open(f"{T}/scenes.json"))["motions"]}
    J = json.load(open(runs_json)); out = {}
    for mot, modes in J.items():
        out[mot] = {}
        for mode, tags in modes.items():
            per = [one(t, S[mot]) for t in tags if os.path.exists(f"{O}/{t}.log")]
            st = lambda key: CA.stat([p[key] for p in per])
            out[mot][mode] = dict(seeds=per, target_gap=st("target_gap"), pass_gap_median=st("pass_gap_median"), hand_gap=st("hand_gap"), arm_gap=st("arm_gap"), eta_at_min=st("eta_at_min"),
                                  overlap_steps=st("overlap_steps"), arrivals_core=st("arrivals_core"), pass_time_s=st("pass_time_s"), side=[p["side"] for p in per])
            print(mot, mode, "target gap", [round(100 * p["target_gap"], 1) for p in per], "pass gaps", [[round(100 * x, 1) for x in p["pass_gaps"]] for p in per], "side", [p["side"] for p in per], "arr core", [p["arrivals_core"] for p in per],
                  "eta", [None if p["eta_at_min"] is None else round(100 * p["eta_at_min"], 1) for p in per], "overlap", [p["overlap_steps"] for p in per], flush=True)
    json.dump(out, open(out_json, "w"), indent=1)


if __name__ == "__main__":
    main(*sys.argv[1:4])
