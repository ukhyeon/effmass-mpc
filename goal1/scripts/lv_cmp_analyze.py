"""lv_cmp_analyze.py <tag> - live cmp: uniform vs per-part reference on one live recording replayed at real time.

Inputs: goal1/live/<tag>/compare/runs.json (written by lv_replay_compare.sh: human file, frame file, scene, run tags per mode/seed).
Motion segmentation (human only, 30 Hz frames): d_path(f) = min over hand (17, 18), forearm (15, 16) and elbow shapes of the
surface distance to the robot reference path (straight TCP segment tcp_A - tcp_B of the scene, Franka base frame). A motion =
d_path < SEG_D (default 0.25 m, env CMP_SEG_D) for at least SEG_T s (default 3, env CMP_SEG_T); gaps shorter than 0.5 s inside are
bridged. Numbered in time order; everything else is "arms down" (one row, all of it).
Per run and interval: min gap (robot sphere surface incl. the 0.02 m buffer to shape surface) to hand, forearm, elbow, upper arm;
the closest of these, the zone thickness eta of that mode at that moment (from the b1 log: mu and m_r of the robot sphere nearest
to that shape; uniform k 0 = eta0), steps with an arm gap < 0, arrivals, pass time (step gap to the previous arrival, median).
Elbow = sphere at the elbow end (p0) of the forearm capsule with the forearm radius (it lies inside the forearm capsule, so the
forearm gap <= elbow gap; "closest part" says elbow when the elbow gap is within 1 cm of the forearm gap).
Writes compare/analysis.json and compare/gaps_ts.npz (per run per step: arm gap, part index).
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

O = BA.O; GG = os.path.expanduser("~/effmass_mpc/goal1")
SEG_D = float(os.environ.get("CMP_SEG_D", "0.25")); SEG_T = float(os.environ.get("CMP_SEG_T", "3.0")); BRIDGE = 0.5
PARTS = ["hand", "forearm", "elbow", "upper_arm"]
ISO_NM = ["pelvis", "left_thigh", "right_thigh", "abdomen", "left_calf", "right_calf", "left_calf", "right_calf", "chest", "neck", "left_shoulder",
          "right_shoulder", "head", "left_upper_arm", "right_upper_arm", "left_lower_arm", "right_lower_arm", "left_hand", "right_hand"]
M_ISO = np.array([json.load(open(f"{GG}/data/iso_15066.json"))[n]["m_h_iso"] for n in ISO_NM], float)


def seg_seg(a, b, A, B, n=9):
    P = a[None] + np.linspace(0, 1, n)[:, None] * (b - a)[None]
    return float(SM.segdist(P, A, B).min())


def human(cfg):
    z = np.load(cfg["human"]); F = len(z["p0"]); R, t = L.load_frame(cfg["frame_file"])
    P0 = z["p0"].reshape(F, -1, 3).astype(float) @ R.T + t; P1 = z["p1"].reshape(F, -1, 3).astype(float) @ R.T + t
    return dict(F=F, P0=P0, P1=P1, rad=z["shape_radius"].astype(float), seg=z["shape_segment"].astype(int))


def segment(H, sc):
    A, B = np.array(sc["tcp_A"], float), np.array(sc["tcp_B"], float); rad, seg = H["rad"], H["seg"]
    ks = [k for k in range(len(rad)) if rad[k] > 0 and seg[k] in (15, 16, 17, 18)]; fk = [k for k in ks if seg[k] in (15, 16)]
    d = np.full(H["F"], np.inf)
    for f in range(H["F"]):
        v = [seg_seg(H["P0"][f][k], H["P1"][f][k], A, B) - rad[k] for k in ks]
        v += [float(SM.segdist(H["P0"][f][k][None], A, B)[0]) - rad[k] for k in fk]
        d[f] = min(v)
    m = d < SEG_D; iv = []; f = 0
    while f < len(m):
        if m[f]:
            g = f
            while g + 1 < len(m) and m[g + 1]:
                g += 1
            iv.append([f, g]); f = g + 1
        else:
            f += 1
    mg = []
    for a, b in iv:
        if mg and (a - mg[-1][1]) / 30.0 <= BRIDGE:
            mg[-1][1] = b
        else:
            mg.append([a, b])
    mot = [(a, b) for a, b in mg if (b - a + 1) / 30.0 >= SEG_T]
    return d, mot


def run_metrics(tag, H, n_h):
    log = open(f"{O}/{tag}.log", errors="replace").read()
    r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1])); rows = list(csv.DictReader(open(r["steps_csv"])))
    Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows]); n = len(Q)
    fr = np.minimum(np.floor((np.arange(n) + 1) / 60.0 * 30 + 1e-6).astype(int), H["F"] - 1)
    rad, seg = H["rad"], H["seg"]; act = [k for k in range(len(rad)) if rad[k] > 0]
    grp = {"hand": [k for k in act if seg[k] in (17, 18)], "forearm": [k for k in act if seg[k] in (15, 16)], "upper_arm": [k for k in act if seg[k] in (13, 14)]}
    G = np.full((n, 4), np.inf); K = np.zeros((n, 4), int)
    for i in range(n):
        C = BA.RI.compute(Q[i])[0]; f = fr[i]
        for j, p in enumerate(PARTS):
            src = grp["forearm"] if p == "elbow" else grp[p]
            for k in src:
                dd = (np.linalg.norm(C - H["P0"][f][k], axis=1) if p == "elbow" else SM.segdist(C, H["P0"][f][k], H["P1"][f][k])) - rad[k] - BA.RS
                v = float(dd.min())
                if v < G[i, j]:
                    G[i, j] = v; K[i, j] = k
    b1 = np.load(f"{O}/{tag}_b1.npz"); rw = b1["rows"]; par = b1["cost_par"]; mode = int(b1["cost_mode"])
    nq = rw.shape[1] - 1 - 3 - 1 - 24 * 5 - 1; o = 1 + nq + 3 + 1
    MU = rw[:, o + 48:o + 72]; MR = rw[:, o + 96:o + 120]; step_b1 = rw[:, 0].astype(int)
    arr = [int(a["step"]) for a in (r.get("arrival_list") or [])]
    return dict(tag=tag, n=n, G=G, K=K, MU=MU, MR=MR, step_b1=step_b1, par=par, mode=mode, arr=arr, rtf=r["realtime_factor"], Q=Q)


def eta_of(R, i, k, seg):
    e0, mref, pw, emin, emax = [float(x) for x in R["par"][1:6]]
    j = int(np.searchsorted(R["step_b1"], i)); j = min(j, len(R["step_b1"]) - 1)
    mu, mr = float(R["MU"][j, k]), float(R["MR"][j, k])
    if R["mode"] == 5:
        ratio = mu * (1.0 / M_ISO[seg] + (1.0 / mr if np.isfinite(mr) and mr > 0 else 0.0))
    else:
        ratio = mu / mref
    return float(np.clip(e0 + pw * max(np.sqrt(max(ratio, 0.0)) - 1.0, 0.0), emin, emax)), mu


def interval(R, H, steps):
    if not len(steps):
        return None
    g = R["G"][steps]; ga = g.min(1); i0 = int(steps[int(np.argmin(ga))]); j = int(np.argmin(R["G"][i0]))
    if PARTS[j] == "forearm" and R["G"][i0, 2] - R["G"][i0, 1] < 0.01:
        j = 2
    k = int(R["K"][i0, j]); eta, mu = eta_of(R, i0, k, int(H["seg"][k]))
    s0, s1 = int(steps[0]), int(steps[-1]); ar = [a for a in R["arr"] if s0 <= a <= s1]
    pt = [(a - max([b for b in R["arr"] if b < a], default=-1)) / 60.0 for a in ar if any(b < a for b in R["arr"])]
    return dict(min_gap={p: float(g[:, q].min()) for q, p in enumerate(PARTS)}, min_gap_arm=float(ga.min()), t_min_s=(i0 + 1) / 60.0,
                closest=PARTS[j], eta_at_min=eta, mu_at_min=mu, overlap_steps=int((ga < 0).sum()), arrivals=len(ar),
                pass_time_s=float(np.median(pt)) if pt else None)


def stat(vals):
    v = [x for x in vals if x is not None]
    return dict(median=float(np.median(v)), lo=float(min(v)), hi=float(max(v))) if v else None


def main(tag):
    C = f"{GG}/live/{tag}/compare"; cfg = json.load(open(f"{C}/runs.json")); sc = json.load(open(cfg["scene"]))
    H = human(cfg); d, mot = segment(H, sc)
    runs = {m: [run_metrics(t, H, H["F"]) for t in tags] for m, tags in cfg["runs"].items()}
    n = min(R["n"] for rs in runs.values() for R in rs)
    st_f = np.minimum(np.floor((np.arange(n) + 1) / 2 + 1e-6).astype(int), H["F"] - 1)
    ivs = [("motion %d" % (q + 1), np.nonzero((st_f >= a) & (st_f <= b))[0], a / 30.0, (b + 1) / 30.0) for q, (a, b) in enumerate(mot)]
    inm = np.zeros(n, bool)
    for _, s, _a, _b in ivs:
        inm[s] = True
    ivs.append(("arms down", np.nonzero(~inm)[0], None, None))
    out = dict(tag=tag, seg_d_m=SEG_D, seg_t_s=SEG_T, bridge_s=BRIDGE, steps=n, duration_s=n / 60.0, intervals=[], modes=list(runs),
               rtf={m: [R["rtf"] for R in rs] for m, rs in runs.items()})
    for name, s, a, b in ivs:
        e = dict(name=name, t0_s=a, t1_s=b, steps=int(len(s)), per_mode={})
        for m, rs in runs.items():
            per = [interval(R, H, s) for R in rs]; per = [p for p in per if p]
            if not per:
                continue
            e["per_mode"][m] = dict(seeds=per, min_gap={p: stat([x["min_gap"][p] for x in per]) for p in PARTS}, min_gap_arm=stat([x["min_gap_arm"] for x in per]),
                                    closest=[x["closest"] for x in per], eta_at_min=stat([x["eta_at_min"] for x in per]),
                                    overlap_steps=stat([x["overlap_steps"] for x in per]), arrivals=stat([x["arrivals"] for x in per]),
                                    pass_time_s=stat([x["pass_time_s"] for x in per]))
        out["intervals"].append(e)
    ms = list(runs)
    if len(ms) == 2:   # first step where the two modes' joints differ, per seed (same seed, same human: equal until a zone differs)
        out["first_divergence_s"] = []
        for A, B in zip(runs[ms[0]], runs[ms[1]]):
            m_ = min(len(A["Q"]), len(B["Q"])); dq = np.nonzero(np.any(A["Q"][:m_] != B["Q"][:m_], axis=1))[0]
            out["first_divergence_s"].append(float((dq[0] + 1) / 60.0) if len(dq) else None)
            if len(dq):   # which human shape was nearest to the robot at that step (any body part) and its gap
                i = int(dq[0]); f = min(int(np.floor((i + 1) / 2 + 1e-6)), H["F"] - 1); Cs = BA.RI.compute(A["Q"][i])[0]
                gk = [(float((SM.segdist(Cs, H["P0"][f][k], H["P1"][f][k]) - H["rad"][k] - BA.RS).min()), int(H["seg"][k])) for k in range(len(H["rad"])) if H["rad"][k] > 0]
                g_, sg_ = min(gk); out.setdefault("divergence_nearest", []).append(dict(gap_cm=round(100 * g_, 1), segment=sg_))
    json.dump(out, open(f"{C}/analysis.json", "w"), indent=1)
    np.savez(f"{C}/gaps_ts.npz", d_path=d, **{f"{m}_s{q}": R["G"][:n].min(1) for m, rs in runs.items() for q, R in enumerate(rs)},
             motions=np.array(mot, float).reshape(-1, 2) / 30.0)
    print(json.dumps(dict(motions=[(round(a / 30.0, 1), round((b + 1) / 30.0, 1)) for a, b in mot], steps=n,
                          first_divergence_s=out.get("first_divergence_s"))))


if __name__ == "__main__":
    main(sys.argv[1])
