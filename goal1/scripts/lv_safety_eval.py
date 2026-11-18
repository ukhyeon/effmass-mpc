"""lv_safety_eval.py <out_json> <spec_json> - overnight2 bundle 4: safety layer on vs off.
spec: list of {"name", "tag", "human", "frame", "gap_sim": [t0, t1] or null}. Per run (all human shapes, every step):
  gap = min over robot spheres (MPC radius + 0.02 m) and human shapes of the surface distance (as all live tables), nearest sphere speed
  = |C(i+1) - C(i-1)| * 30 (m/s, measured joints), TCP speed likewise.
  near3: steps with gap < 0.03 m: count, nearest-sphere speed median / p95 / max, and its approach part (velocity component toward the
  nearest human point, negative = moving away, clipped at 0); overlap: steps with gap < 0 (count, speeds); gaps always against the
  real human frames (in a simulated loss the robot saw a frozen frame);
  arrivals; gap_sim window (if any): TCP speed median / max after the first 0.5 s of the window (loss hold expected).
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

O = BA.O; HB, HT = BA.RI.body_of["fr3v2_1_hand"]


def one(sp):
    log = open(f"{O}/{sp['tag']}.log", errors="replace").read()
    r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1])); rows = list(csv.DictReader(open(r["steps_csv"])))
    Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows]); n = len(Q)
    z = np.load(sp["human"]); F = len(z["p0"]); R, t = L.load_frame(sp["frame"]); rate = float(z["rate"]) if "rate" in z.files else 30.0
    P0 = z["p0"].reshape(F, -1, 3).astype(float) @ R.T + t; P1 = z["p1"].reshape(F, -1, 3).astype(float) @ R.T + t; rad = z["shape_radius"].astype(float)
    act = [k for k in range(len(rad)) if rad[k] > 0]
    fr = np.minimum(np.floor((np.arange(n) + 1) / 60.0 * rate + 1e-6).astype(int), F - 1)
    # gaps against the real human motion (also in a simulated tracking loss, where the robot saw a frozen frame)
    C = np.stack([BA.RI.compute(q)[0] for q in Q]); TCP = np.stack([(BA.RI._fk(q)[HB + 1] @ HT) @ np.array([0, 0, 0.1034, 1.0]) for q in Q])[:, :3]
    g = np.full(n, np.inf); js = np.zeros(n, int); un = np.zeros((n, 3))
    for i in range(n):
        f = fr[i]
        for k in act:
            d = SM.segdist(C[i], P0[f][k], P1[f][k]) - rad[k] - BA.RS; j = int(np.argmin(d))
            if d[j] < g[i]:
                a_, b_ = P0[f][k], P1[f][k]; ab = b_ - a_; tq = np.clip((C[i, j] - a_) @ ab / max(ab @ ab, 1e-12), 0, 1); w = a_ + tq * ab - C[i, j]
                g[i] = d[j]; js[i] = j; un[i] = w / max(np.linalg.norm(w), 1e-9)   # unit vector sphere -> nearest human point
    sv = np.zeros(n); tv = np.zeros(n); av = np.zeros(n)
    for i in range(1, n - 1):
        vv = (C[i + 1, js[i]] - C[i - 1, js[i]]) * 30; sv[i] = np.linalg.norm(vv); av[i] = float(vv @ un[i]); tv[i] = np.linalg.norm(TCP[i + 1] - TCP[i - 1]) * 30
    st = lambda x: dict(n=int(len(x)), median=float(np.median(x)) if len(x) else None, p95=float(np.percentile(x, 95)) if len(x) else None, max=float(np.max(x)) if len(x) else None)
    out = dict(name=sp["name"], tag=sp["tag"], steps=n, arrivals=int(r["arrivals"]), infeasible=int(r["infeasible_steps"]), min_gap_cm=float(g.min() * 100),
               near3=st(sv[g < 0.03]), near3_approach=st(np.maximum(av[g < 0.03], 0)), overlap=st(sv[g < 0]), overlap_approach=st(np.maximum(av[g < 0], 0)),
               near12_speed=st(sv[(g < 0.12) & (g >= 0.03)]))
    if sp.get("gap_sim"):
        a, b = sp["gap_sim"]; tt = (np.arange(n) + 1) / 60.0; w = (tt >= a + 0.5) & (tt < b); w0 = (tt >= a - 2) & (tt < a)
        out["loss_window_tcp_speed"] = st(tv[w]); out["before_window_tcp_speed"] = st(tv[w0])
    sf = f"{O}/{sp['tag']}_safety.npz"
    if os.path.exists(sf):
        S = np.load(sf)["rows"]; out["safety_scale_lt1_steps"] = int((S[:, 1] < 1).sum()); out["safety_hold_steps"] = int((S[:, 1] == 0).sum()); out["loss_hold_steps"] = int(S[:, 4].sum())
    return out


def main(out, spec):
    res = [one(s) for s in json.load(open(spec))]
    json.dump(res, open(out, "w"), indent=1)
    for x in res:
        print(json.dumps({k: (round(v, 3) if isinstance(v, float) else ({kk: (round(vv, 3) if isinstance(vv, float) else vv) for kk, vv in v.items()} if isinstance(v, dict) else v)) for k, v in x.items()}))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
