"""lv_corr_analyze.py <tag> <motion> <ver> <off|cmp> - live3m corrected paths: per run of one path version
(goal1/live/<tag>/corrected/cal/m<motion><ver>/), over the steps whose human frame lies in the held window and whose TCP is at least
0.10 m from both path ends:
  gap: min over every robot sphere (MPC radius + 0.02 m) and every active human shape of the surface distance; the closest pair
       (human segment, robot link), the time; gap to the target shapes only (hand shapes / elbow spheres)
  angle: between the unit vector from the nearest human surface point to the nearest robot sphere centre (the direction the robot
       lies in, = minus the robot -> human direction) and the target direction d of that frame (forearm axis / upper-arm axis)
  near_from_ref_m: distance of that human surface point from the target reference point of the frame (fingertip / elbow front point)
  per pass (between arrivals): min gap and angle;  first pass = start to the first arrival, its time (s)
  zone thickness along that direction from the nearest surface point (static field as lv_zone_figs.py, robot point mass 4.6 kg):
       uniform and per-part;  for runs with the human cost: eta of the closest pair at that step from the run's b1 record
  overlap steps (gap < 0)
off: runs corr_<tag>_m<q><ver>_{AB,BA}_off;  cmp: ..._{AB,BA}_{uni,part}_s{0,1,2}. Writes analysis_<mode>.json there.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lv_corr_lib as CL
import lv_cmp_analyze as CA
import lv_zone_figs as ZF
import g1_human_link as L

GG = CL.GG; O = CL.O


def one(tag, P, D, H, dh=None):
    """dh: optional translation of the human (m, base frame) - prediction for another path distance with the same robot record"""
    m = H.motion(P["q"] - 1); z = np.load(P["human"]); F = len(z["p0"]); R, t = L.load_frame(P["frame_file"]); t = t + (0 if dh is None else np.asarray(dh, float))
    P0 = z["p0"].reshape(F, -1, 3).astype(float) @ R.T + t; P1 = z["p1"].reshape(F, -1, 3).astype(float) @ R.T + t; rad, seg, act = H.rad, H.seg, H.act
    Q, arr, r = CL.load_run(tag); n = len(Q); fr = np.minimum(np.floor((np.arange(n) + 1) / 2 + 1e-6).astype(int), F - 1)
    sc = json.load(open(P[f"scene_{D}"])); a, b = np.array(sc["tcp_A"]), np.array(sc["tcp_B"]); e = (b - a) / np.linalg.norm(b - a); Ln = float(np.linalg.norm(b - a))
    C = [CL.spheres(q) for q in Q]; s = np.array([(CL.tcp_of(q) - a) @ e for q in Q]); cuts = [0] + arr + [n]; leg = np.zeros(n, int)
    for i, (x, y) in enumerate(zip(cuts[:-1], cuts[1:])):
        leg[x:y] = i
    sfwd = np.where(leg % 2 == 0, s, Ln - s)   # distance from the end the pass started at
    valid = np.nonzero((fr <= P["held_slice"][1]) & (s >= CL.END) & (s <= Ln - CL.END))[0]
    tk = sorted({c[2] for c in H.tpoints(m, P0[0], P1[0])}); g = np.full(n, np.inf); gt = np.full(n, np.inf); pair = {}
    for i in valid:
        f = fr[i]; nb = CL.nearest(C[i], P0[f], P1[f], rad, act); g[i] = nb[0]; pair[i] = nb
        if m["target"] == "fingertip":
            gt[i] = CL.nearest(C[i], P0[f], P1[f], rad, tk)[0]
        else:
            gt[i] = min(float((np.linalg.norm(C[i] - p, axis=1) - rr - CL.RS).min()) for p, rr, _ in H.tpoints(m, P0[f], P1[f]))

    def at(i):
        f = fr[i]; gap, j, k, v = pair[i]; d = H.tdir(m, P0[f], P1[f]); a_, b_ = P0[f][k], P1[f][k]; ab = b_ - a_
        sp = a_ + np.clip((C[i][j] - a_) @ ab / max(ab @ ab, 1e-12), 0, 1) * ab + rad[k] * v   # nearest human surface point
        Dz = dict(P0=P0[f], P1=P1[f], RAD=rad, SEG=seg, LAM={q_: R @ z["inv"][f][q_].astype(float) @ R.T for q_ in range(19)})
        arm = [k_ for k_ in act if seg[k_] in CL.ARM[m["side"]]]
        return dict(step=int(i), t_s=(i + 1) / 60.0, gap=float(gap), angle_deg=CL.ang(v, d), seg=int(seg[k]), shape=int(k), link=str(CL.LINK[j]).replace("fr3v2_1_", ""), sphere=int(j),
                    v=[float(x) for x in v], d=[float(x) for x in d], s_from_start=float(sfwd[i]), target_gap=float(gt[i]),
                    near_point=[float(x) for x in sp], near_from_ref_m=float(np.linalg.norm(sp - H.tref(m, P0[f], P1[f])[0])),
                    zone_uniform=ZF.ray(Dz, arm, "uniform", sp, v), zone_part=ZF.ray(Dz, arm, "part", sp, v), zone_part_along_d=ZF.ray(Dz, arm, "part", sp, d))

    out = dict(tag=tag, start=D[0], steps=n, arrivals=arr, first_pass_s=(arr[0] + 1) / 60.0 if arr else None, valid_steps=int(len(valid)), held_steps=int((fr <= P["held_slice"][1]).sum()),
               overlap_steps=int((g[valid] < 0).sum()) if len(valid) else 0, infeasible=int(r["infeasible_steps"]), passes=[])
    if not len(valid):
        return out
    out["min"] = at(int(valid[int(np.argmin(g[valid]))])); out["target_gap_min"] = float(gt[valid].min())
    for i in range(len(cuts) - 1):
        vi = valid[leg[valid] == i]
        if len(vi):
            p = at(int(vi[int(np.argmin(g[vi]))])); p.update(leg=i, dir=("A->B" if (i % 2 == 0) == (D == "AB") else "B->A"), complete=bool(i < len(arr))); out["passes"].append(p)
    if os.path.exists(f"{O}/{tag}_b1.npz"):
        b1 = np.load(f"{O}/{tag}_b1.npz"); rw = b1["rows"]; nq = rw.shape[1] - 1 - 3 - 1 - 120 - 1; o = 1 + nq + 3 + 1
        RR = dict(par=b1["cost_par"], mode=int(b1["cost_mode"]), MU=rw[:, o + 48:o + 72], MR=rw[:, o + 96:o + 120], step_b1=rw[:, 0].astype(int))
        out["min"]["eta_b1"], out["min"]["mu_b1"] = CA.eta_of(RR, out["min"]["step"], out["min"]["shape"], out["min"]["seg"])
    out["gap_ts"] = [None if not np.isfinite(x) else round(float(x), 4) for x in g]
    return out


def main(tag, q, ver, mode):
    T = f"{GG}/live/{tag}/corrected/cal/m{q}{ver}"; P = json.load(open(f"{T}/path.json")); H = CL.Human(tag); res = {}
    names = {"off": [("off", ["off"])], "cmp": [("uniform", [f"uni_s{s}" for s in (0, 1, 2)]), ("part", [f"part_s{s}" for s in (0, 1, 2)])]}[mode]
    for D in ("AB", "BA"):
        for md, sfx in names:
            for x in sfx:
                t = f"corr_{tag}_m{q}{ver}_{D}_{x}"
                if not os.path.exists(f"{O}/{t}.log"):
                    continue
                o = one(t, P, D, H); res.setdefault(D, {}).setdefault(md, []).append(o)
                if "min" not in o:
                    print(t, "no valid steps"); continue
                mn = o["min"]
                print(f"{t}: gap {100 * mn['gap']:.1f} cm at {mn['t_s']:.2f} s, angle {mn['angle_deg']:.0f} deg, seg {mn['seg']} / {mn['link']}, target gap {100 * mn['target_gap']:.1f}, "
                      f"zone uni {100 * mn['zone_uniform']:.1f} part {100 * mn['zone_part']:.1f}" + (f" eta_b1 {100 * mn['eta_b1']:.1f}" if "eta_b1" in mn else "")
                      + f", first pass {o['first_pass_s']} s, overlap {o['overlap_steps']}, passes " + " ".join(f"[{p['dir']} {100 * p['gap']:.1f}cm {p['angle_deg']:.0f}deg seg{p['seg']}{'' if p['complete'] else ' (unfinished)'}]" for p in o["passes"]), flush=True)
    json.dump(res, open(f"{T}/analysis_{mode}.json", "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4])
