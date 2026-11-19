"""rp1_analyze.py [A|B] - real_prep1: tables for the acceleration-limit comparison (A) and the low-speed profile (B).
A: settings acc 15 (current), 10, 8 rad/s^2.
  corrected-path comparison (live3m cal/m1b fingertip, cal/m3b elbow; uniform / effective-mass; seeds 0-2 x start A, B):
     acc 15 = rp1_a15_* (rerun with the command and horizon logs; the earlier runs corr_live3m_* are reported as "15 old"),
     acc 10 / 8 = rp1_a<acc>_*. Gap etc. from lv_corr_analyze.one (all shapes, held window,
     path ends left out).
  moving hand replay, fingertip 0.5 m/s (rp1_a<acc>_r1v050_*; 15 rerun with the command log): pr_gaps.one (pass-section rule).
  first front-pass path (compare_tip, motions 2 and 3, whole motion): acc 15 = tip_live3m_m<q>_*, acc 10 / 8 = rp1_a<acc>_tipm<q>_*;
     gap to the target on the held frames etc. from lv_tip_analyze.one. Added because the exported records exceed 10 rad/s^2 only here.
  per setting and scene:
     joint target acceleration = |second difference| x 3600 of the 60 Hz joint targets q_cmd, max per joint; velocity = |first
        difference| x 60, max per joint (q_cmd from <tag>_cmd.npz, else <tag>_b1.npz);
     closest gap median [min, max] over the runs, overlap steps, pass time (corrected: start to the first arrival; moving hand:
        median time between arrivals), runs that reached the goal, solver ms mean, realtime factor;
     robot sphere speed from the measured joints (central difference, max over spheres per step): max and p99 over steps.
B: runs rp1_<name>_m1b_AB_<uni|part>_s0 for the names given after B (e.g. slow_acc10), same metrics for each run.
Writes goal1/real_prep1/summary_A.json / summary_B.json and prints tables (markdown).
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lv_corr_lib as CL
import lv_corr_analyze as AN
import pr_gaps as PG
import lv_tip_analyze as TA

GG = CL.GG; O = CL.O; RP = f"{GG}/real_prep1"


def qcmd(tag):
    if os.path.exists(f"{O}/{tag}_cmd.npz"):
        z = np.load(f"{O}/{tag}_cmd.npz"); r = z["rows"]; return r[:, 1:8], r[:, 8:15], r
    r = np.load(f"{O}/{tag}_b1.npz")["rows"]; return r[:, 1:8], None, None


def cmd_metrics(tag):
    q, qd, rows = qcmd(tag); acc = np.abs(np.diff(q, 2, axis=0)).max(0) * 3600.0; vel = np.abs(np.diff(q, axis=0)).max(0) * 60.0
    if os.path.exists(f"{O}/{tag}_traj.npz"):   # acceleration of the exported target itself (the planned acceleration, before differencing)
        tz = np.load(f"{O}/{tag}_traj.npz"); tq = np.abs(tz["qdd"]).max(0)
    else:
        tq = None
    out = dict(acc_max=acc.tolist(), vel_max=vel.tolist(), over10_steps=int((np.abs(np.diff(q, 2, axis=0)).max(1) * 3600.0 > 10.0).sum()),
               over10_at=[int(i) + 1 for i in np.nonzero(np.abs(np.diff(q, 2, axis=0)).max(1) * 3600.0 > 10.0)[0]], target_qdd_max=None if tq is None else tq.tolist())
    if qd is not None:
        out["qd_mpc_max"] = np.abs(qd).max(0).tolist(); out["limiter_scaled_steps"] = int((rows[:, 30] < 1.0).sum())
    return out


def sphere_speed(Q, tag=None):
    """measured: central difference of the spheres along the measured joints (m/s, max over spheres per step);
    commanded (when the command log exists): |C(q_cmd_i) - C(q_meas_i)| / dt, the step the limiter bounds"""
    C = np.stack([CL.spheres(q) for q in Q]); v = np.linalg.norm(C[2:] - C[:-2], axis=2) * 30.0; m = v.max(1)
    out = dict(max=float(m.max()), p99=float(np.percentile(m, 99)), over025_steps=int((m > 0.25).sum()), steps=int(len(m)))
    if tag and os.path.exists(f"{O}/{tag}_cmd.npz"):
        r = np.load(f"{O}/{tag}_cmd.npz")["rows"]; cm = np.array([np.linalg.norm(CL.spheres(a) - CL.spheres(b), axis=1).max() * 60.0 for a, b in zip(r[:, 1:8], r[:, 15:22])])
        out.update(cmd_max=float(cm.max()), cmd_p99=float(np.percentile(cm, 99)), cmd_over025_steps=int((cm > 0.25 + 1e-6).sum()))
    return out


def st(v):
    v = [x for x in v if x is not None]
    return None if not v else dict(median=float(np.median(v)), lo=float(min(v)), hi=float(max(v)), n=len(v))


def f(v, s=100.0, d=1):
    return "-" if not v else f"{v['median'] * s:.{d}f} [{v['lo'] * s:.{d}f}, {v['hi'] * s:.{d}f}]"


def corr_run(tag, q, D, H, P):
    o = AN.one(tag, P, D, H); Q, arr, r = CL.load_run(tag)
    mn = o.get("min", {})   # no "min": the robot never got 0.10 m away from a path end inside the held window
    return dict(tag=tag, gap=mn.get("gap"), seg=mn.get("seg"), angle=mn.get("angle_deg"), overlap=o["overlap_steps"], first_pass_s=o["first_pass_s"], arrivals=len(arr), valid_steps=o["valid_steps"],
                solver_ms=r["solver_ms_mean"], solver_p95=r["solver_ms_p95"], rtf=r["realtime_factor"], infeasible=int(r["infeasible_steps"]), cmd=cmd_metrics(tag), speed=sphere_speed(Q, tag))


def agg(runs, key_gap="gap"):
    J = lambda k: np.max([r["cmd"][k] for r in runs], 0).tolist()
    return dict(n=len(runs), gap=st([r[key_gap] for r in runs]), overlap=int(sum(r["overlap"] for r in runs)), pass_s=st([r["pass_s"] for r in runs]), reached=int(sum(r["arrivals"] > 0 for r in runs)),
                solver_ms=float(np.mean([r["solver_ms"] for r in runs])), rtf=float(np.mean([r["rtf"] for r in runs])), acc_max=J("acc_max"), vel_max=J("vel_max"),
                over10_steps=int(sum(r["cmd"]["over10_steps"] for r in runs)), speed_max=float(max(r["speed"]["max"] for r in runs)), speed_p99=float(np.median([r["speed"]["p99"] for r in runs])))


def gap_at(tag, m, steps):
    """all-shape gap (as the corrected tables) at given steps of a first-path run (human = compare_tip slice of the motion)"""
    import g1_human_link as L
    z = np.load(m["human"]); F = len(z["p0"]); R, t = L.load_frame(m["frame_file"]); P0 = z["p0"].reshape(F, -1, 3).astype(float) @ R.T + t; P1 = z["p1"].reshape(F, -1, 3).astype(float) @ R.T + t
    rad = z["shape_radius"].astype(float); act = [k for k in range(len(rad)) if rad[k] > 0]; Q, arr, r = CL.load_run(tag)
    return [float(CL.nearest(CL.spheres(Q[i]), P0[min((i + 1) // 2, F - 1)], P1[min((i + 1) // 2, F - 1)], rad, act)[0]) for i in steps if i < len(Q)]


def tip_detail(S):
    """first path: per run (motion, method, seed) the steps whose differenced target acceleration exceeds 10 rad/s^2 under each limit,
    the closest-approach gap per limit, and under 15 the all-shape gap at the exceeding steps"""
    TS = json.load(open(f"{GG}/live/live3m/compare_tip/scenes.json"))["motions"]; rows = []
    for q in (2, 3):
        for mode in ("uni", "part"):
            for s_ in (0, 1, 2):
                row = dict(motion=q, mode=mode, seed=s_)
                for acc in (15, 10, 8):
                    a = S.get(acc, {}).get(f"tip{q}_{mode}")
                    rr = [x for x in (a["runs"] if a else []) if x["tag"].endswith(f"_s{s_}")]
                    if rr:
                        x = rr[0]; row[acc] = dict(over10=x["cmd"]["over10_steps"], gap=x["gap"], acc_max=max(x["cmd"]["acc_max"]), target_qdd_max=None if x["cmd"]["target_qdd_max"] is None else max(x["cmd"]["target_qdd_max"]))
                        if acc == 15 and x["cmd"]["over10_at"]:
                            row[acc]["gap_at_over10"] = gap_at(x["tag"], TS[q - 1], x["cmd"]["over10_at"]); row[acc]["over10_at"] = x["cmd"]["over10_at"]
                rows.append(row)
    return rows


def runs_of(S, acc):
    return [x for k, v in S.get(acc, {}).items() if isinstance(v, dict) and "runs" in v for x in v["runs"]]


def excess_tables(S):
    """differenced target acceleration above the limit of the setting (10 / 8): per joint count of steps, max and median excess"""
    L = []
    for acc in (10, 8):
        for grp, pre in (("corrected path", ("m1_", "m3_")), ("first path", ("tip2_", "tip3_"))):
            E = [[] for _ in range(7)]; n = 0
            for k, v in S.get(acc, {}).items():
                if not (isinstance(v, dict) and "runs" in v and k.startswith(pre)):
                    continue
                for x in v["runs"]:
                    q = qcmd(x["tag"])[0]; a = np.abs(np.diff(q, 2, axis=0)) * 3600.0; n += 1
                    for j in range(7):
                        E[j] += (a[:, j][a[:, j] > acc] - acc).tolist()
            if not n:
                continue
            L += ["", f"Limit {acc} rad/s^2, {grp} ({n} runs): steps whose differenced target acceleration exceeds {acc}, per joint" + (" (appendix)" if acc == 8 else ""), "",
                  "| joint | steps over the limit | excess max (rad/s^2) | excess median (rad/s^2) |", "|---|---|---|---|"]
            for j in range(7):
                L.append(f"| {j + 1} | {len(E[j])} | " + (f"{max(E[j]):.2f} | {np.median(E[j]):.2f} |" if E[j] else "- | - |"))
    return L


def filter_check(S):
    """decomposition of the differenced target acceleration with the state filter of cuRobo (filtered = c * measured + (1 - c) * previous command,
    c = 0.1 for position and velocity): d2q/dt^2 = a_i + c (v_meas_i - v_cmd_{i-1}) / dt + c (e_i - e_{i-1}) / dt^2, e_i = q_meas_i - q_cmd_{i-1};
    checked on the runs of setting 10 with a command log; for the steps above 10: share of the two feedback terms and the largest c that
    would have kept the step at 10 if the rest stayed the same (first-order estimate, no new runs)"""
    c, dt = 0.1, 1.0 / 60.0; res = []; cmax = []; fb = []; n = 0
    for x in runs_of(S, 10):
        tag = x["tag"]
        if not os.path.exists(f"{O}/{tag}_cmd.npz"):
            continue
        r = np.load(f"{O}/{tag}_cmd.npz")["rows"]; q, a, qm, vm, vc = r[:, 1:8], r[:, 8:15] * 0, r[:, 15:22], r[:, 22:29], r[:, 8:15]
        tz = np.load(f"{O}/{tag}_traj.npz") if os.path.exists(f"{O}/{tag}_traj.npz") else None
        if tz is None:
            continue
        a = tz["qdd"].astype(float); n += 1
        for i in range(2, len(q)):
            if r[i, 30] < 1 or r[i - 1, 30] < 1:
                continue
            d2 = (q[i] - 2 * q[i - 1] + q[i - 2]) / dt ** 2
            t1 = c * (vm[i] - vc[i - 1]) / dt; e_i = qm[i] - q[i - 1]; e_p = qm[i - 1] - q[i - 2]; t2 = c * (e_i - e_p) / dt ** 2
            res.append(float(np.abs(d2 - (a[i] + t1 + t2)).max()))
            for j in range(7):
                if abs(d2[j]) > 10.0:
                    f_ = (t1[j] + t2[j]) / c; fb.append(abs(c * f_) / abs(d2[j]))
                    room = 10.0 * np.sign(d2[j]) - a[i][j]; cmax.append(float(np.clip(room / f_, 0, 1)) if abs(f_) > 1e-9 else 1.0)
    L = ["", f"State-filter check ({n} runs of setting 10 with the horizon log): max |differenced acc - (planned acc + filter terms)| = {max(res):.2e} rad/s^2 over {len(res)} steps"]
    if cmax:
        L.append(f"Steps above 10 (joint-steps {len(cmax)}): share of the filter terms in the differenced acceleration median {100 * np.median(fb):.0f} %, max {100 * max(fb):.0f} %; "
                 f"largest filter coefficient that would have kept each step at 10 (first-order): median {np.median(cmax):.3f}, min {min(cmax):.3f}")
    S["filter_check"] = dict(resid_max=max(res) if res else None, joint_steps_over10=len(cmax), share_median=float(np.median(fb)) if fb else None, c_ok_median=float(np.median(cmax)) if cmax else None, c_ok_min=float(min(cmax)) if cmax else None)
    return L


def appendix(S, NM):
    L = ["", "Appendix (not used for the decision): setting 8 (runs finished before it was dropped) and the earlier runs with 15 (corr_live3m_*)", "",
         "| scene, method | acc limit | gap cm, median [min, max] | overlap steps | pass time s | reached | solver ms | realtime x |", "|---|---|---|---|---|---|---|---|"]
    for k in NM:
        for acc in (8, "15old"):
            a = S.get(acc, {}).get(k)
            if a:
                L.append(f"| {NM[k]} | {acc} | {f(a['gap'])} | {a['overlap']} | {f(a['pass_s'], 1, 2)} | {a['reached']} of {a['n']} | {a['solver_ms']:.1f} | {a['rtf']:.2f} |")
    return L


def main_A():
    H = CL.Human("live3m"); S = {}
    for acc in (15, 10, 8, "15old"):
        S[acc] = {}
        for q in (1, 3):
            P = json.load(open(f"{GG}/live/live3m/corrected/cal/m{q}b/path.json"))
            for mode in ("uni", "part"):
                runs = []
                for D in ("AB", "BA"):
                    for s in (0, 1, 2):
                        tag = f"corr_live3m_m{q}b_{D}_{mode}_s{s}" if acc == "15old" else f"rp1_a{acc}_m{q}b_{D}_{mode}_s{s}"
                        if os.path.exists(f"{O}/{tag}.log") and "saved " in open(f"{O}/{tag}.log", errors="replace").read():
                            r = corr_run(tag, q, D, H, P); r["pass_s"] = r["first_pass_s"]; runs.append(r)
                if runs:
                    S[acc][f"m{q}_{mode}"] = dict(agg(runs), runs=runs)
        if acc == "15old":
            continue
        TS = json.load(open(f"{GG}/live/live3m/compare_tip/scenes.json"))["motions"]
        for q in (2, 3):
            for mode in ("uni", "part"):
                runs = []
                for s in (0, 1, 2):
                    tag = f"tip_live3m_m{q}_{mode}_s{s}" if acc == 15 else f"rp1_a{acc}_tipm{q}_{mode}_s{s}"
                    if os.path.exists(f"{O}/{tag}.log") and "saved " in open(f"{O}/{tag}.log", errors="replace").read():
                        t = TA.one(tag, TS[q - 1]); Q, arr, r = CL.load_run(tag)
                        runs.append(dict(tag=tag, gap=t["target_gap"], overlap=t["overlap_steps"], pass_s=t["pass_time_s"], arrivals=len(arr), solver_ms=r["solver_ms_mean"], solver_p95=r["solver_ms_p95"],
                                         rtf=r["realtime_factor"], infeasible=int(r["infeasible_steps"]), cmd=cmd_metrics(tag), speed=sphere_speed(Q)))
                if runs:
                    S[acc][f"tip{q}_{mode}"] = dict(agg(runs), runs=runs)
        for mode in ("uni", "part"):
            runs = []
            for s in (0, 1, 2):
                tag = f"rp1_a{acc}_r1v050_{mode}_s{s}"
                if os.path.exists(f"{O}/{tag}.log") and "saved " in open(f"{O}/{tag}.log", errors="replace").read():
                    g = PG.one(tag, "human_r1_reach_v050", f"{GG}/b2a/g2/scene_b2a4_front_0.38h1.json"); Q, arr, r = CL.load_run(tag)
                    runs.append(dict(tag=tag, gap=g["gap_hand"], gap_all=g["gap_all"], overlap=g["pen_all"], pass_s=float(np.median(np.diff(arr)) / 60.0) if len(arr) > 1 else None, arrivals=len(arr),
                                     solver_ms=r["solver_ms_mean"], solver_p95=r["solver_ms_p95"], rtf=r["realtime_factor"], infeasible=int(r["infeasible_steps"]), cmd=cmd_metrics(tag), speed=sphere_speed(Q)))
            if runs:
                S[acc][f"r1_{mode}"] = dict(agg(runs), runs=runs)
        print("acc", acc, "done", {k: v["n"] for k, v in S[acc].items()}, flush=True)
    S["tip_detail"] = tip_detail(S)
    json.dump(S, open(f"{RP}/summary_A.json", "w"), indent=1)
    NM = dict(m1_uni="fingertip, uniform", m1_part="fingertip, effective-mass", m3_uni="elbow, uniform", m3_part="elbow, effective-mass", r1_uni="moving hand 0.5 m/s, uniform", r1_part="moving hand 0.5 m/s, effective-mass",
              tip2_uni="first path, bent hand, uniform", tip2_part="first path, bent hand, effective-mass", tip3_uni="first path, elbow, uniform", tip3_part="first path, elbow, effective-mass")
    ACCS = (15, 10)   # main comparison; 8 and the earlier 15 runs go to the appendix tables
    L = ["| scene, method | acc limit | gap cm, median [min, max] | overlap steps | pass time s | reached | solver ms | realtime x |", "|---|---|---|---|---|---|---|---|"]
    for k in NM:
        for acc in ACCS:
            a = S[acc].get(k)
            if a:
                L.append(f"| {NM[k]} | {acc} | {f(a['gap'])} | {a['overlap']} | {f(a['pass_s'], 1, 2)} | {a['reached']} of {a['n']} | {a['solver_ms']:.1f} | {a['rtf']:.2f} |")
    L += ["", "| scenes | acc limit | joint target acceleration max, joints 1-7 (rad/s^2) | steps over 10 | joint target velocity max, joints 1-7 (rad/s) | sphere speed max / p99 (m/s) |", "|---|---|---|---|---|---|"]
    for grp, keys in (("corrected path (48 runs)", ("m1_uni", "m1_part", "m3_uni", "m3_part")), ("moving hand (6 runs)", ("r1_uni", "r1_part")), ("first path, motions 2 and 3 (12 runs)", ("tip2_uni", "tip2_part", "tip3_uni", "tip3_part"))):
        for acc in ACCS:
            a = [S[acc][k] for k in keys if k in S[acc]]
            if a:
                am = np.max([x["acc_max"] for x in a], 0); vm = np.max([x["vel_max"] for x in a], 0)
                L.append(f"| {grp.split(' (')[0]} ({sum(x['n'] for x in a)} runs) | {acc} | " + " / ".join(f"{x:.1f}" for x in am) + f" | {sum(x['over10_steps'] for x in a)} | " + " / ".join(f"{x:.2f}" for x in vm)
                         + f" | {max(x['speed_max'] for x in a):.2f} / {np.median([x['speed_p99'] for x in a]):.2f} |")
    L += ["", "First path (motions 2 and 3): steps whose target acceleration (second difference of the 60 Hz joint targets) exceeds 10 rad/s^2, and the closest gap (cm), per run",
          "", "| motion, method, seed | steps over 10: acc 15 / 10 / 8 | max differenced acc (rad/s^2): 15 / 10 / 8 | closest gap cm: 15 / 10 / 8 | acc 15: gap at the exceeding steps (cm) |", "|---|---|---|---|---|"]
    for r in S["tip_detail"]:
        g = lambda acc, k, sc=1.0, d=1: "-" if acc not in r else (f"{r[acc][k] * sc:.{d}f}" if isinstance(r[acc][k], float) else str(r[acc][k]))
        L.append(f"| {r['motion']} {'uniform' if r['mode'] == 'uni' else 'effective-mass'} s{r['seed']} | {g(15, 'over10')} / {g(10, 'over10')} / {g(8, 'over10')} | {g(15, 'acc_max')} / {g(10, 'acc_max')} / {g(8, 'acc_max')} | "
                 f"{g(15, 'gap', 100)} / {g(10, 'gap', 100)} / {g(8, 'gap', 100)} | " + (", ".join(f"{100 * v:.1f}" for v in r[15].get("gap_at_over10", [])) if 15 in r and r[15].get("gap_at_over10") else "-") + " |")
    L += excess_tables(S) + filter_check(S) + appendix(S, NM)
    open(f"{RP}/tables_A.md", "w").write("\n".join(L) + "\n"); print("\n".join(L))


def gs(r):
    return "- (robot did not reach the path interior)" if r["gap"] is None else f"{100 * r['gap']:.1f}"


def main_B(names):
    H = CL.Human("live3m"); P = json.load(open(f"{GG}/live/live3m/corrected/cal/m1b/path.json")); S = {}
    L = ["| profile, method | gap cm | sphere speed measured max / p99 (m/s) | measured steps over 0.25 | commanded step speed max / p99 | joint velocity max, target / measured (rad/s) | joint target acceleration max | first pass s | limiter steps | solver ms / realtime x |", "|---|---|---|---|---|---|---|---|---|---|"]
    for nm in names:
        for mode in ("uni", "part"):
            tag = f"rp1_{nm}_m1b_AB_{mode}_s0" if not nm.startswith("=") else nm[1:].format(mode=mode)
            if not os.path.exists(f"{O}/{tag}.log"):
                continue
            r = corr_run(tag, 1, "AB", H, P); Q, arr, res = CL.load_run(tag); r["meas_vel_max"] = float(np.abs(np.diff(Q, axis=0)).max() * 60.0); S[f"{nm}_{mode}"] = r; c = r["cmd"]
            L.append(f"| {nm}, {'uniform' if mode == 'uni' else 'effective-mass'} | {gs(r)} | {r['speed']['max']:.3f} / {r['speed']['p99']:.3f} | {r['speed']['over025_steps']} of {r['speed']['steps']} | {r['speed'].get('cmd_max', float('nan')):.3f} / {r['speed'].get('cmd_p99', float('nan')):.3f} | {max(c['vel_max']):.2f} / {r['meas_vel_max']:.2f} | "
                     f"{max(c['acc_max']):.1f} | {r['first_pass_s'] if r['first_pass_s'] is None else round(r['first_pass_s'], 2)} | {c.get('limiter_scaled_steps', '-')} | {r['solver_ms']:.1f} / {r['rtf']:.2f} |")
    json.dump(S, open(f"{RP}/summary_B.json", "w"), indent=1); open(f"{RP}/tables_B.md", "w").write("\n".join(L) + "\n"); print("\n".join(L))


if __name__ == "__main__":
    main_A() if len(sys.argv) < 2 or sys.argv[1] == "A" else main_B(sys.argv[2:])
