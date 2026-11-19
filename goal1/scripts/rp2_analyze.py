"""rp2_analyze.py <name> ... - real_prep2 tables from goal1/real_prep2/<name>_steps.npz, _summary.json, _fake.npz, _sys.csv.
Per case: wall-clock message period (sent by the MPC; received by the fake robot), transfer latency (receive - send, same message order),
MPC solve time, step computation (start to publish), overrun share (> 1/60 s), deadline misses, where the time of the overrun steps goes
(solve / state read / horizon + limiter + publish / goal check after publish), differenced acceleration of the sent first points (> 10 rad/s^2:
count, excess per joint, at a goal switch or not), infeasible steps, robot-sphere speed of the sent targets (numpy FR3 model, successive
first points / (1/60 s) and per actual wall interval), CPU of the processes (sys monitor). Writes goal1/real_prep2/summary.json, tables.md."""
import csv, json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lv_corr_lib as CL
RP = os.path.expanduser(os.environ.get("RP2_DIR", "~/effmass_mpc/goal1/real_prep2")); DT = 1.0 / 60.0   # RP2_DIR: other result folder (real_prep3)


def st(x):
    x = np.asarray(x, float); return dict(mean=float(x.mean()), p95=float(np.percentile(x, 95)), p99=float(np.percentile(x, 99)), max=float(x.max()))


def one(nm):
    global RP
    if "/" in nm:   # "<folder>/<name>": a run in another result folder (g1_run_human.sh clears the environment, so RP2_DIR does not reach here)
        RP, nm = os.path.dirname(os.path.expanduser(nm)), os.path.basename(nm)
    R = np.load(f"{RP}/{nm}_steps.npz"); S = json.load(open(f"{RP}/{nm}_summary.json")); F = np.load(f"{RP}/{nm}_fake.npz"); N = len(R["t_pub"])
    tp = R["t_pub_ns"] / 1e9; per = np.diff(tp) * 1000; sec = np.floor(tp - tp[0]).astype(int); cnt = np.bincount(sec)[:-1]
    rt = F["recv_t_ns"] / 1e9; lat = (rt[:N] - tp[:len(rt)]) * 1000 if len(rt) >= N - 1 else None; rper = np.diff(rt) * 1000
    solve = (R["t_solve1"] - R["t_solve0"]) * 1000; comp = (R["t_pub"] - R["t_start"]) * 1000; pre = (R["t_solve0"] - R["t_start"]) * 1000; post = (R["t_pub"] - R["t_solve1"]) * 1000
    goal = (R["t_end"] - R["t_pub"]) * 1000; lag = (R["t_start"] - R["t_sched"]) * 1000; late = (R["t_pub"] - R["t_sched"]) * 1000
    ov = comp > DT * 1000; gaps = per > DT * 1000 * 1.5
    q = R["sent_q"].astype(float); a2 = np.abs(np.diff(q, 2, axis=0)) * 3600.0; over = a2 > 10; sw = np.r_[False, np.diff(R["goal_idx"]) != 0]
    near_sw = np.array([sw[max(i - 2, 0):i + 3].any() for i in range(1, N - 1)])
    exc = {j + 1: (int(over[:, j].sum()), float((a2[over[:, j], j] - 10).max()) if over[:, j].any() else None, float(np.median(a2[over[:, j], j] - 10)) if over[:, j].any() else None) for j in range(7)}
    C = np.stack([CL.spheres(x) for x in q]); sv = np.linalg.norm(np.diff(C, axis=0), axis=2).max(1); spd_nom = sv / DT; spd_wall = sv / np.maximum(np.diff(R["t_pub"]), 1e-4)
    out = dict(name=nm, profile=os.path.basename(S["profile"]), tau_ms=float(F["tau_ms"]), k=S["k"], steps=N, wall_s=float(tp[-1] - tp[0]),
               sent_per_s=float((N - 1) / (tp[-1] - tp[0])), per_s_min=int(cnt.min()), per_s_max=int(cnt.max()), period_ms=st(per), period_min_ms=float(per.min()), gaps_over_25ms=int(gaps.sum()),
               recv_per_s=float((len(rt) - 1) / (rt[-1] - rt[0])), recv_period_ms=st(rper), received=int(len(rt)), latency_ms=None if lat is None else st(lat),
               solve_ms=st(solve), step_ms=st(comp), overrun_steps=int(ov.sum()), overrun_pct=float(100 * ov.mean()), deadline_miss=int((late > DT * 1000).sum()), deadline_miss_pct=float(100 * (late > DT * 1000).mean()),
               late_start_ms=st(lag), overrun_parts_ms=dict(solve=float(solve[ov].mean()) if ov.any() else None, before_solve=float(pre[ov].mean()) if ov.any() else None, after_solve_to_publish=float(post[ov].mean()) if ov.any() else None),
               overrun_by_solve=int((ov & (solve > DT * 1000)).sum()), parts_all_ms=dict(before_solve=st(pre), after_solve_to_publish=st(post), goal_check=st(goal)),
               infeasible=int((R["feasible"] == 0).sum()), arrivals=S["arrivals"], diffacc_over10_steps=int(over.any(1).sum()), diffacc_over10_near_goal_switch=int((over.any(1) & near_sw).sum()),
               diffacc_excess_per_joint=exc, diffacc_max_per_joint=a2.max(0).tolist(), limiter_scaled_steps=int((R["lim_scale"] < 1).sum()),
               sphere_speed_nominal=dict(max=float(spd_nom.max()), p99=float(np.percentile(spd_nom, 99))), sphere_speed_wall=dict(max=float(spd_wall.max()), p99=float(np.percentile(spd_wall, 99))),
               state_age_ms=S["state_age_ms"], js_received=S["js_received"], js_source=S.get("js_source", "topic"), age_comp=S.get("age_comp", 0), fake_rate_hz=float(F["rate_hz"]) if "rate_hz" in F.files else 1000.0,
               age_by_receive_ms=dict(mean=float(R["state_age"].mean() * 1000), std=float(R["state_age"].std() * 1000), max=float(R["state_age"].max() * 1000)),
               age_by_stamp_ms=None if "stamp_age" not in R.files else dict(mean=float(R["stamp_age"].mean() * 1000), std=float(R["stamp_age"].std() * 1000), max=float(R["stamp_age"].max() * 1000)))
    if "clipped" in R.files:   # (real_prep3) acceleration clip
        cl = R["clipped"].astype(bool); dq = np.abs(R["clip_dq"].astype(float))
        out.update(acc_clip=S.get("acc_clip"), acc_max=S.get("acc_max"), clipped_steps=int(cl.sum()), clipped_pct=float(100 * cl.mean()), clip_dq_max_per_joint_mrad=(dq.max(0) * 1000).tolist(),
                   clip_dq_median_mrad=float(np.median(dq[cl].max(1)) * 1000) if cl.any() else None, clipped_first_10_steps=int(cl[:10].sum()),
                   acc_pre_over_lim_steps=int((np.abs(R["acc_pre"]).max(1) > (S.get("acc_max") or 1e9)).sum()), diffacc_over95_steps=int((a2.max(1) > 9.5 + 1e-6).sum()), diffacc_max=float(a2.max()))
    if os.path.exists(f"{RP}/{nm}_sys.csv"):
        rows = list(csv.DictReader(open(f"{RP}/{nm}_sys.csv"))); keys = rows[0].keys() if rows else []
        out["sys_columns"] = list(keys)[:12]
        for p in ("mpc", "fake", "relay"):
            k_ = [k for k in keys if k.startswith(p) and "cpu" in k]
            if k_:
                v = [float(r[k_[0]]) for r in rows if r[k_[0]] not in ("", "nan")]; out[f"cpu_{p}_pct"] = dict(mean=float(np.mean(v)), max=float(np.max(v))) if v else None
    return out


if __name__ == "__main__":
    OUTD = RP; A = []
    for n in sys.argv[1:]:
        A.append(one(n)); A[-1]["folder"] = RP
    OUTD = os.path.dirname(os.path.expanduser(sys.argv[-1])) if "/" in sys.argv[-1] else OUTD
    json.dump(A, open(f"{OUTD}/summary.json", "w"), indent=1)
    for a in A:
        print(json.dumps(a))
