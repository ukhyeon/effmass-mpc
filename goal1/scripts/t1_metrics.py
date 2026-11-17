"""t1_metrics.py <set> [tag ...] - T1 tuning metrics from saved runs (system python via g1_run_human.sh py, parallel over runs).
Per run (executed joint commands): robot sphere speed |c_i - c_(i-1)| * 60 for the MPC spheres whose shape-based gap to any human shape (MPC radius) is
<= 0.4 m at step i (p95, p99, max over all such sphere-steps); gripper rotation about the vertical vs the goal at the closest moment (shape-based hand
minimum) and max inside the pass window; arrival, infeasible; recorded only: max TCP distance from the line (x-z), pass time, EE path, hand gap shape /
mesh, side, speed-cap rollout counts. Output logs/meff_isaac/t1_<set>_metrics.json."""
import json, os, sys, csv, glob, re
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import b2a4_cal as CAL
import shape_metrics as SM
O = BA.O; RI = BA.RI; HB, HT = RI.body_of["fr3v2_1_hand"]; HY = json.load(open(f"{BA.G}/lines.json"))["hand_com"][1]
ALL = [k for k in range(24) if BA.RAD[k] > 0]


def one(tag):
    m = re.match(r"t1_(\w+?)_((?:front|top)_[\w.]+?)_(off|unim|iso|massA|uni10)_s(\d)", tag); key = m.group(2)
    js = json.load(open(f"{BA.G}/scene_b2a4_{key}.json")); A_ = np.array(js["tcp_A"])
    r = BA.result(tag); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))])
    Cs = np.stack([RI.compute(q)[0] for q in Q]); gap = np.stack([np.stack([SM.segdist(C, BA.P0[k], BA.P1[k]) - BA.RAD[k] for k in ALL], 1).min(1) - BA.RS for C in Cs])
    v = np.linalg.norm(np.diff(Cs, axis=0), axis=-1) * 60.0; near = gap[1:] <= 0.4; vv = v[near]
    c = CAL.measure(tag, float(A_[0])); B = np.load(f"{O}/{tag}_b1.npz")["rows"]; tcp = B[:, 8:11] - np.array([0, 0, 0.1034]); w = np.abs(tcp[:, 1] - HY) <= 0.12
    Rh = np.stack([(RI._fk(q)[HB + 1] @ HT)[:3, :3] for q in Q]); yaw = np.degrees(np.arctan2(Rh[:, 1, 0], Rh[:, 0, 0]))
    gm = np.full(len(Q), np.inf)
    for i in range(len(Q)):
        d, _ = BA.hand_mesh_signed(Cs[i]); gm[i] = (d - BA.RS).min()
    arr = r["arrival_list"]; out = dict(tag=tag, scene=key.split("_")[0], cond=m.group(3), seed=int(m.group(4)), steps=len(Q),
        v_near_p95=float(np.percentile(vv, 95)) if len(vv) else None, v_near_p99=float(np.percentile(vv, 99)) if len(vv) else None, v_near_max=float(vv.max()) if len(vv) else None, n_near=int(len(vv)),
        tcp_speed_max=float(np.linalg.norm(np.diff(tcp, axis=0), axis=1).max() * 60), yaw_at_min_deg=float(abs(yaw[c["step"]])), yaw_window_max_deg=float(np.abs(yaw[w]).max()) if w.any() else None,
        yaw_run_max_deg=float(np.abs(yaw).max()), arrived=bool(arr), arrival_step=(arr[0]["step"] if arr else None), pass_time_s=((arr[0]["step"] + 1) / 60 if arr else None),
        infeasible=int(r["infeasible_steps"]), rtf=float(r["realtime_factor"]), solver_ms_mean=float(r["solver_ms_mean"]),
        max_dev_line_xz=float(np.sqrt((tcp[:, 0] - A_[0]) ** 2 + (tcp[:, 2] - A_[2]) ** 2).max()), ee_path_m=float(np.linalg.norm(np.diff(tcp, axis=0), axis=1).sum()),
        hand_gap_shape=c["min_gap"], hand_gap_mesh=float(gm.min()), side=c["side"], u_vs_front_deg=c["u_vs_front_deg"], mu=c["mu"])
    ca = f"{O}/{tag}_costagg.npz"
    if os.path.exists(ca):
        z = np.load(ca)
        if "speed_n" in z.files:
            out.update(speed_cost_steps=int((z["speed_n"] > 0).sum()), speed_cost_count=float(z["speed_n"].sum()))
    return out


if __name__ == "__main__":
    from multiprocessing import Pool
    st = sys.argv[1]; tags = sys.argv[2:] or sorted({os.path.basename(f)[:-7] for f in glob.glob(f"{O}/t1_{st}_*_s[0-9]_b1.npz")})
    with Pool(min(10, len(tags))) as p:
        res = p.map(one, tags)
    json.dump(res, open(f"{O}/t1_{st}_metrics.json", "w"), indent=1)
    for r in res:
        print(f"{r['tag']:42s} v99 {r['v_near_p99']:.3f} vmax {r['v_near_max']:.2f} yaw@min {r['yaw_at_min_deg']:5.1f} yawwin {r['yaw_window_max_deg']:5.1f} arr {r['arrival_step']} inf {r['infeasible']} "
              f"dev {r['max_dev_line_xz']*100:5.1f} pass {r['pass_time_s']} path {r['ee_path_m']:.3f} gapS {r['hand_gap_shape']*100:5.2f} gapM {r['hand_gap_mesh']*100:5.2f} {r['side']} sc {r.get('speed_cost_steps')}")
