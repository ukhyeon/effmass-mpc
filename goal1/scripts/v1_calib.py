"""v1_calib.py - V1 calibration analysis (system python via g1_run_human.sh py): uniform eta 0.065 / 0.10 / 0.15 / 0.20 m and gain-mode check runs on the
B2a-5 scenes (profile t1). Per run: hand minimum gap (shape = kernel distance with the MPC sphere radius, i.e. the cost's d; mesh), gap / eta (eta of
the condition; gain: eta of the closest pair), side, u, mu at the minimum, arrival, pass time, infeasible, max TCP distance from the line (x-z), EE path,
rollout cost share of non-hand shapes, near-human sphere speed p99. Output logs/meff_isaac/v1_calib.json."""
import json, os, sys, csv, re, glob
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import b2a4_cal as CAL
import shape_metrics as SM
O = BA.O; SC = {"front": "front_0.38h1", "top": "top_0.455"}; ETA = {"u065": 0.065, "u100": 0.10, "u150": 0.15, "u200": 0.20}
ALL = [k for k in range(24) if BA.RAD[k] > 0]
geta = lambda mu: float(np.clip(0.065 + 0.10 * max(np.sqrt(mu / 0.53) - 1, 0), 0.065, 0.25))


def one(job):
    sc, cond, seed = job; key = SC[sc]; tag = f"v1_{key}_{cond}_s{seed}"; js = json.load(open(f"{BA.G}/scene_b2a4_{key}.json")); A_ = np.array(js["tcp_A"])
    r = BA.result(tag); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))]); c = CAL.measure(tag, float(A_[0]))
    B = np.load(f"{O}/{tag}_b1.npz")["rows"]; tcp = B[:, 8:11] - np.array([0, 0, 0.1034]); Cs = np.stack([BA.RI.compute(q)[0] for q in Q])
    gm = np.array([(BA.hand_mesh_signed(C)[0] - BA.RS).min() for C in Cs])
    gap = np.stack([np.stack([SM.segdist(C, BA.P0[k], BA.P1[k]) - BA.RAD[k] for k in ALL], 1).min(1) - BA.RS for C in Cs]); vv = (np.linalg.norm(np.diff(Cs, axis=0), axis=-1) * 60)[gap[1:] <= 0.4]
    eta = ETA.get(cond, geta(c["mu"]) if cond == "gain" else None); arr = r["arrival_list"]
    z = np.load(f"{O}/{tag}_costagg.npz"); nn = z["n"].sum(0); sg = z["shape_segment"]; tot = nn.sum()
    shares = {str(int(g)): float(nn[sg == g].sum() / tot) for g in np.unique(sg) if tot and nn[sg == g].sum() > 0}
    return dict(tag=tag, scene=sc, cond=cond, seed=seed, eta=eta, gap_shape=c["min_gap"], gap_mesh=float(gm.min()), ratio_shape=c["min_gap"] / eta if eta else None,
                ratio_mesh=float(gm.min()) / eta if eta else None, side=c["side"], u_vs_front_deg=c["u_vs_front_deg"], mu=c["mu"], arrived=bool(arr), arrival_step=arr[0]["step"] if arr else None,
                pass_time_s=(arr[0]["step"] + 1) / 60 if arr else None, infeasible=int(r["infeasible_steps"]), steps=len(Q),
                max_dev_line_xz=float(np.sqrt((tcp[:, 0] - A_[0]) ** 2 + (tcp[:, 2] - A_[2]) ** 2).max()), ee_path_m=float(np.linalg.norm(np.diff(tcp, axis=0), axis=1).sum()),
                rollout_share=shares, rollout_non_hand_share=float(1 - shares.get("18", 0.0)) if shares else None, v_near_p99=float(np.percentile(vv, 99)) if len(vv) else None,
                tcp_max_above_hand_top=c["tcp_max_above_hand_top"], last_pos_err=float(np.linalg.norm(tcp[-1] - np.array(js["tcp_B"]))))


if __name__ == "__main__":
    from multiprocessing import Pool
    jobs = [(sc, c, s) for sc in SC for c in list(ETA) + ["gain"] for s in (0, 1) if os.path.exists(f"{O}/v1_{SC[sc]}_{c}_s{s}_b1.npz")]
    with Pool(10) as p:
        R = p.map(one, jobs)
    json.dump(R, open(f"{O}/v1_calib.json", "w"), indent=1)
    for r in R:
        print(f"{r['scene']:5s} {r['cond']:5s} s{r['seed']} eta {r['eta']*100:5.1f} gapS {r['gap_shape']*100:5.2f} gapM {r['gap_mesh']*100:5.2f} rS {r['ratio_shape']:.2f} rM {r['ratio_mesh']:.2f} {r['side']:5s} "
              f"u {r['u_vs_front_deg']:5.1f} mu {r['mu']:.2f} arr {r['arrival_step']} inf {r['infeasible']} dev {r['max_dev_line_xz']*100:5.1f} pass {r['pass_time_s']} nonhand {r['rollout_non_hand_share']:.2f} "
              f"v99 {r['v_near_p99']:.3f} last_err {r['last_pos_err']*100:.1f}cm")
