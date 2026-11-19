"""rp1_export.py - real_prep1 C: records for the control PC (robot joint values only, no human data).
Usage: rp1_export.py <low-speed profile name, e.g. slowl_acc10>. Runs: acceleration limit 10 (rp1_a10_m<1|3>b_<AB|BA>_<uni|part>_s<seed>) and
the chosen low-speed profile (rp1_<name>_m1b_AB_<uni|part>_s0) (sources goal1/logs/meff_isaac/<tag>_traj.npz and <tag>_cmd.npz).
Outputs, one setting per folder (acc10, <name>):
  goal1/real_prep1/mpc_traj_export/<setting>/<setting>_<scene>.npz  (np.savez_compressed; keys in FORMAT.md)
  goal1/real_prep1/export_mpc_cmd/<setting>/<setting>_<scene>.csv   (same columns as goal1/export_mpc_cmd/: step, t_s, q_cmd1..7,
                                                                      q_meas1..7, feasible; q_cmd = the target sent to the simulator)
  manifest.json in both folders. Prints the file count and total size per setting."""
import csv, glob, json, os, re
import numpy as np
GG = os.path.expanduser("~/effmass_mpc/goal1"); O = f"{GG}/logs/meff_isaac"; RP = f"{GG}/real_prep1"
import sys
SLOW = sys.argv[1] if len(sys.argv) > 1 else "slowl_acc10"   # the chosen low-speed profile
OUT_T = f"{RP}/mpc_traj_export"; OUT_C = f"{RP}/export_mpc_cmd"; MT, MC = [], []
for f in sorted(glob.glob(f"{O}/rp1_*_traj.npz")):
    tag = os.path.basename(f)[:-9]; m = re.match(r"rp1_(a10|" + SLOW + r")_(m[13]b_(AB|BA)_(uni|part)_s\d)$", tag)
    if not m or not os.path.exists(f"{O}/{tag}_cmd.npz") or "saved " not in open(f"{O}/{tag}.log", errors="replace").read():
        continue
    setting = dict(a15="acc15", a10="acc10", a8="acc8").get(m.group(1), m.group(1)); scene = m.group(2)
    z = np.load(f); c = np.load(f"{O}/{tag}_cmd.npz"); r = c["rows"]
    assert np.array_equal(z["step"], r[:, 0].astype(int)), tag
    d = {k: z[k] for k in ("step", "t_sim", "t_wall_ns", "horizon_t", "joint_names", "model_joint_names", "feasible", "first_point_resid")}
    d.update(t_step=z["t_sim"], t_wall_s=((z["t_wall_ns"] - z["t_wall_ns"][0]) / 1e9).astype(np.float64), target_q=z["q"], target_qd=z["qd"], target_qdd=z["qdd"],
             horizon_q=z["horizon_q"], horizon_qd=z["horizon_qd"], horizon_qdd=z["horizon_qdd"], plan_start_q=z["start_q"], plan_start_qd=z["start_qd"],
             sent_q=r[:, 1:8].astype(np.float32), meas_q=r[:, 15:22].astype(np.float32), meas_qd=r[:, 22:29].astype(np.float32), limiter_scale=r[:, 30].astype(np.float32),
             setting=np.array(setting), scene=np.array(scene), robot_cfg=np.array(str(c["robot_cfg"]) or "goal1/fr3v2_1/fr3v2_1.yml (default)"),
             sphere_speed_limit=np.float32(c["sphere_speed_limit"]), source_tag=np.array(tag))
    del d["t_sim"]
    os.makedirs(f"{OUT_T}/{setting}", exist_ok=True); ft = f"{OUT_T}/{setting}/{setting}_{scene}.npz"; np.savez_compressed(ft, **d)
    os.makedirs(f"{OUT_C}/{setting}", exist_ok=True); fc = f"{OUT_C}/{setting}/{setting}_{scene}.csv"
    with open(fc, "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["step", "t_s"] + [f"q_cmd{k}" for k in range(1, 8)] + [f"q_meas{k}" for k in range(1, 8)] + ["feasible"])
        for x in r:
            w.writerow([int(x[0]), f"{(x[0] + 1) / 60.0:.6f}"] + [f"{v:.6f}" for v in x[1:8]] + [f"{v:.6f}" for v in x[15:22]] + [int(x[29])])
    info = dict(setting=setting, scene=scene, tag=tag, steps=int(len(r)), duration_s=round(len(r) / 60.0, 3))
    MT.append(dict(info, file=os.path.relpath(ft, OUT_T), bytes=os.path.getsize(ft))); MC.append(dict(info, file=os.path.relpath(fc, OUT_C), bytes=os.path.getsize(fc)))
for out, M in ((OUT_T, MT), (OUT_C, MC)):
    if M:
        json.dump(M, open(f"{out}/manifest.json", "w"), indent=1)
        by = {}
        for x in M:
            by.setdefault(x["setting"], []).append(x["bytes"])
        print(os.path.basename(out), {k: (len(v), round(sum(v) / 1e6, 2)) for k, v in by.items()}, "total MB %.2f" % (sum(x["bytes"] for x in M) / 1e6))
