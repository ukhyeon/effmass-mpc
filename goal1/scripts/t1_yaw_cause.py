"""t1_yaw_cause.py - gripper rotation about the vertical vs the goal over time in saved off runs (B2a-4 scenes), with TCP speed and the goal
orientation check at the end (system python via g1_run_human.sh py). Output logs/meff_isaac/t1_yaw_cause.json."""
import json, os, sys, csv
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
RI = BA.RI; HB, HT = RI.body_of["fr3v2_1_hand"]; out = {}
for tag in ("b2a4_front_0.31h1_off_s0", "b2a4_front_0.31h1_off_s1", "b2a4_top_0.514_off_s0", "b2a4_top_0.514_off_s1"):
    r = BA.result(tag); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))])
    Rh = np.stack([(RI._fk(q)[HB + 1] @ HT)[:3, :3] for q in Q]); yaw = np.degrees(np.arctan2(Rh[:, 1, 0], Rh[:, 0, 0])); tilt = np.degrees(np.arccos(np.clip(-Rh[:, 2, 2], -1, 1)))
    B = np.load(f"{BA.O}/{tag}_b1.npz")["rows"]; tcp = B[:, 8:11] - np.array([0, 0, 0.1034]); v = np.linalg.norm(np.gradient(tcp, 1 / 60, axis=0), axis=1)
    i = int(np.argmax(np.abs(yaw))); arr = r["arrival_list"][0]["step"]
    out[tag] = dict(yaw_start=float(yaw[0]), yaw_max=float(yaw[i]), yaw_max_step=i, speed_at_yaw_max=float(v[i]), speed_max=float(v.max()), speed_max_step=int(np.argmax(v)),
                    yaw_at_arrival=float(yaw[arr]), yaw_end=float(yaw[-1]), tilt_max=float(tilt.max()), arrival=arr, corr_abs_yaw_speed=float(np.corrcoef(np.abs(yaw), v)[0, 1]),
                    yaw_series=yaw[::5].round(1).tolist(), speed_series=v[::5].round(2).tolist())
    print(tag, {k: (round(x, 2) if isinstance(x, float) else x) for k, x in out[tag].items() if "series" not in k})
json.dump(out, open(f"{BA.O}/t1_yaw_cause.json", "w"), indent=1)
