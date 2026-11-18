"""lv_replay_pose.py - live prep: replay a precomputed human motion as /hrc/human_pose (HumanPose) in wall time, so the
whole ROS path runs (effmass_human node -> /effmass/human_inertia_state -> step2b --human-source inertia).

  - pose66 per frame from --npz; trans per frame from the npz if it has "trans" (lv_make_live_replay.py), else
    --trans-from (hrc pose npz, frame 0, as r1_motion.py); betas from the npz (zeros in the precomputed files).
  - frame at wall time t: floor((t - t0) * rate), last frame held after the end.
  - /hrc/camera_pose (reliable + transient_local, published once): --camera legacy = the camera used to precompute the goal1 files
    (roll 110.8, t (-0.80, -2.3, 1.55)); with the node's pelvis_correction false the node then reproduces the precomputed file.
  - --wait-file: start (t0) when this file appears (step2b --ready-file), so frame 0 meets the first MPC step.
  - --gaps "s:d,s:d": no message from s to s + d seconds after t0 (tracking loss test).
  - log (--out npz): frame, publish wall ns, stamp ns of every published message.
Run with the ROS environment of hrc_ws + effmass ws (g1_run_human.sh style), ROS_DOMAIN_ID as the run.
"""
import argparse
import json
import os
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from hrc_interfaces.msg import HumanPose

LEGACY = dict(roll_deg=110.8, pitch_deg=0.0, yaw_deg=0.0, t=[-0.80, -2.3, 1.55])

ap = argparse.ArgumentParser()
ap.add_argument("--npz", required=True)
ap.add_argument("--trans-from", default=os.path.expanduser("~/hrc_ws/pose_npz/human_state_1781524848.npz"))
ap.add_argument("--rate", type=float, default=30.0)
ap.add_argument("--duration", type=float, default=65.0, help="seconds after t0")
ap.add_argument("--wait-file", default="")
ap.add_argument("--wait-timeout", type=float, default=300.0)
ap.add_argument("--camera", default="legacy", choices=["legacy", "live", "none"], help="legacy pose, the pose stored in the npz (live, lv_make_live_replay.py) or none (sensing_node publishes it)")
ap.add_argument("--gaps", default="", help="s:d,s:d (seconds after t0)")
ap.add_argument("--out", required=True)
ap.add_argument("--topic", default="/hrc/human_pose", help="HumanPose topic (remap the node to it when sensing also runs)")
a = ap.parse_args()

z = np.load(a.npz)
P = z["pose66"].astype(np.float32).reshape(len(z["pose66"]), -1)
F = len(P)
# per-frame trans from the npz when present (lv_make_live_replay.py output), else trans of --trans-from frame 0 (r1_motion.py)
TR = z["trans"].astype(np.float32).reshape(F, 3) if "trans" in z.files else np.tile(np.load(a.trans_from)["trans"][0].astype(np.float32), (F, 1))
B = z["betas"].astype(np.float32).reshape(-1)[:10] if "betas" in z.files else np.zeros(10, np.float32)
gaps = [tuple(float(v) for v in g.split(":")) for g in a.gaps.split(",") if g]
be = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
tl = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
rclpy.init()
n = rclpy.create_node("lv_replay_pose")
pub = n.create_publisher(HumanPose, a.topic, be)
if a.camera in ("legacy", "live"):
    if a.camera == "live":
        c = np.asarray(z["camera"], np.float64).reshape(-1)
        cam = dict(roll_deg=float(c[0]), pitch_deg=float(c[1]), yaw_deg=float(c[2]), t=[float(v) for v in c[3:6]], source="lv_replay_pose live (npz camera)")
    else:
        cam = dict(LEGACY, source="lv_replay_pose legacy")
    cp = n.create_publisher(String, "/hrc/camera_pose", tl)
    cp.publish(String(data=json.dumps(cam)))
if a.wait_file:
    tw = time.time()
    while not os.path.exists(a.wait_file):
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - tw > a.wait_timeout:
            raise SystemExit(f"no {a.wait_file} within {a.wait_timeout} s")
print(f"[replay] start: {F} frames at {a.rate} Hz, duration {a.duration} s, gaps {gaps}, subscribers {n.count_subscribers(a.topic)}", flush=True)
rec = []
t0 = time.time(); k_last = -1; msg = HumanPose(); msg.smpl_betas = B.tolist()
in_gap_prev = False
try:
    while True:
        t = time.time() - t0
        if t > a.duration:
            break
        k = min(int(np.floor(t * a.rate + 1e-9)), F - 1)
        tick = int(np.floor(t * a.rate + 1e-9))
        if tick != k_last:
            k_last = tick
            in_gap = any(s <= t < s + d for s, d in gaps)
            if in_gap != in_gap_prev:
                print(f"[replay] t {t:.2f} s: {'gap start' if in_gap else 'gap end'}", flush=True)
                in_gap_prev = in_gap
            if not in_gap:
                now = n.get_clock().now()
                msg.header.stamp = now.to_msg(); msg.header.frame_id = f"replay:{k}"
                msg.body_full_pose = P[k].tolist(); msg.global_trans = TR[k].tolist()
                pub.publish(msg)
                rec.append((k, time.time_ns(), now.nanoseconds))
        rclpy.spin_once(n, timeout_sec=0.002)
except KeyboardInterrupt:
    print("[replay] interrupted", flush=True)
finally:   # the log is needed by lv_analyze.py --ros even when the run stops the replay early
    np.savez(a.out, frame=np.array([r[0] for r in rec], np.int64), wall_ns=np.array([r[1] for r in rec], np.int64),
             stamp_ns=np.array([r[2] for r in rec], np.int64), t0_ns=np.int64(int(t0 * 1e9)), gaps=np.array(gaps, np.float64).reshape(-1, 2))
    print(f"[replay] done: {len(rec)} messages, last frame {k_last}, log {a.out}", flush=True)
n.destroy_node()
if rclpy.ok():
    rclpy.shutdown()
