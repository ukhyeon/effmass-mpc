"""lv_stand_monitor.py - live: where the person is, once per second, from /effmass/human_inertia_state (hrc_ws world, node output).

Prints the pelvis position in world, its offset from the standing place of scene_live_front.json, the horizontal distances to the
Franka base and to the camera, and the right hand (COM) in the Franka base frame next to the robot path (x of the path, |y| <= 0.37).
Run in the effmass ROS environment (g1_run_human.sh py lv_stand_monitor.py).
"""
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from effmass_msgs.msg import HumanInertiaState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g1_human_link as L

GG = os.path.expanduser("~/effmass_mpc/goal1")
import argparse
ap = argparse.ArgumentParser(); ap.add_argument("--frame", default=f"{GG}/config/live_frame.yml"); a = ap.parse_args()
sc = json.load(open(f"{GG}/b2a/g2/scene_live_front.json")); cam = np.asarray(sc["live"]["camera_world_xy"], float)
R, t = L.load_frame(a.frame)
# standing place in world = the base-frame place of the scene mapped back through this frame (live_frame_user.yml: where you stood)
target = (R.T @ (np.r_[sc["live"]["stand_pelvis_base_xy"], 0.0] - t))[:2]
path_x = float(sc["tcp_A"][0]); path_z = float(sc["tcp_A"][2])
be = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
rclpy.init(); n = rclpy.create_node("lv_stand_monitor"); buf = []
n.create_subscription(HumanInertiaState, "/effmass/human_inertia_state", lambda m: buf.append(np.array(m.com_position, float).reshape(-1, 3)), be)
print(f"frame file {a.frame}")
print(f"standing place: pelvis above world ({target[0]:+.2f}, {target[1]:+.2f}); robot path in base frame x {path_x:.2f}, z {path_z:.2f}, y -0.37..+0.37", flush=True)
last = time.time()
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - last < 1.0:
        continue
    last = time.time()
    if not buf:
        print(time.strftime("%H:%M:%S"), "no human yet (sensing estimates the body shape first; stand in view)", flush=True); continue
    P = np.mean(buf, axis=0); buf = []
    pel = P[0]; d = pel[:2] - target
    hb = R @ P[18] + t
    ok = "  <- OK" if np.hypot(*d) <= 0.05 else ""
    print(f"{time.strftime('%H:%M:%S')}  pelvis world ({pel[0]:+.2f}, {pel[1]:+.2f})  move ({-d[0]:+.2f} x, {-d[1]:+.2f} y){ok}  "
          f"base {np.hypot(*(R @ pel + t)[:2]):.2f} m  camera {np.hypot(*(pel[:2] - cam)):.2f} m  |  right hand base x {hb[0]:.2f} (path {path_x:.2f}) "
          f"y {hb[1]:+.2f} z {hb[2]:.2f} (path {path_z:.2f})", flush=True)
