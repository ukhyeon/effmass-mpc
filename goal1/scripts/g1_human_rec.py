"""g1_human_rec.py --out <npz> --stop-file F — /effmass/human_inertia_state 수신 기록 (g1_run_human.sh py 로 실행)"""
import argparse, os, sys, threading, time
import numpy as np, rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from effmass_msgs.msg import HumanInertiaState
ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--stop-file", required=True); a = ap.parse_args()
rclpy.init(); node = rclpy.create_node("g1_human_rec"); R = []
qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
def cb(m):
    R.append((time.time_ns(), m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, m.input_stamp.sec * 10**9 + m.input_stamp.nanosec,
              int(m.input_frame_id.split("/")[-1]) if m.input_frame_id.split("/")[-1].isdigit() else -1, int(m.input_count), m.compute_ms, m.velocity_dt,
              int(m.velocity_valid), np.asarray(m.com_position, np.float32), np.asarray(m.com_velocity, np.float32), np.asarray(m.inv_inertia, np.float32),
              len(m.shape_radius)))
node.create_subscription(HumanInertiaState, "/effmass/human_inertia_state", cb, qos)
ex = SingleThreadedExecutor(); ex.add_node(node); threading.Thread(target=ex.spin, daemon=True).start()
while not os.path.exists(a.stop_file): time.sleep(0.2)
ex.shutdown(); node.destroy_node(); rclpy.shutdown()
np.savez(a.out, t_recv=np.array([r[0] for r in R]), stamp=np.array([r[1] for r in R]), in_stamp=np.array([r[2] for r in R]), frame=np.array([r[3] for r in R]),
         count=np.array([r[4] for r in R]), compute_ms=np.array([r[5] for r in R]), vdt=np.array([r[6] for r in R]), vvalid=np.array([r[7] for r in R]),
         com=np.stack([r[8] for r in R]).reshape(-1, 19, 3), vel=np.stack([r[9] for r in R]).reshape(-1, 19, 3), inv=np.stack([r[10] for r in R]).reshape(-1, 19, 3, 3),
         n_shapes=np.array([r[11] for r in R]))
print(f"[rec] HumanInertiaState 수신 {len(R)}", flush=True)
