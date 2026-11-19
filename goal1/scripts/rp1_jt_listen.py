"""rp1_jt_listen.py <out_npz> <timeout_s> - real_prep1 D: subscribe to /mpc/joint_trajectory (trajectory_msgs/JointTrajectory) and
record every message: receive wall time, joint names, header stamp, per point positions / velocities / accelerations and
time_from_start. Stops after <timeout_s> s or 5 s without a message once messages have started. Run with ROS_LOCALHOST_ONLY=1
(g1_run_human.sh py sets ROS_DOMAIN_ID 30 and ROS_LOCALHOST_ONLY 1, as the runner)."""
import sys, time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile
from trajectory_msgs.msg import JointTrajectory
out, to = sys.argv[1], float(sys.argv[2]); R = []
rclpy.init(); n = rclpy.create_node("rp1_jt_listen")


def cb(m):
    R.append(dict(t=time.time_ns(), names=list(m.joint_names), stamp=(m.header.stamp.sec, m.header.stamp.nanosec), q=[list(p.positions) for p in m.points],
                  qd=[list(p.velocities) for p in m.points], qdd=[list(p.accelerations) for p in m.points], tfs=[p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec for p in m.points]))


n.create_subscription(JointTrajectory, "/mpc/joint_trajectory", cb, QoSProfile(depth=100)); t0 = time.time(); last = None
while time.time() - t0 < to:
    rclpy.spin_once(n, timeout_sec=0.1)
    if R and last is None:
        last = time.time()
    if R:
        last = max(last, R[-1]["t"] / 1e9)
        if time.time() - last > 5.0:
            break
print("messages", len(R), "names", R[0]["names"] if R else None, "points", len(R[0]["q"]) if R else None, "stamp", R[0]["stamp"] if R else None, flush=True)
if R:
    np.savez(out, t_recv_ns=np.array([r["t"] for r in R], np.int64), q=np.array([r["q"] for r in R]), qd=np.array([r["qd"] for r in R]), qdd=np.array([r["qdd"] for r in R]),
             tfs=np.array([r["tfs"] for r in R]), stamp=np.array([r["stamp"] for r in R]), names=np.array(R[0]["names"]))
n.destroy_node(); rclpy.shutdown()
