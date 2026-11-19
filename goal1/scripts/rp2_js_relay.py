"""rp2_js_relay.py - real_prep2 test (a): joint-state receiver in its own process. Subscribes to sensor_msgs/JointState on --js-topic
(QoS depth 1) and overwrites one record in POSIX shared memory --shm-name (no queue): float64[17] = seq, header stamp (s), receive wall
time (s, time.time), q fr3_joint1..7, qd fr3_joint1..7. seq is odd while the record is written (seqlock); a reader copies the record and
accepts it only if seq is even and unchanged. Creates the block, removes it on exit. Stops after --duration s or a stop file.
Run with system python and ROS_LOCALHOST_ONLY=1 (goal1/scripts/g1_run_human.sh py)."""
import argparse, os, time
import numpy as np
import rclpy
from multiprocessing import shared_memory
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
ap = argparse.ArgumentParser(); ap.add_argument("--js-topic", default="/NS_1/franka/joint_states"); ap.add_argument("--shm-name", default="rp2_js")
ap.add_argument("--duration", type=float, default=900.0); ap.add_argument("--stop-file", default=""); a = ap.parse_args()
NAMES = [f"fr3_joint{k + 1}" for k in range(7)]
try:
    old = shared_memory.SharedMemory(name=a.shm_name); old.close(); old.unlink()
except FileNotFoundError:
    pass
shm = shared_memory.SharedMemory(name=a.shm_name, create=True, size=17 * 8); B = np.ndarray((17,), np.float64, buffer=shm.buf); B[:] = 0.0
rclpy.init(); node = rclpy.create_node("rp2_js_relay"); N = [0]


def cb(m):
    idx = {n: i for i, n in enumerate(m.name)}
    try:
        q = [m.position[idx[n]] for n in NAMES]; v = [m.velocity[idx[n]] for n in NAMES] if len(m.velocity) == len(m.name) else [0.0] * 7
    except KeyError:
        return
    s = B[0]; B[0] = s + 1.0   # odd: writing
    B[1] = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec; B[2] = time.time(); B[3:10] = q; B[10:17] = v
    B[0] = s + 2.0; N[0] += 1


node.create_subscription(JointState, a.js_topic, cb, QoSProfile(depth=1)); t0 = time.time()
print(f"[relay] {a.js_topic} -> shared memory {a.shm_name}", flush=True)
try:
    while time.time() - t0 < a.duration and not (a.stop_file and os.path.exists(a.stop_file)):
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    print(f"[relay] messages {N[0]} in {time.time() - t0:.1f} s", flush=True)
    node.destroy_node(); rclpy.shutdown(); del B; shm.close(); shm.unlink()
