"""step4_js_sink.py --out csv --stop-file F — /step3/joint_states 수신 기록 (3단계 (b)와 같은 구독 부하 유지용, 시스템 Python)"""
import argparse, csv, os, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step3_msgs as M
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--stop-file", required=True); a = ap.parse_args()
rclpy.init(); node = rclpy.create_node("step4_js_sink"); rec = []
def cb(m):
    t = time.time_ns(); k, s = M.parse_frame_id(m.header.frame_id); rec.append([s, M.stamp_ns(m.header), t] + list(m.position))
node.create_subscription(JointState, "/step3/joint_states", cb, QoSProfile(depth=10))
ex = SingleThreadedExecutor(); ex.add_node(node); threading.Thread(target=ex.spin, daemon=True).start()
while not os.path.exists(a.stop_file): time.sleep(0.2)
ex.shutdown(); node.destroy_node(); rclpy.shutdown()
with open(a.out, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["seq", "stamp_ns", "t_recv_ns"] + [f"p{i}" for i in range(9)]); w.writerows(rec)
print(f"[sink] JointState 수신 {len(rec)}", flush=True)
