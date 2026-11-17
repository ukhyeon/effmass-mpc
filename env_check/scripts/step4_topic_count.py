"""step4_topic_count.py --out csv --stop-file F — /hrc/worst_head_meff (DirectionalMeffResult) 수신 수 기록 (시스템 Python). 4-3 (C) 확인용"""
import argparse, os, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step3_msgs as M
import rclpy
from rclpy.executors import SingleThreadedExecutor
from hrc_interfaces.msg import DirectionalMeffResult
ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--stop-file", required=True); a = ap.parse_args()
rclpy.init(); node = rclpy.create_node("step4_worst_count"); rec = []
node.create_subscription(DirectionalMeffResult, "/hrc/worst_head_meff", lambda m: rec.append(time.time_ns()), M.best_effort_qos())
ex = SingleThreadedExecutor(); ex.add_node(node); threading.Thread(target=ex.spin, daemon=True).start()
pubs = 0
while not os.path.exists(a.stop_file):
    time.sleep(0.5); pubs = max(pubs, node.count_publishers("/hrc/worst_head_meff"))
ex.shutdown(); node.destroy_node(); rclpy.shutdown()
open(a.out, "w").write("t_recv_ns\n" + "".join(f"{t}\n" for t in rec))
print(f"[worst_count] /hrc/worst_head_meff 수신 {len(rec)}개, 발행자 수 최대 {pubs}", flush=True)
