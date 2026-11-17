"""step4_dyn_republish.py — 4-3 (A): 노드 없이, 이전 실행에서 Isaac 이 받은 HumanDynamicsState 기록을 30 Hz 로 다시 발행 (시스템 Python)

  --rec <접두어>   <접두어>_human_rec.npz (dyn_stamp, dyn_pos, dyn_vel) 와 <접두어>_replay_sent.csv 를 읽음
                  재생 구간 [첫 발행, 마지막 발행 + 50 ms] 안에 stamp 가 있는 메시지만 (step4_analyze_41 과 같은 구간), 받은 순서대로
  시작: /hrc/human_pose 첫 수신 즉시 30 Hz 고정 주기로 발행. 메시지는 노드와 같은 필드·QoS (frame_id "world", indices 0..18, names 19개),
        header.stamp = 발행 직전 시각
  --out <접두어>  <접두어>_republish_sent.csv (k, src_index, t_send_ns)
"""
import argparse, array, csv, os, re, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step3_msgs as M
import rclpy
from rclpy.executors import SingleThreadedExecutor
import threading
from hrc_interfaces.msg import HumanDynamicsState, HumanPose
ap = argparse.ArgumentParser(); ap.add_argument("--rec", required=True); ap.add_argument("--out", required=True)
ap.add_argument("--rate", type=float, default=30.0); a = ap.parse_args()
log = lambda *x: print("[republish]", *x, flush=True)
R = np.load(a.rec + "_human_rec.npz")
sent = [int(r["t_send_ns"]) for r in csv.DictReader(open(a.rec + "_replay_sent.csv"))]
w = (R["dyn_stamp"] >= sent[0]) & (R["dyn_stamp"] <= sent[-1] + 50_000_000)
P, V = R["dyn_pos"][w].astype(np.float32), R["dyn_vel"][w].astype(np.float32); idx = np.nonzero(w)[0]
src = open(os.path.expanduser("~/hrc_ws/install/hrc_body_tracking/lib/python3.10/site-packages/hrc_body_tracking/human_dynamics_node.py")).read()
NAMES = re.findall(r"'([^']+)'|\"([^\"]+)\"", src[src.index("self.body_part_names = ["):].split("]", 1)[0])
NAMES = [x or y for x, y in NAMES]
assert len(NAMES) == 19, NAMES
log(f"기록 {a.rec}: 전체 {len(R['dyn_stamp'])}, 재생 구간 {len(P)}개 → {a.rate} Hz 로 {len(P) / a.rate:.1f} s, 이름 {NAMES[:3]}...")
rclpy.init(); node = rclpy.create_node("step4_dyn_republish")
pub = node.create_publisher(HumanDynamicsState, "/hrc/human_dynamics_state", M.best_effort_qos())
first = []
node.create_subscription(HumanPose, "/hrc/human_pose", lambda m: first.append(time.time_ns()) if not first else None, M.best_effort_qos())
ex = SingleThreadedExecutor(); ex.add_node(node); threading.Thread(target=ex.spin, daemon=True).start()
t0 = time.time()
while not first:
    time.sleep(0.002)
    if time.time() - t0 > 400: log("HumanPose 대기 시간 초과"); sys.exit(3)
log(f"첫 HumanPose 수신 ({time.time() - t0:.1f}s 대기), 발행 시작")
period = 1.0 / a.rate; start = time.perf_counter(); rec = []; late = 0
for k in range(len(P)):
    tg = start + k * period; now = time.perf_counter()
    if tg > now: time.sleep(tg - now)
    elif now - tg > period: late += 1
    m = HumanDynamicsState(); m.header.frame_id = "world"
    m.body_part_indices = list(range(19)); m.body_part_names = NAMES
    m.body_part_positions = array.array("f", P[k].reshape(-1).tobytes())
    m.body_part_velocities = array.array("f", V[k].reshape(-1).tobytes())
    ts = M.stamp_now(m.header); pub.publish(m); rec.append((k, int(idx[k]), ts))
el = time.perf_counter() - start
log(f"발행 완료 {len(P)}개, {el:.2f}s, 실제 {len(P) / el:.2f} Hz, 한 주기 이상 늦은 발행 {late}")
with open(a.out + "_republish_sent.csv", "w", newline="") as f:
    cw = csv.writer(f); cw.writerow(["k", "src_index", "t_send_ns"]); cw.writerows(rec)
ex.shutdown(); node.destroy_node(); rclpy.shutdown()
