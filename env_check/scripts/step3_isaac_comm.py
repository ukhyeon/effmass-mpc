"""step3_isaac_comm.py — 3-1 Isaac 쪽 시험 노드 (ROS_METHOD=min run_isaac_ros.sh 로 실행)

  SimulationApp(headless) 안에서 /step3/human_pose(HumanPose), /step3/human_dynamics_state(HumanDynamicsState)를 구독하고
  /step3/joint_states(JointState, 규칙 값)를 60 Hz 로 발행한다. ROS 수신은 별도 스레드의 SingleThreadedExecutor.spin().
  메인 루프: simulation_app.update() + 60 Hz 일정에 맞춘 JointState 발행. --end-file 이 생기고 1.5 s 뒤 종료.
  출력: <out>_isaac_recv_pose.csv, <out>_isaac_recv_dyn.csv, <out>_isaac_sent_js.csv, <out>_isaac_meta.json
"""
import argparse, csv, json, os, sys, threading, time
ap = argparse.ArgumentParser()
ap.add_argument("--ready-file", required=True)
ap.add_argument("--end-file", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--timeout", type=float, default=240)
a = ap.parse_args()
log = lambda *x: print("[isaac]", *x, flush=True)
T0 = time.time()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step3_ros_common as C
added = C.add_ros_paths()
import numpy as np, torch
import step3_msgs as M
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from hrc_interfaces.msg import HumanPose, HumanDynamicsState
from sensor_msgs.msg import JointState
log(f"SimulationApp ready {time.time()-T0:.1f}s; rclpy {C.short(rclpy.__file__)}; numpy {C.short(np.__file__)}; torch {C.short(torch.__file__)}; "
    f"ROS 경로 {len(added)}개를 sys.path 뒤에 추가")

rclpy.init()
node = rclpy.create_node("step3_isaac_node")
recv = {"pose": [], "dyn": []}
def on_pose(msg):
    t = time.time_ns(); kind, seq = M.parse_frame_id(msg.header.frame_id)
    recv["pose"].append([kind, seq, M.stamp_ns(msg.header), t, int(M.check_pose(msg, seq, full=True))])
def on_dyn(msg):
    t = time.time_ns(); kind, seq = M.parse_frame_id(msg.header.frame_id)
    recv["dyn"].append([kind, seq, M.stamp_ns(msg.header), t, int(M.check_dyn(msg, seq))])
node.create_subscription(HumanPose, "/step3/human_pose", on_pose, M.best_effort_qos())
node.create_subscription(HumanDynamicsState, "/step3/human_dynamics_state", on_dyn, M.best_effort_qos())
pub_js = node.create_publisher(JointState, "/step3/joint_states", QoSProfile(depth=10))
ex = SingleThreadedExecutor(); ex.add_node(node)
th = threading.Thread(target=ex.spin, daemon=True); th.start()
open(a.ready_file, "w").write(str(time.time_ns()))
log("READY")

sent_js = []; seq = 0; period = 1 / 60; start = time.perf_counter(); end_seen = None; upd_ms = []
while True:
    now = time.perf_counter()
    target = start + seq * period
    if now >= target:
        msg = M.fill_js(JointState(), seq)
        t = M.stamp_now(msg.header); pub_js.publish(msg); sent_js.append([seq, t]); seq += 1
    u0 = time.perf_counter(); app.update(); upd_ms.append((time.perf_counter() - u0) * 1000)
    rem = start + seq * period - time.perf_counter()
    if rem > 0: time.sleep(min(rem, 0.002))
    if end_seen is None and os.path.exists(a.end_file): end_seen = time.time()
    if end_seen and time.time() - end_seen > 1.5: break
    if time.time() - T0 > a.timeout: log("시간 초과로 종료"); break
el = time.perf_counter() - start
ex.shutdown(); node.destroy_node(); rclpy.shutdown()
for k in ("pose", "dyn"):
    with open(f"{a.out}_isaac_recv_{k}.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["kind", "seq", "stamp_ns", "t_recv_ns", "ok"]); w.writerows(recv[k])
with open(f"{a.out}_isaac_sent_js.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["seq", "t_send_ns"]); w.writerows(sent_js)
json.dump(dict(js_sent=seq, js_rate_hz=seq / el, app_update_ms_mean=float(np.mean(upd_ms)),
               app_update_ms_p95=float(np.percentile(upd_ms, 95))), open(f"{a.out}_isaac_meta.json", "w"))
log(f"완료: JointState 발행 {seq}개 ({seq/el:.2f} Hz), 수신 pose {len(recv['pose'])} dyn {len(recv['dyn'])}, "
    f"app.update 평균 {np.mean(upd_ms):.2f} ms")
app.close()
