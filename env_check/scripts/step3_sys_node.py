"""step3_sys_node.py — 시스템 쪽 시험 노드 (run_sys_ros.sh 로 실행)

  --plan "pose30:30:20,pose60:60:20,dyn30:30:20"   kind:Hz:초 를 차례로 발행 (kind 가 pose* 면 HumanPose, dyn* 면 HumanDynamicsState)
  --until-file <파일>   이 파일이 생기면 발행 중단 (3-2: Isaac 측정이 끝날 때까지 계속 발행할 때. 이때 plan 의 초는 상한)
  --ready-file <파일>   Isaac 쪽이 만드는 준비 표시. 생길 때까지 기다린 뒤 시작
  --end-file <파일>     발행을 마치면 만든다 (Isaac 쪽 종료 신호)
  --js-check rule|log   받은 JointState 검사: rule = 3-1 규칙 값과 비교, log = 값만 기록(3-2, 나중에 Isaac 기록과 비교)
  --out <접두어>        <접두어>_sys_sent.csv, <접두어>_sys_recv_js.csv, <접두어>_sys_meta.json
ROS 수신은 별도 스레드의 SingleThreadedExecutor.spin() 이 처리한다.
"""
import argparse, csv, json, os, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step3_msgs as M
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from hrc_interfaces.msg import HumanPose, HumanDynamicsState
from sensor_msgs.msg import JointState

ap = argparse.ArgumentParser()
ap.add_argument("--plan", required=True)
ap.add_argument("--until-file", default=None)
ap.add_argument("--ready-file", required=True)
ap.add_argument("--end-file", required=True)
ap.add_argument("--js-check", default="rule")
ap.add_argument("--out", required=True)
a = ap.parse_args()
log = lambda *x: print("[sys]", *x, flush=True)
log(f"python {sys.version.split()[0]} {sys.executable}, rclpy {rclpy.__file__}, ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} "
    f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY')}")

rclpy.init()
node = rclpy.create_node("step3_sys_node")
pub_pose = node.create_publisher(HumanPose, "/step3/human_pose", M.best_effort_qos())
pub_dyn = node.create_publisher(HumanDynamicsState, "/step3/human_dynamics_state", M.best_effort_qos())
recv_js = []
def on_js(msg):
    t = time.time_ns()
    kind, seq = M.parse_frame_id(msg.header.frame_id)
    ok = M.check_js_rule(msg, seq) if a.js_check == "rule" else None
    recv_js.append([seq, M.stamp_ns(msg.header), t, "" if ok is None else int(ok)] + list(msg.position))
node.create_subscription(JointState, "/step3/joint_states", on_js, QoSProfile(depth=10))
ex = SingleThreadedExecutor(); ex.add_node(node)
th = threading.Thread(target=ex.spin, daemon=True); th.start()

t0 = time.time()
while not os.path.exists(a.ready_file):
    if time.time() - t0 > 300: log("READY 대기 시간 초과"); sys.exit(3)
    time.sleep(0.2)
t1 = time.time()
while (pub_pose.get_subscription_count() == 0 or pub_dyn.get_subscription_count() == 0) and time.time() - t1 < 30:
    time.sleep(0.1)
log(f"READY 확인, 구독자 수 pose {pub_pose.get_subscription_count()} dyn {pub_dyn.get_subscription_count()} "
    f"(대기 {time.time()-t0:.1f}s)")
time.sleep(0.5)
listen_start_ns = time.time_ns()

sent = []
for item in a.plan.split(","):
    kind, hz, dur = item.split(":"); hz = float(hz); dur = float(dur)
    is_pose = kind.startswith("pose")
    period = 1.0 / hz; n_max = int(round(hz * dur)); seq = 0
    start = time.perf_counter(); late = 0
    log(f"phase {kind}: {hz:.0f} Hz, 최대 {n_max}개")
    while seq < n_max:
        if a.until_file and os.path.exists(a.until_file):
            log(f"phase {kind}: until-file 감지, {seq}개에서 중단"); break
        target = start + seq * period
        now = time.perf_counter()
        if target > now:
            time.sleep(target - now)
        elif now - target > period:
            late += 1
        msg = M.fill_pose(HumanPose(), seq, kind) if is_pose else M.fill_dyn(HumanDynamicsState(), seq, kind)
        tsend = M.stamp_now(msg.header)
        (pub_pose if is_pose else pub_dyn).publish(msg)
        sent.append([kind, seq, tsend]); seq += 1
    el = time.perf_counter() - start
    log(f"phase {kind}: {seq}개 발행, {el:.2f}s, 실제 {seq/el:.2f} Hz, 한 주기 이상 늦은 발행 {late}개")
    time.sleep(1.0)

with open(a.end_file, "w") as f: f.write(str(time.time_ns()))
time.sleep(1.5)
listen_end_ns = time.time_ns()
ex.shutdown(); node.destroy_node(); rclpy.shutdown()
with open(a.out + "_sys_sent.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["kind", "seq", "t_send_ns"]); w.writerows(sent)
with open(a.out + "_sys_recv_js.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["seq", "stamp_ns", "t_recv_ns", "ok"] + [f"p{i}" for i in range(9)]); w.writerows(recv_js)
json.dump(dict(listen_start_ns=listen_start_ns, listen_end_ns=listen_end_ns, n_sent=len(sent), n_recv_js=len(recv_js)),
          open(a.out + "_sys_meta.json", "w"))
log(f"완료: 발행 {len(sent)}개, JointState 수신 {len(recv_js)}개")
