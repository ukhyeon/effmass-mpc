"""step4_replay.py — 저장된 SMPL 동작(pose_npz)을 HumanPose 로 재생 (시스템 Python, run_sys_ros.sh -B 로 실행)

  --npz <파일>          ~/hrc_ws/pose_npz/human_state_*.npz (읽기만)
  --rate 30             발행 주기 (Hz). 파일에 시각이 없어 sensing 설정 fps 30 / 노드 dt 1/30 에 맞춤
  --min-subs 2          /hrc/human_pose 구독자가 이 수 이상 될 때까지 대기 (human_dynamics_node + Isaac)
  --ready-file <파일>   Isaac 쪽 준비 표시 (생길 때까지 대기)
  --end-file <파일>     재생을 마치고 2 s 뒤 만든다
  --out <접두어>        <접두어>_replay_sent.csv (seq, t_send_ns), <접두어>_replay_meta.json
  --vertices            (4-5) smplx 로 betas=0 정점을 계산해 smpl_vertices 에 싣는다

메시지 (정의 변경 없음): header.stamp = 발행 직전 시각, header.frame_id = "step4/pose/<seq>" (순번 표시용, 노드는 읽지 않음),
  body_full_pose = poses[seq] (66), global_trans = trans[seq] (3), smpl_betas = 0 (10, 환경 점검용 평균 체형), 나머지 비움.
QoS: sensing_node 의 /hrc/human_pose 와 같음 (KEEP_LAST 5, BEST_EFFORT, VOLATILE).
"""
import argparse, array, csv, json, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step3_msgs as M
import rclpy
from hrc_interfaces.msg import HumanPose

ap = argparse.ArgumentParser()
ap.add_argument("--npz", required=True)
ap.add_argument("--rate", type=float, default=30.0)
ap.add_argument("--min-subs", type=int, default=2)
ap.add_argument("--ready-file", required=True)
ap.add_argument("--end-file", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--vertices", action="store_true")
ap.add_argument("--max-frames", type=int, default=0)
ap.add_argument("--start-wait", type=float, default=1.0, help="구독자 확인 후 시작 전 대기(s): 노드 초기화(모델·GPU) 시간")
ap.add_argument("--drop-every", type=int, default=0, help="(goal1) N>0 이면 N 번째 프레임마다 발행하지 않음 (시각은 그대로 흐름 → 프레임 차 2)")
a = ap.parse_args()
log = lambda *x: print("[replay]", *x, flush=True)

z = np.load(a.npz)
P = z["poses"].astype(np.float32); T = z["trans"].astype(np.float32)
if a.max_frames: P, T = P[:a.max_frames], T[:a.max_frames]
N = len(P)
BETAS = np.zeros(10, np.float32)
VERTS = None
if a.vertices:
    import torch, smplx
    m = smplx.create(os.path.expanduser("~/realtime_smpl/pykinect/body_models"), model_type="smpl", gender="neutral",
                     batch_size=1, create_transl=False)
    m.eval()
    # (4-5) 저장 자세는 SMPL-H 몸 22관절(66) → SMPL 24관절 중 손 2개(22, 23)는 0. 발행 주기를 흔들지 않도록 재생 시작 전에 전 프레임 정점을 미리 계산 (betas 0, 카메라 좌표 = sensing_node 와 같은 기준)
    v0 = time.perf_counter(); VA = np.zeros((N, 6890, 3), np.float32)
    with torch.no_grad():
        for s0 in range(0, N, 64):
            s1 = min(N, s0 + 64); nb = s1 - s0
            o = m(betas=torch.zeros(nb, 10), body_pose=torch.from_numpy(np.concatenate([P[s0:s1, 3:], np.zeros((nb, 6), np.float32)], 1)), global_orient=torch.from_numpy(P[s0:s1, :3]),
                  transl=torch.from_numpy(T[s0:s1]), return_verts=True, pose2rot=True)
            VA[s0:s1] = o.vertices.numpy()
    VPRE_S = time.perf_counter() - v0
    print(f"[replay] 정점 미리 계산 {N}프레임 {VPRE_S:.2f}s (프레임당 {VPRE_S / N * 1000:.2f} ms), 메시지당 {VA[0].size} float", flush=True)
    def VERTS(k):
        return VA[k]
log(f"npz {a.npz} 프레임 {N}, {a.rate} Hz → {N / a.rate:.1f} s, betas 0, vertices {'on' if a.vertices else 'off'}")

rclpy.init()
node = rclpy.create_node("step4_replay")
pub = node.create_publisher(HumanPose, "/hrc/human_pose", M.best_effort_qos())
t0 = time.time()
while not os.path.exists(a.ready_file) or pub.get_subscription_count() < a.min_subs:
    rclpy.spin_once(node, timeout_sec=0.1)
    if time.time() - t0 > 300:
        log(f"대기 시간 초과: ready={os.path.exists(a.ready_file)} subs={pub.get_subscription_count()}"); sys.exit(3)
log(f"구독자 {pub.get_subscription_count()} 확인 ({time.time() - t0:.1f}s), {a.start_wait:.0f}s 더 기다린 뒤 시작")
time.sleep(a.start_wait)

sent = []; vt = []
period = 1.0 / a.rate; start = time.perf_counter(); late = 0
for k in range(N):
    target = start + k * period
    now = time.perf_counter()
    if target > now:
        time.sleep(target - now)
    elif now - target > period:
        late += 1
    if a.drop_every > 0 and k % a.drop_every == a.drop_every - 1:
        continue
    msg = HumanPose()
    msg.header.frame_id = f"step4/pose/{k}"
    msg.body_full_pose = array.array("f", P[k].tobytes())
    msg.global_trans = array.array("f", T[k].tobytes())
    msg.smpl_betas = array.array("f", BETAS.tobytes())
    if VERTS is not None:
        v0 = time.perf_counter(); V = VERTS(k); vt.append((time.perf_counter() - v0) * 1000)
        msg.smpl_vertices = array.array("f", V.reshape(-1).tobytes())
    ts = M.stamp_now(msg.header)
    pub.publish(msg)
    sent.append([k, ts])
el = time.perf_counter() - start
log(f"재생 완료: {N}개, {el:.2f}s, 실제 {N / el:.2f} Hz, 한 주기 이상 늦은 발행 {late}" +
    (f", 정점 계산 평균 {np.mean(vt):.2f} ms p95 {np.percentile(vt, 95):.2f} ms" if vt else ""))
time.sleep(2.0)
open(a.end_file, "w").write(str(time.time_ns()))
with open(a.out + "_replay_sent.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["seq", "t_send_ns"]); w.writerows(sent)
json.dump(dict(npz=a.npz, n=N, rate=a.rate, actual_hz=N / el, late=late, betas=BETAS.tolist(), vertices=bool(a.vertices),
               vert_ms_mean=float(np.mean(vt)) if vt else None), open(a.out + "_replay_meta.json", "w"))
node.destroy_node(); rclpy.shutdown()
