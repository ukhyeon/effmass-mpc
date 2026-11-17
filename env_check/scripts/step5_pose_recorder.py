"""step5_pose_recorder.py --out <prefix> --stop-file F — /hrc/human_pose 수신 기록 (시스템 Python)
  매 메시지: 수신 시각, header.stamp, 정점 수, body_full_pose(66), global_trans(3), smpl_betas(10), smpl_joint 첫 관절(3, 카메라 좌표 골반)
  정점 전체는 저장하지 않음 (메시지당 82.7 KB). 처음 1개 메시지의 정점만 따로 저장
"""
import argparse, os, sys, threading, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step3_msgs as M
import rclpy
from rclpy.executors import SingleThreadedExecutor
from hrc_interfaces.msg import HumanPose
ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--stop-file", required=True); a = ap.parse_args()
rclpy.init(); node = rclpy.create_node("step5_pose_recorder")
R = dict(t_recv=[], stamp=[], nv=[], pose=[], trans=[], betas=[], joint0=[]); first_v = []
def cb(m):
    t = time.time_ns()
    R["t_recv"].append(t); R["stamp"].append(M.stamp_ns(m.header)); R["nv"].append(len(m.smpl_vertices))
    R["pose"].append(np.asarray(m.body_full_pose, np.float32)); R["trans"].append(np.asarray(m.global_trans, np.float32))
    b = np.zeros(10, np.float32); b[:min(10, len(m.smpl_betas))] = np.asarray(m.smpl_betas, np.float32)[:10]; R["betas"].append(b)
    j = np.full(3, np.nan, np.float32)
    if len(m.smpl_joint) >= 3: j = np.asarray(m.smpl_joint[:3], np.float32)
    R["joint0"].append(j)
    if not first_v and len(m.smpl_vertices) == 20670: first_v.append(np.asarray(m.smpl_vertices, np.float32).reshape(6890, 3))
node.create_subscription(HumanPose, "/hrc/human_pose", cb, M.best_effort_qos())
ex = SingleThreadedExecutor(); ex.add_node(node); th = threading.Thread(target=ex.spin, daemon=True); th.start()
while not os.path.exists(a.stop_file): time.sleep(0.2)
ex.shutdown(); node.destroy_node(); rclpy.shutdown()
np.savez(a.out + "_pose_rec.npz", **{k: np.asarray(v) for k, v in R.items()}, first_vertices=first_v[0] if first_v else np.zeros((0, 3)))
print(f"[recorder] HumanPose 수신 {len(R['t_recv'])}, 정점 있는 메시지 {sum(1 for x in R['nv'] if x == 20670)}", flush=True)
