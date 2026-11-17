"""step2_design_mover.py <v1 steps.csv> — 움직이는 장애물 경로 후보를 numpy 로 평가 (run_isaac.sh 로 실행, SimulationApp 없음)

  - v1 측정의 스텝별 관절값(q0..q6)으로 cuRobo kinematics 의 로봇 충돌 구(반지름에 sphere buffer 0.02 포함)를 계산
  - 정지 자세: 목표 도달 스텝의 자세 (goal 0, goal 1 각각)
  - 이동 경로: feasible 이면서 목표 도달 스텝이 아닌 스텝들의 자세 (v1 장애물의 영향을 받은 경로이므로 근사)
  - 후보마다 장애물의 한 주기 위치(120점)와 위 자세들의 구 사이 최소 여유 거리(축 정렬 박스–구)를 계산
"""
import csv
import json
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step2_scene as S
from curobo.types.state import JointState

rows = list(csv.DictReader(open(sys.argv[1])))
Q = np.array([[float(r[f"q{i}"]) for i in range(7)] for r in rows], dtype=np.float32)
feas = np.array([int(r["feasible"]) for r in rows])
switched = np.array([int(r["goal_switched"]) for r in rows])
goal = np.array([int(r["goal_idx"]) for r in rows])

S.set_scene("v1")
mpc = S.build_mpc(S.build_robot_cfg(), S.build_world_config())
js = JointState.from_position(torch.as_tensor(Q, device="cuda"), joint_names=mpc.joint_names)
kin = mpc.rollout_fn.compute_kinematics(js)
SPH = kin.robot_spheres.reshape(len(rows), -1, 4).cpu().numpy()  # (N, n_sph, 4)
print(f"[design] 로봇 구 {SPH.shape[1]}개, 자세 {SPH.shape[0]}개 (반지름에 collision_sphere_buffer {S.COLLISION_SPHERE_BUFFER} 포함)")

rest_idx = {g: [int(i) for i in np.where((switched == 1) & (goal == g))[0]] for g in (0, 1)}
path_idx = np.where((feas == 1) & (switched == 0))[0]
print(f"[design] 정지 자세(목표 도달 스텝): goal0 {rest_idx[0]}, goal1 {rest_idx[1]}; 경로 자세 {len(path_idx)}개")


def clearance(sph, center, dims):
    s = sph[sph[:, 3] > 0]
    return float((S.box_signed_distance(s[:, :3], center, dims) - s[:, 3]).min())


def evaluate(center, amp, period, dims, axis=0, n=120):
    cs = []
    for k in range(n):
        c = list(center)
        c[axis] = center[axis] + amp * np.sin(2 * np.pi * k / n)
        cs.append(c)
    rest = {g: min(clearance(SPH[i], c, dims) for i in rest_idx[g] for c in cs) for g in (0, 1)}
    path = min(clearance(SPH[i], c, dims) for i in path_idx[::3] for c in cs[::4])
    return rest, path


cands = {
    "v1 (현재)": dict(center=[0.65, 0.0, 0.45], amp=0.15),
    "A x 0.70±0.10": dict(center=[0.70, 0.0, 0.45], amp=0.10),
    "B x 0.72±0.10": dict(center=[0.72, 0.0, 0.45], amp=0.10),
    "C x 0.75±0.10": dict(center=[0.75, 0.0, 0.45], amp=0.10),
    "D x 0.70±0.12": dict(center=[0.70, 0.0, 0.45], amp=0.12),
    "E x 0.68±0.10": dict(center=[0.68, 0.0, 0.45], amp=0.10),
}
out = {}
for name, c in cands.items():
    rest, path = evaluate(c["center"], c["amp"], 6.0, S.MOVER_DIMS)
    xmin = c["center"][0] - c["amp"]
    out[name] = dict(center=c["center"], amp=c["amp"], x_range=[round(xmin, 3), round(c["center"][0] + c["amp"], 3)],
                     rest_goal0=round(rest[0], 4), rest_goal1=round(rest[1], 4), path_min=round(path, 4))
    print(f"[design] {name:16s} x {xmin:.2f}~{c['center'][0]+c['amp']:.2f}  정지자세 최소여유 goal0 {rest[0]:+.4f} m, "
          f"goal1 {rest[1]:+.4f} m  |  v1 경로 자세와 최소여유 {path:+.4f} m")
print("[design] JSON " + json.dumps(out, ensure_ascii=False))
