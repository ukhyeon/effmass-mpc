"""g1_goal_search.py — g1 장면 목표 자세 두 개 정하기 (Isaac Python, curobo, SimulationApp 없음)
  사람: 4단계 재생(human_state_1781524848.npz, 4단계 임시 변환) 분절 COM (step4/replay_com_isaac.npy, 1,262 프레임) + 분절 반지름 = 4단계 표시 구 반지름 RAD
  후보: x 0.30~0.80, y −0.60~0.60, z −0.30~0.70 (0.05 m 격자), 손(fr3v2_1_hand) 방향 wxyz (0,1,0,0) = 2단계 목표와 같음 (손이 아래를 봄)
  조건: (1) 전 프레임·전 분절 표면까지 최소 거리 ≥ 0.15 m, (2) cuRobo IK 성공 (fr3v2_1 좁은 한계, 자기충돌·탁자·바닥 충돌 없음)
  선택: y < −0.15 쪽과 y > +0.15 쪽에서 각각 '사람 표면 최소 거리'가 가장 작은 (= 사람에 가장 가까운) 점. 같으면 x 큰 것
  출력: ~/effmass_mpc/goal1/scene_g1.json, goal1/logs/g1_goal_search.json"""
import os, sys, json, numpy as np, torch
os.environ["STEP_ROBOT"] = "fr3v2_1"
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts"))
import step2_scene as S
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
S.set_scene("g1")
ta = TensorDeviceType()
X = np.load(os.path.expanduser("~/effmass_mpc/env_check/step4/replay_com_isaac.npy"))
RAD = np.array([0.12, 0.08, 0.08, 0.11, 0.06, 0.06, 0.05, 0.05, 0.12, 0.06, 0.06, 0.06, 0.10, 0.05, 0.05, 0.045, 0.045, 0.045, 0.045])
g = np.stack(np.meshgrid(np.arange(0.30, 0.801, 0.05), np.arange(-0.60, 0.601, 0.05), np.arange(-0.30, 0.701, 0.05), indexing="ij"), -1).reshape(-1, 3)
d = np.full(len(g), np.inf)
for f in range(len(X)):
    d = np.minimum(d, (np.linalg.norm(g[:, None] - X[f][None], axis=2) - RAD[None]).min(1))
ok_h = d >= 0.15
print(f"[goal] 후보 {len(g)}, 사람 표면 ≥ 0.15 m {int(ok_h.sum())}", flush=True)
cfg = S.build_robot_cfg()
rc = RobotConfig.from_dict(cfg, ta)
ik = IKSolver(IKSolverConfig.load_from_robot_config(rc, S.build_world_config(), num_seeds=32, self_collision_check=True, self_collision_opt=True,
                                                    tensor_args=ta, use_cuda_graph=False, position_threshold=0.005, rotation_threshold=0.05))
cand = g[ok_h]
P = Pose(position=torch.tensor(cand, **ta.as_torch_dict()).float(), quaternion=torch.tensor([[0.0, 1.0, 0.0, 0.0]] * len(cand), **ta.as_torch_dict()).float())
r = ik.solve_batch(P)
succ = r.success.view(-1).cpu().numpy(); q = r.solution.view(len(cand), -1).cpu().numpy()
print(f"[goal] IK 성공 {int(succ.sum())}/{len(cand)}", flush=True)
dc = d[ok_h]
out = {}
for side, sel in (("right(y<0)", cand[:, 1] < -0.15), ("left(y>0)", cand[:, 1] > 0.15)):
    m = sel & succ
    idx = np.nonzero(m)[0]
    best = idx[np.lexsort((-cand[idx, 0], dc[idx]))][0]
    out[side] = dict(pos=cand[best].round(3).tolist(), human_surface_min_m=round(float(dc[best]), 4), q=q[best].round(4).tolist(),
                     n_feasible_side=int(m.sum()))
    print(f"[goal] {side}: {out[side]}", flush=True)
# 가까운 분절 정보
names = ["pelvis", "L_thigh", "R_thigh", "abdomen", "L_calf", "R_calf", "L_foot", "R_foot", "chest", "neck", "L_shoulder", "R_shoulder", "head", "L_upperarm", "R_upperarm", "L_lowerarm", "R_lowerarm", "L_hand", "R_hand"]
for k, v in out.items():
    p = np.array(v["pos"]); dd = (np.linalg.norm(X - p, axis=2) - RAD).min(0); b = int(dd.argmin()); v["closest_segment"] = names[b]
    v["retract_ee_dist_m"] = None
goals = [out["right(y<0)"]["pos"] + [0.0, 1.0, 0.0, 0.0], out["left(y>0)"]["pos"] + [0.0, 1.0, 0.0, 0.0]]
json.dump(dict(goals_wxyz=goals, rule="손 방향 (0,1,0,0); 사람(4단계 재생·임시 변환) 전 프레임 분절 표면까지 ≥ 0.15 m; cuRobo IK(fr3v2_1 좁은 한계, 탁자·바닥) 성공; 좌우 각각 사람에 가장 가까운 점",
               detail=out), open(os.path.expanduser("~/effmass_mpc/goal1/scene_g1.json"), "w"), indent=1)
# 표: 쪽별 사람 거리 분포
json.dump(dict(grid=len(g), human_ok=int(ok_h.sum()), ik_ok=int(succ.sum()), detail=out,
               feasible_points=[dict(p=cand[i].round(3).tolist(), dmin=round(float(dc[i]), 3)) for i in np.nonzero(succ)[0]]),
          open(os.path.expanduser("~/effmass_mpc/goal1/logs/g1_goal_search.json"), "w"), indent=1)
print("[goal] 목표", goals, flush=True)
