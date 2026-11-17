"""step3_posthoc_clearance.py <steps.csv> [scene] — 기록된 관절값으로 실행 후 여유 거리 계산 (run_isaac.sh, SimulationApp 없음)
  로봇 구(반지름에 sphere buffer 포함) ↔ 그 스텝의 장애물(mover_x/y/z) 및 정적 cuboid 각각의 최소 여유
"""
import csv, os, sys
import numpy as np, torch
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step2_scene as S
from curobo.types.state import JointState
rows = list(csv.DictReader(open(sys.argv[1])))
S.set_scene(sys.argv[2] if len(sys.argv) > 2 else "v2")
wc = S.build_world_config()
mpc = S.build_mpc(S.build_robot_cfg(), wc)
Q = np.array([[float(r[f"q{i}"]) for i in range(7)] for r in rows], dtype=np.float32)
kin = mpc.rollout_fn.compute_kinematics(JointState.from_position(torch.as_tensor(Q, device="cuda"), joint_names=mpc.joint_names))
SPH = kin.robot_spheres.reshape(len(Q), -1, 4).cpu().numpy()
def clr(sph, c, d):
    s = sph[sph[:, 3] > 0]; return float((S.box_signed_distance(s[:, :3], c, d) - s[:, 3]).min())
mv = [clr(SPH[i], [float(rows[i]["mover_x"]), float(rows[i]["mover_y"]), float(rows[i]["mover_z"])], S.MOVER_DIMS) for i in range(len(rows))]
k = int(np.argmin(mv))
print(f"[posthoc] {os.path.basename(sys.argv[1])}: 스텝 {len(rows)}, 장면 {S.SCENE}")
print(f"[posthoc] mover      최소 여유 {mv[k]:+.4f} m (step {k}), 여유<0 스텝 {sum(1 for x in mv if x < 0)}")
for cub in wc.cuboid:
    if cub.name == S.MOVER_NAME: continue
    v = [clr(SPH[i], cub.pose[:3], cub.dims) for i in range(len(rows))]
    print(f"[posthoc] {cub.name:10s} 최소 여유 {min(v):+.4f} m (step {int(np.argmin(v))}), 여유<0 스텝 {sum(1 for x in v if x < 0)}")
