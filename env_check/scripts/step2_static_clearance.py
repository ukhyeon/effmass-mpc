"""step2_static_clearance.py <steps.csv> [시작 끝] — 기록된 관절값으로 로봇 구와 각 cuboid 사이 최소 여유를 numpy 로 계산 (run_isaac.sh)"""
import csv, os, sys
import numpy as np, torch
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step2_scene as S
from curobo.types.state import JointState
rows = list(csv.DictReader(open(sys.argv[1])))
a, b = (int(sys.argv[2]), int(sys.argv[3])) if len(sys.argv) > 3 else (0, len(rows) - 1)
scene = sys.argv[4] if len(sys.argv) > 4 else "v1"
S.set_scene(scene)
wc = S.build_world_config()
mpc = S.build_mpc(S.build_robot_cfg(), wc)
Q = np.array([[float(rows[k][f"q{i}"]) for i in range(7)] for k in range(a, b + 1)], dtype=np.float32)
kin = mpc.rollout_fn.compute_kinematics(JointState.from_position(torch.as_tensor(Q, device="cuda"), joint_names=mpc.joint_names))
SPH = kin.robot_spheres.reshape(len(Q), -1, 4).cpu().numpy()
def clr(sph, c, d):
    s = sph[sph[:, 3] > 0]; return float((S.box_signed_distance(s[:, :3], c, d) - s[:, 3]).min())
print(f"[static] steps {a}-{b} ({len(Q)}), scene {scene}")
for cub in wc.cuboid:
    if cub.name == S.MOVER_NAME: continue
    v = [clr(SPH[i], cub.pose[:3], cub.dims) for i in range(len(Q))]
    neg = [a + i for i, x in enumerate(v) if x < 0]
    print(f"[static] {cub.name:10s} 최소 여유 {min(v):+.4f} m (step {a + int(np.argmin(v))}), 여유<0 스텝 {len(neg)}" + (f" ({neg[0]}~{neg[-1]})" if neg else ""))
for k in range(a, b + 1, 25):
    i = k - a
    print(f"[static]   step {k}: block_low {clr(SPH[i], wc.cuboid[2].pose[:3], wc.cuboid[2].dims):+.4f}  "
          f"block_high {clr(SPH[i], wc.cuboid[3].pose[:3], wc.cuboid[3].dims):+.4f}  table {clr(SPH[i], wc.cuboid[0].pose[:3], wc.cuboid[0].dims):+.4f}  feasible {rows[k]['feasible']}")
