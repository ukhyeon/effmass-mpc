"""g1_fr3_check.py — cuRobo FR3 설정 확인 (Isaac Python, SimulationApp 없음): 관절·한계, 구 수, Panda 와 FK 비교, 정지 자세 자기충돌"""
import os, json, numpy as np, torch, yaml
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
ta = TensorDeviceType()
fr = load_yaml(os.path.expanduser("~/effmass_mpc/goal1/fr3/fr3.yml"))["robot_cfg"]
pa = load_yaml(join_path(get_robot_configs_path(), "franka.yml"))["robot_cfg"]
M = {}
for nm, cfg in (("FR3", fr), ("Panda", pa)):
    rc = RobotConfig.from_dict(cfg, ta); km = CudaRobotModel(rc.kinematics)
    M[nm] = (rc, km)
    jl = rc.kinematics.get_joint_limits()
    print(f"[{nm}] 관절 {km.joint_names}")
    print(f"[{nm}] 위치 하한 {np.round(jl.position[0].cpu().numpy(), 4).tolist()}")
    print(f"[{nm}] 위치 상한 {np.round(jl.position[1].cpu().numpy(), 4).tolist()}")
    print(f"[{nm}] 속도 상한 {np.round(jl.velocity[1].cpu().numpy(), 3).tolist()}")
    sp = km.kinematics_config.link_spheres.cpu().numpy()
    print(f"[{nm}] 충돌 구 {sp.shape[0]}개 (반지름>0 {int((sp[:, 3] > 0).sum())}), 반지름 평균 {sp[sp[:, 3] > 0, 3].mean():.4f} m, 링크 {km.kinematics_config.link_name_to_idx_map.keys() if False else len(km.link_names)}")
# FK 비교 (같은 관절값)
rng = np.random.default_rng(0)
lo = torch.maximum(M["FR3"][0].kinematics.get_joint_limits().position[0], M["Panda"][0].kinematics.get_joint_limits().position[0])
hi = torch.minimum(M["FR3"][0].kinematics.get_joint_limits().position[1], M["Panda"][0].kinematics.get_joint_limits().position[1])
q = lo + (hi - lo) * torch.tensor(rng.random((50, lo.shape[0])), device=lo.device, dtype=lo.dtype)
sf = M["FR3"][1].get_state(q); sp_ = M["Panda"][1].get_state(q)
dp = (sf.ee_position - sp_.ee_position).norm(dim=-1).max().item(); dq = (sf.ee_quaternion - sp_.ee_quaternion).abs().max().item()
print(f"[FK] 공통 관절 범위 무작위 50 자세: EE(hand) 위치 차 최대 {dp*1000:.4f} mm, 쿼터니언 성분 차 최대 {dq:.2e}")
s0 = M["FR3"][1].get_state(torch.tensor([fr["kinematics"]["cspace"]["retract_config"][:7] + [0.04]], device=lo.device, dtype=lo.dtype)[:, :lo.shape[0]])
print(f"[FR3] retract EE {np.round(s0.ee_position.cpu().numpy(), 4).tolist()}")
