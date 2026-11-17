"""g1_robot_in_isaac.py — 1-2c: Isaac Python 에서 effmass_robot(Pinocchio) import + 경로 불변 확인 + cuRobo 구 중심과 비교
실행: ROS_METHOD=min RUN_ISAAC_NO_DISPLAY=1 run_isaac_ros.sh <이 파일> [isaac_first|paths_first]
  isaac_first : numpy·torch·warp·curobo 를 먼저 import → 경로 추가 → pinocchio·effmass_robot import → 같은 모듈 객체·파일인지
  paths_first : 경로를 먼저 추가 → numpy·torch·warp·curobo import → 파일 위치가 Isaac 쪽인지 (가려지지 않는지)
  추가 경로 = 3단계 min 방식 STEP3_ROS_PYPATHS (sys.path 맨 뒤) + ~/effmass_mpc/ws/install/effmass_robot/.../site-packages (맨 뒤)
"""
import json, os, sys, time
ORDER = sys.argv[1] if len(sys.argv) > 1 else "isaac_first"
HOME = os.path.expanduser("~"); sys.path.insert(0, f"{HOME}/effmass_mpc/env_check/scripts")
import step3_ros_common as RC
EFF = f"{HOME}/effmass_mpc/ws/install/effmass_robot/lib/python3.10/site-packages"
MODS = ["numpy", "torch", "warp", "curobo", "yaml"]
def add_paths():
    a = RC.add_ros_paths()
    if EFF not in sys.path: sys.path.append(EFF); a.append(EFF)
    return a
def snap():
    out = {}
    for m in MODS:
        mod = __import__(m); out[m] = dict(file=getattr(mod, "__file__", None), version=getattr(mod, "__version__", None), id=id(mod))
    return out
res = dict(order=ORDER, python=sys.executable)
if ORDER == "isaac_first":
    before = snap(); res["added"] = add_paths()
    import pinocchio as pin
    from effmass_robot.robot_inertia import RobotInertia
    after = snap(); res["unchanged"] = all(before[m] == after[m] for m in MODS)
else:
    res["added"] = add_paths(); after = snap()
    import pinocchio as pin
    from effmass_robot.robot_inertia import RobotInertia
res["modules"] = {m: {k: v for k, v in d.items() if k != "id"} for m, d in after.items()}
res["all_isaac"] = all((d["file"] or "").startswith(os.environ.get("ISAAC_ROOT", "")) or ".local/share/ov" in (d["file"] or "") or "isaacsim" in (d["file"] or "") for m, d in after.items())
res["pinocchio"] = dict(version=pin.__version__, file=pin.__file__)
import effmass_robot.robot_inertia as RI; res["effmass_robot_file"] = RI.__file__
import numpy as np, torch
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.util_file import load_yaml
ta = TensorDeviceType()
cfg = RobotConfig.from_dict(load_yaml(f"{HOME}/effmass_mpc/goal1/fr3v2_1/fr3v2_1.yml")["robot_cfg"], ta)
km = CudaRobotModel(cfg.kinematics)
ri = RobotInertia(rotor=True)
rng = np.random.default_rng(3)
lo = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]); hi = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
Q = np.stack([np.array([0, -1.3, 0, -2.5, 0, 1.0, 0])] + [lo + (hi - lo) * rng.uniform(0.05, 0.95, 7) for _ in range(30)])
res["curobo_dof"] = int(km.get_dof()); res["curobo_joint_names"] = list(km.joint_names)
qc = torch.tensor(Q, dtype=torch.float32, device=ta.device)
if km.get_dof() == 8: qc = torch.cat([qc, torch.full((len(Q), 1), 0.04, device=ta.device)], 1)
sph = km.get_state(qc).link_spheres_tensor.cpu().numpy()          # (B, n, 4)
valid = sph[0, :, 3] > 0
res["curobo_spheres_total"] = int(sph.shape[1]); res["curobo_spheres_r_pos"] = int(valid.sum())
d = []; dr = 0.0
for b in range(len(Q)):
    C = ri.compute(Q[b])[0]; d.append(np.abs(sph[b, valid, :3] - C).max())
dr = float(np.abs(sph[0, valid, 3] - (ri.sphere_radius + cfg.kinematics.kinematics_config.link_spheres[valid, 3].cpu().numpy() * 0 + 0)).max())
res["center_vs_curobo_max_m"] = float(max(d)); res["radius_vs_yml_max_m"] = dr
json.dump(res, open(f"{HOME}/effmass_mpc/goal1/logs/g1_robot_in_isaac_{ORDER}.json", "w"), indent=1)
print("[robot-isaac]", json.dumps(res, ensure_ascii=False), flush=True)
