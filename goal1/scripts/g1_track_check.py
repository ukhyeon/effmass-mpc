"""g1_track_check.py — Isaac 위치 명령 추종 확인 (진단, 설정 변경 없음). STEP_ROBOT=panda|fr3, step2b 와 같은 로봇 생성·max effort 5000·60 Hz 물리
  관절 1·2·4 에 1 rad/s 경사 명령(1 s) 후 정지 → 명령−측정 차(지연)와 정착 시간 기록"""
import os, sys, json
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts")); sys.path.insert(0, os.path.expanduser("~/effmass_mpc/curobo/examples/isaac_sim"))
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
import step2_scene as S
world = World(stage_units_in_meters=1.0); world.set_simulation_dt(physics_dt=S.PHYSICS_DT, rendering_dt=S.PHYSICS_DT)
st = world.stage; x = st.DefinePrim("/World", "Xform"); st.SetDefaultPrim(x); world.scene.add_default_ground_plane()
cfg = S.build_robot_cfg(); names = cfg["kinematics"]["cspace"]["joint_names"]; q0 = np.array(cfg["kinematics"]["cspace"]["retract_config"])
robot, path = S.add_isaac_robot(cfg, world)
world.reset(); world.play(); S.apply_arm_gains_runtime(robot)
idx = [robot.get_dof_index(n) for n in names]; robot.set_joint_positions(q0, idx)
robot._articulation_view.set_max_efforts(values=np.full(len(idx), 5000.0), joint_indices=idx)
ctl = robot.get_articulation_controller()
for _ in range(30): world.step(render=False)
arm = idx[:7]; q = q0[:7].copy(); rec = []
for k in range(180):
    t = k * S.PHYSICS_DT
    tgt = q.copy()
    if t < 1.0:
        tgt[[0, 1, 3]] = q0[[0, 1, 3]] + np.array([1.0, 0.5, 0.5]) * t
    else:
        tgt[[0, 1, 3]] = q0[[0, 1, 3]] + np.array([1.0, 0.5, 0.5]) * 1.0
    ctl.apply_action(ArticulationAction(tgt, joint_indices=arm)); world.step(render=False)
    m = robot.get_joint_positions()[arm]; rec.append((t, tgt[0] - m[0], tgt[1] - m[1], tgt[3] - m[3]))
r = np.array(rec)
ramp = r[(r[:, 0] > 0.5) & (r[:, 0] < 1.0)]; after = r[r[:, 0] >= 1.0]
settle = next((float(t - 1.0) for t, a, b, c in after if max(abs(a), abs(b), abs(c)) < 0.005), None)
out = dict(robot=S.robot_name(), gains=dict(S.FR3_GAINS_APPLIED), ramp_lag_rad_mean=np.abs(ramp[:, 1:]).mean(0).round(4).tolist(), ramp_lag_s=(np.abs(ramp[:, 1:]).mean(0) / np.array([1.0, 0.5, 0.5])).round(3).tolist(),
           settle_to_5mrad_s=settle, final_err_rad=np.abs(r[-1, 1:]).round(5).tolist(), start_err=np.abs(robot.get_joint_positions()[arm] - tgt).max().round(5).tolist())
print("[track]", json.dumps(out), flush=True)
app.close()
