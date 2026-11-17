"""g1_fr3v21_open_stage.py — fr3v2_1.usda 를 Isaac 4.5 에서 루트 스테이지로 직접 열기 (참조 대신). 관절 생성 여부만 확인 (측정 없음)"""
import os
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.usd, numpy as np
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from pxr import UsdPhysics
P = os.path.expanduser("~/effmass_mpc/ref/franka_simulation_release_v0.1.0/fr3v2_1/fr3v2_1.usda")
print("[open] open_stage", open_stage(P), flush=True)
st = omni.usd.get_context().get_stage()
art = [p.GetPath().pathString for p in st.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print("[open] 관절 루트", art, flush=True)
w = World(stage_units_in_meters=1.0)
try:
    r = w.scene.add(Robot(prim_path=art[0], name="v21")); w.reset()
    print("[open] dof_names", r.dof_names, "num_dof", r.num_dof, flush=True)
    for _ in range(30): w.step(render=False)
    print("[open] 30 스텝 뒤 관절값", np.round(r.get_joint_positions(), 4).tolist(), flush=True)
except Exception as e:
    print("[open] 실패", repr(e)[:300], flush=True)
app.close()
