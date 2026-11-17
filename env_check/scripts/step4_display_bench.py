"""step4_display_bench.py — 4-0/4-2 준비: 사람 분절 19개 표시 갱신 방식별 비용 (run_isaac.sh, headless, ROS 없음)

  입력(대체): ~/hrc_ws/debug_npz/human_dynamics_debug_*.npz 의 com_global_world (N,19,3) — human_dynamics 노드 출력과 같은 형식
             (읽기만 함). 실제 노드 출력은 4-1 에서 확인한다.
  임시 변환: node world → Isaac(로봇 기준) — yaw 회전 + 평행이동으로 사람 중심을 로봇 앞 (0.9, 0, ·) 에 두고 발을 z≈0.05 m 에 맞춤
  방식:
    usd_prim     : pxr 로 prim 19개 각각 xformOp:translate Set (미리 받아 둔 attribute 핸들 사용)
    isaac_usd    : isaacsim.core.prims.XFormPrim(usd=True).set_world_poses(positions) — 배치 API, USD 경로
    isaac_fabric : isaacsim.core.prims.XFormPrim(usd=False).set_world_poses(positions) — 배치 API, Fabric(USDRT) 경로
    instancer    : UsdGeom.PointInstancer 의 positions 속성 1회 Set (USD 단일 배열 쓰기)
  각 방식에서 프레임마다: 갱신 호출 시간, 이어지는 world.step(render=False) 시간, render 1회 시간(app.update)을 잰다.
  Fabric 경로가 렌더에 반영되는지: 갱신 전·후 뷰포트를 캡처해 픽셀 차이로 확인 (headless 렌더).
"""
import glob, json, os, sys, time
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 640, "height": 480})
import numpy as np
from pxr import Gf, UsdGeom, Sdf, Vt
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import XFormPrim
log = lambda *a: print("[disp]", *a, flush=True)

src = sorted(glob.glob(os.path.expanduser("~/hrc_ws/debug_npz/human_dynamics_debug_*.npz")))[1]
COM = np.load(src)["com_global_world"].astype(np.float64)        # (N,19,3) node world
N = COM.shape[0]
# 임시 변환: 첫 30프레임 기준. pelvis(0) xy 평균 → (0.9, 0), 정면 방향은 왼쪽 엉덩이(1)→오른쪽(2) 축의 수직으로 추정해 로봇(-x)을 보게
c0 = COM[:30].mean(0)
lr = c0[2] - c0[1]; lr[2] = 0; lr /= np.linalg.norm(lr)
fwd = np.cross(np.array([0, 0, 1.0]), lr)          # 왼→오 축에 수직 (SMPL 정면 근사)
yaw_now = np.arctan2(fwd[1], fwd[0]); yaw = np.pi - yaw_now
Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
P = COM @ Rz.T
shift = np.array([0.9, 0.0, 0.0]) - np.array([P[:30, 0, 0].mean(), P[:30, 0, 1].mean(), 0])
shift[2] = 0.05 - P[:, [6, 7], 2].min()             # 발(6,7) 최저점 → z 0.05
P = P + shift
log(f"입력 {os.path.basename(src)} N={N}; 임시 변환 yaw {np.degrees(yaw):.1f} deg, shift {np.round(shift, 3).tolist()}; "
    f"변환 후 pelvis 평균 {np.round(P[:, 0].mean(0), 3).tolist()}")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
RAD = [0.12, 0.08, 0.08, 0.11, 0.06, 0.06, 0.05, 0.05, 0.12, 0.06, 0.06, 0.06, 0.10, 0.05, 0.05, 0.045, 0.045, 0.045, 0.045]
paths = {}
for m in ["usd_prim", "isaac_usd", "isaac_fabric"]:
    paths[m] = []
    for k in range(19):
        p = f"/World/human_{m}/seg_{k:02d}"
        s = UsdGeom.Sphere.Define(stage, p); s.CreateRadiusAttr(RAD[k])
        UsdGeom.Xformable(s).AddTranslateOp().Set(Gf.Vec3d(*P[0, k]))
        paths[m].append(p)
inst = UsdGeom.PointInstancer.Define(stage, "/World/human_instancer")
proto = UsdGeom.Sphere.Define(stage, "/World/human_instancer/proto"); proto.CreateRadiusAttr(0.06)
inst.CreatePrototypesRel().SetTargets([proto.GetPath()])
inst.CreateProtoIndicesAttr(Vt.IntArray([0] * 19))
pos_attr = inst.CreatePositionsAttr(Vt.Vec3fArray([Gf.Vec3f(*x) for x in P[0]]))
world.reset()
for _ in range(5): app.update()

t_attr = [stage.GetPrimAtPath(p).GetAttribute("xformOp:translate") for p in paths["usd_prim"]]
xf_usd = XFormPrim("/World/human_isaac_usd/seg_.*", usd=True)
xf_fab = XFormPrim("/World/human_isaac_fabric/seg_.*", usd=False)
log(f"XFormPrim count usd {xf_usd.count}, fabric {xf_fab.count}")

def upd(m, X):
    if m == "usd_prim":
        for k in range(19): t_attr[k].Set(Gf.Vec3d(float(X[k, 0]), float(X[k, 1]), float(X[k, 2])))
    elif m == "isaac_usd":
        xf_usd.set_world_poses(positions=X)
    elif m == "isaac_fabric":
        xf_fab.set_world_poses(positions=X)
    elif m == "instancer":
        pos_attr.Set(Vt.Vec3fArray.FromNumpy(X.astype(np.float32)))

res = {}
NF = min(N, 300)
for m in ["usd_prim", "isaac_usd", "isaac_fabric", "instancer"]:
    up, st, rd = [], [], []
    for f in range(NF):
        X = P[f]
        t0 = time.perf_counter(); upd(m, X); t1 = time.perf_counter()
        world.step(render=False); t2 = time.perf_counter()
        if f % 10 == 0:
            app.update(); rd.append((time.perf_counter() - t2) * 1000)
        up.append((t1 - t0) * 1000); st.append((t2 - t1) * 1000)
    q = lambda v: dict(mean=round(float(np.mean(v)), 4), p95=round(float(np.percentile(v, 95)), 4), max=round(float(np.max(v)), 4))
    res[m] = dict(update_ms=q(up), next_physics_step_ms=q(st), render_ms_every10=q(rd))
    log(f"{m:13s} 갱신 {res[m]['update_ms']}  다음 물리 스텝 {res[m]['next_physics_step_ms']}  렌더(10프레임마다) {res[m]['render_ms_every10']}")

# 값 반영 확인: USD 에서 읽은 위치 (fabric 은 usdrt 에서 읽음)
X = P[NF - 1]
got_usd = np.array([list(t_attr[k].Get()) for k in range(19)])
gp, _ = xf_fab.get_world_poses()
log(f"마지막 프레임 반영 오차: usd_prim {np.abs(got_usd - X).max():.2e} m, isaac_fabric(get_world_poses) {np.abs(np.asarray(gp) - X).max():.2e} m")

# Fabric 경로가 렌더에 반영되는지: 다른 방식 prim 은 숨기고 fabric prim 만 보이게 한 뒤, 큰 이동 전후 캡처 비교
try:
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    from isaacsim.core.utils.viewports import set_camera_view
    for m in ["usd_prim", "isaac_usd"]:
        UsdGeom.Imageable(stage.GetPrimAtPath(f"/World/human_{m}")).MakeInvisible()
    UsdGeom.Imageable(inst).MakeInvisible()
    set_camera_view(eye=[3.0, 0.0, 1.2], target=[0.9, 0.0, 0.8])
    outd = os.path.expanduser("~/effmass_mpc/env_check/step4"); os.makedirs(outd, exist_ok=True)
    shots = []
    for tag, dx in [("a", 0.0), ("b", 0.6)]:
        xf_fab.set_world_poses(positions=P[0] + np.array([0, dx, 0]))
        for _ in range(20): app.update()
        fn = f"{outd}/fabric_render_check_{tag}.png"; capture_viewport_to_file(get_active_viewport(), fn)
        for _ in range(10): app.update()
        shots.append(fn)
    from PIL import Image
    a = np.asarray(Image.open(shots[0]).convert("L"), np.float32); b = np.asarray(Image.open(shots[1]).convert("L"), np.float32)
    diff = np.abs(a - b)
    res["fabric_render_check"] = dict(files=shots, mean_abs_diff=float(diff.mean()), changed_pixels=int((diff > 10).sum()))
    log(f"Fabric 렌더 반영 확인: 캡처 {shots}, 평균 차 {diff.mean():.3f}, 10 넘게 바뀐 픽셀 {int((diff > 10).sum())}")
except Exception as e:
    log(f"렌더 확인 실패: {type(e).__name__}: {e}")
json.dump(res, open(os.path.expanduser("~/effmass_mpc/env_check/step4/display_bench.json"), "w"), indent=1)
log("DONE")
app.close()
