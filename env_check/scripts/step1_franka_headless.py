"""step1_franka_headless.py — Isaac Sim 4.5 단독 실행 시험 (run_isaac.sh 로 실행)

  사용법: run_isaac.sh step1_franka_headless.py [--gui] [--steps 600] [--hold SEC] [--screenshot PNG]
    --hold SEC       : GUI 모드에서 측정 후 창 유지 시간(기본 15초, 0 이면 창을 닫을 때까지)
    --screenshot PNG : GUI 모드에서 측정 직후 뷰포트를 PNG 로 저장(참고용)

  빈 장면 + 기본 바닥 + Franka(franka_alt_fingers.usd, standalone_examples 와 동일 경로/방식)
  물리 dt = 1/60, 지정 스텝 수만큼 진행하면서 스텝당 실제 시간을 측정한다.
  출력: 시작 시각, 기동 시간(SimulationApp/장면 로딩), 스텝 시간 평균/p95/최대,
        실시간 배율 = (스텝 수 × 1/60) ÷ 스텝 루프 실제 시간
"""
import sys, time, datetime, statistics

T_START = time.perf_counter()
START_STAMP = datetime.datetime.now().isoformat(timespec="seconds")
GUI = "--gui" in sys.argv
STEPS = int(sys.argv[sys.argv.index("--steps") + 1]) if "--steps" in sys.argv else 600
HOLD = float(sys.argv[sys.argv.index("--hold") + 1]) if "--hold" in sys.argv else 15.0
SHOT = sys.argv[sys.argv.index("--screenshot") + 1] if "--screenshot" in sys.argv else None
PHYS_DT = 1.0 / 60.0

print(f"[step1] start {START_STAMP}  mode={'GUI' if GUI else 'headless'}  steps={STEPS}  physics_dt={PHYS_DT:.5f}", flush=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": not GUI})
t_app = time.perf_counter()
print(f"[step1] SimulationApp ready: {t_app - T_START:.1f} s", flush=True)

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
print(f"[step1] assets_root_path = {assets_root_path}", flush=True)
if assets_root_path is None:
    print("[step1] ERROR: get_assets_root_path() 가 None — 클라우드/로컬 에셋 루트를 찾지 못함. 중단.", flush=True)
    simulation_app.close(); sys.exit(3)

asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
print(f"[step1] Franka USD = {asset_path}", flush=True)

world = World(stage_units_in_meters=1.0, physics_dt=PHYS_DT, rendering_dt=PHYS_DT)
world.scene.add_default_ground_plane()
t0 = time.perf_counter()
prim = add_reference_to_stage(asset_path, "/Franka")
simulation_app.update()
while is_stage_loading():
    simulation_app.update()
t_load = time.perf_counter()
print(f"[step1] Franka 로딩 {t_load - t0:.1f} s  prim={prim.GetPath() if prim else None} valid={bool(prim and prim.IsValid())}", flush=True)
if not (prim and prim.IsValid()):
    print("[step1] ERROR: Franka prim 이 유효하지 않음 (에셋 읽기 실패). 중단.", flush=True)
    simulation_app.close(); sys.exit(4)

world.reset()
print(f"[step1] world.reset 완료, 기동 총 {time.perf_counter() - T_START:.1f} s (장면 준비 완료 시점)", flush=True)

# 워밍업 (측정에서 제외) — GUI 는 셰이더 컴파일 포함
t_w0 = time.perf_counter()
for _ in range(30):
    world.step(render=GUI)
print(f"[step1] warmup 30 steps: {time.perf_counter() - t_w0:.1f}s (측정 제외)", flush=True)

dts = []
loop_t0 = time.perf_counter()
for i in range(STEPS):
    s = time.perf_counter()
    world.step(render=GUI)
    dts.append(time.perf_counter() - s)
    if (i + 1) % 100 == 0:
        print(f"[step1] step {i+1}/{STEPS}  sim_t={world.current_time:.3f}s", flush=True)
loop_wall = time.perf_counter() - loop_t0

dts_ms = sorted(d * 1000 for d in dts)
p95 = dts_ms[int(0.95 * len(dts_ms)) - 1]
sim_time = STEPS * PHYS_DT
rtf = sim_time / loop_wall
print("\n[step1] ===== RESULT =====")
print(f"[step1] mode               : {'GUI' if GUI else 'headless'}")
print(f"[step1] start              : {START_STAMP}")
print(f"[step1] startup_to_scene_s : {t_load - T_START:.1f}")
print(f"[step1] steps              : {STEPS}  physics_dt={PHYS_DT:.5f}")
print(f"[step1] step_ms mean/p95/max: {statistics.mean(dts_ms):.3f} / {p95:.3f} / {dts_ms[-1]:.3f}")
print(f"[step1] sim_time_s         : {sim_time:.3f}")
print(f"[step1] wall_time_s        : {loop_wall:.3f}")
print(f"[step1] realtime_factor    : {rtf:.2f}")
print(f"[step1] world.current_time : {world.current_time:.3f}")
print(f"[step1] total_wall_s       : {time.perf_counter() - T_START:.1f}", flush=True)
if GUI and SHOT:
    try:
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[1.8, 1.8, 1.3], target=[0.0, 0.0, 0.4])
        for _ in range(30):
            world.step(render=True)
        capture_viewport_to_file(get_active_viewport(), SHOT)
        for _ in range(10):
            world.step(render=True)   # 캡처가 디스크에 쓰일 시간
        print(f"[step1] screenshot      : {SHOT}", flush=True)
    except Exception as e:
        print(f"[step1] screenshot 실패: {type(e).__name__}: {e}", flush=True)
if GUI:
    print(f"[step1] GUI 모드: 창 유지 {'창을 닫을 때까지' if HOLD == 0 else f'{HOLD:.0f}초'} (화면 확인용)", flush=True)
    t_hold = time.perf_counter()
    while simulation_app.is_running() and (HOLD == 0 or time.perf_counter() - t_hold < HOLD):
        world.step(render=True)
world.stop()
simulation_app.close()
print("[step1] DONE", flush=True)
