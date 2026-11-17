"""step3_ros_import_probe.py — ROS 2 경로 방식별 sys.path 순서와 import 경로 비교 (run_isaac_ros.sh 로 실행)

  사용법: ROS_METHOD=min|source run_isaac_ros.sh step3_ros_import_probe.py [--with-kit] [--bridge]
    --with-kit : SimulationApp(headless) 를 먼저 띄움
    --bridge   : (--with-kit 일 때) Isaac 예제처럼 enable_extension("isaacsim.ros2.bridge") 후 진행
"""
import json
import os
import sys
import time

WITH_KIT = "--with-kit" in sys.argv
BRIDGE = "--bridge" in sys.argv
METHOD = os.environ.get("STEP3_ROS_METHOD", "?")
print(f"[probe] method={METHOD} kit={WITH_KIT} bridge={BRIDGE}", flush=True)
if WITH_KIT:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    if BRIDGE:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.ros2.bridge")
        app.update()
        app.update()

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step3_ros_common as C

added = C.add_ros_paths() if METHOD == "min" else []
print(f"[probe] add_ros_paths 로 sys.path 뒤에 붙인 경로 {len(added)}개", flush=True)

# sys.path 순서 요약
labels = [C.label(p) for p in sys.path if p]
first_ros = next((i for i, l in enumerate(labels) if l in ("ros", "hrc_ws")), None)
last_isaac = max((i for i, l in enumerate(labels) if l == "isaac"), default=None)
print(f"[probe] sys.path 항목 {len(labels)}개: " + " ".join(f"{i}:{l}" for i, l in enumerate(labels)), flush=True)
print(f"[probe] 첫 ROS/hrc_ws 위치 {first_ros}, 마지막 Isaac 위치 {last_isaac} → "
      f"{'ROS 경로가 Isaac 경로보다 앞에 있음' if first_ros is not None and last_isaac is not None and first_ros < last_isaac else 'ROS 경로가 모두 Isaac 경로 뒤'}", flush=True)
print(f"[probe] ~/.local 경로: {[C.short(p) for p in sys.path if C.label(p) == 'LOCAL!'] or '없음'}", flush=True)

# 모듈 import 경로
import importlib

mods = ["numpy", "torch", "scipy", "warp", "curobo", "rclpy", "std_msgs.msg", "sensor_msgs.msg", "builtin_interfaces.msg",
        "hrc_interfaces.msg"]
res = {}
for m in mods:
    try:
        mod = importlib.import_module(m)
        top = importlib.import_module(m.split(".")[0])
        res[m] = dict(file=C.short(getattr(mod, "__file__", "?")), version=str(getattr(top, "__version__", "")),
                      where=C.label(getattr(mod, "__file__", "") or ""))
    except Exception as e:
        res[m] = dict(error=f"{type(e).__name__}: {e}")
    print(f"[probe] import {m:24s} " + json.dumps(res[m], ensure_ascii=False), flush=True)

# rclpy: 노드 생성, 같은 프로세스 안에서 HumanPose / HumanDynamicsState 발행→구독 1회씩
ok = {}
try:
    import rclpy
    from rclpy.utilities import get_rmw_implementation_identifier
    from hrc_interfaces.msg import HumanDynamicsState, HumanPose

    rclpy.init()
    node = rclpy.create_node("step3_probe")
    got = {}
    node.create_subscription(HumanPose, "/step3_probe/pose", lambda m: got.setdefault("pose", m), 10)
    node.create_subscription(HumanDynamicsState, "/step3_probe/dyn", lambda m: got.setdefault("dyn", m), 10)
    pp = node.create_publisher(HumanPose, "/step3_probe/pose", 10)
    pd = node.create_publisher(HumanDynamicsState, "/step3_probe/dyn", 10)
    t0 = time.time()
    while time.time() - t0 < 3.0 and len(got) < 2:
        m = HumanPose(); m.body_full_pose = [0.1] * 66; m.global_trans = [1.0, 2.0, 3.0]; pp.publish(m)
        d = HumanDynamicsState(); d.body_part_indices = list(range(19)); d.body_part_positions = [0.5] * 57; pd.publish(d)
        rclpy.spin_once(node, timeout_sec=0.05)
    ok = dict(rmw=get_rmw_implementation_identifier(), domain=os.environ.get("ROS_DOMAIN_ID"),
              pose_roundtrip=("pose" in got and len(got["pose"].body_full_pose) == 66),
              dyn_roundtrip=("dyn" in got and len(got["dyn"].body_part_positions) == 57))
    node.destroy_node()
    rclpy.shutdown()
except Exception as e:
    ok = dict(error=f"{type(e).__name__}: {e}")
print(f"[probe] rclpy 자체 발행→구독 " + json.dumps(ok, ensure_ascii=False), flush=True)

# 메모리에 올라온 공유 라이브러리: /opt/ros·hrc_ws 에서 온 것과, 같은 이름이 Isaac 쪽에도 있는지
maps = set()
for line in open("/proc/self/maps"):
    parts = line.split()
    if len(parts) >= 6 and ".so" in parts[-1]:
        maps.add(parts[-1])
ros_libs = sorted(p for p in maps if p.startswith("/opt/ros") or "/hrc_ws/" in p)
isaac_names = {}
for p in maps:
    if p.startswith(C.ISAAC):
        isaac_names.setdefault(os.path.basename(p), []).append(p)
clash = [os.path.basename(p) for p in ros_libs if os.path.basename(p) in isaac_names]
key = {}
for k in ["libtorch_cpu.so", "libc10.so", "libc10_cuda.so", "libcudart", "openblas", "libgomp", "libstdc++", "librcl.so",
          "librmw_fastrtps_cpp.so", "libfastrtps.so", "libhrc_interfaces__rosidl_typesupport_c.so", "libspdlog", "libyaml",
          "libtinyxml2", "libssl", "libcrypto"]:
    hit = sorted({C.short(p) for p in maps if k in os.path.basename(p)})
    if hit:
        key[k] = hit
print(f"[probe] 로드된 .so 총 {len(maps)}개, /opt/ros·hrc_ws 에서 온 것 {len(ros_libs)}개", flush=True)
print(f"[probe] /opt/ros·hrc_ws 라이브러리 중 Isaac 쪽에도 같은 이름으로 로드된 것: {clash or '없음'}", flush=True)
for k, v in key.items():
    print(f"[probe] so {k:44s} {v}", flush=True)
print("[probe] DONE", flush=True)
if WITH_KIT:
    app.close()
