"""step1_python_check.py — Isaac Sim 내장 Python 점검 (run_isaac.sh 로 실행)
   인자 없음: Kit 을 띄우지 않고 sys/torch/pxr 확인 (EULA 불필요)
   --with-kit: SimulationApp(headless) 를 띄운 뒤 pxr 경로 확인 (EULA 동의 필요)
"""
import sys, os, time

def section(t): print(f"\n### {t}")

section("sys")
print("sys.version   :", sys.version.replace("\n", " "))
print("sys.executable:", sys.executable)
home_local = os.path.expanduser("~/.local")
bad = [p for p in sys.path if p.startswith(home_local) or p.startswith("/opt/ros")]
print("sys.path 항목 수:", len(sys.path))
print("~/.local 또는 /opt/ros 포함 항목:", bad if bad else "없음")
print("PYTHONNOUSERSITE:", os.environ.get("PYTHONNOUSERSITE"))
print("ROS_DISTRO/AMENT_PREFIX_PATH:", os.environ.get("ROS_DISTRO"), os.environ.get("AMENT_PREFIX_PATH"))
print("CUDA_HOME:", os.environ.get("CUDA_HOME"))

section("torch")
import torch
print("torch.__version__ :", torch.__version__)
print("torch.version.cuda:", torch.version.cuda)
print("cudnn             :", torch.backends.cudnn.version())
print("torch.__file__    :", torch.__file__)
print("cuda.is_available :", torch.cuda.is_available())
if torch.cuda.is_available():
    print("device 0          :", torch.cuda.get_device_name(0), "cap", torch.cuda.get_device_capability(0))
    a = torch.randn(2048, 2048, device="cuda"); b = torch.randn(2048, 2048, device="cuda")
    torch.cuda.synchronize(); t0 = time.perf_counter()
    c = (a @ b).sum(); torch.cuda.synchronize()
    print(f"GPU matmul 2048x2048 sum={c.item():.3f}  ({(time.perf_counter()-t0)*1000:.2f} ms)")
    x = torch.arange(10, device="cuda", dtype=torch.float32)
    print("GPU 텐서 연산 (arange*2).sum() =", (x * 2).sum().item(), "(기대값 90.0)")
    print("VRAM allocated MiB:", torch.cuda.memory_allocated() / 2**20)

section("pxr (Kit 없이)")
try:
    import pxr
    print("pxr.__file__:", getattr(pxr, "__file__", pxr.__path__))
except Exception as e:
    print("Kit 없이 pxr import 실패:", type(e).__name__, e)

if "--with-kit" in sys.argv:
    section("pxr (SimulationApp headless)")
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import pxr, omni
    print("pxr.__path__:", list(pxr.__path__))
    from pxr import Usd
    print("Usd module  :", Usd.__file__)
    print("isaacsim    :", __import__("isaacsim").__file__)
    bad = [p for p in sys.path if p.startswith(home_local) or p.startswith("/opt/ros")]
    print("Kit 기동 후 sys.path 에 ~/.local 또는 /opt/ros:", bad if bad else "없음")
    app.close()
print("\nDONE")
