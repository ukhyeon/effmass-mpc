"""Isaac Python 에서 시스템 ROS Pinocchio 를 import 할 수 있는지만 확인 (SimulationApp 없음, 설치 없음)"""
import sys, os
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts"))
import step3_ros_common as RC
RC.add_ros_paths()
import numpy as np, torch
print("[pin-isaac] numpy", np.__version__, np.__file__, "torch", torch.__version__, flush=True)
try:
    import pinocchio as pin
    m = pin.buildModelFromUrdf(os.path.expanduser("~/effmass_mpc/goal1/fr3v2_1/fr3v2_1_curobo_narrow.urdf")); d = m.createData()
    q = pin.neutral(m); pin.crba(m, d, q)
    print("[pin-isaac] OK", pin.__version__, pin.__file__, "nv", m.nv, "M[0,0]", float(d.M[0, 0]), flush=True)
except Exception as e:
    print("[pin-isaac] 실패", type(e).__name__, str(e)[:200], flush=True)
