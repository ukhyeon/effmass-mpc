"""g1_spin_probe.py — 진단만: Isaac Python(Kit 없음)에서 Λ_r numpy 계산·각 numpy 연산 뒤 쉬는 동안 프로세스 CPU 를 재서 바쁜 대기 스레드가 생기는지 본다"""
import os, sys, time, threading
sys.path.append(os.path.expanduser("~/effmass_mpc/ws/install/effmass_robot/lib/python3.10/site-packages"))
import numpy as np
from effmass_robot.robot_inertia_np import RobotInertiaNP
def nthreads(): return len(os.listdir("/proc/self/task"))
def idle_cpu(fn, n=60, label=""):
    """fn 을 60 Hz 로 n 번: 각 주기에서 fn 뒤 남은 시간 sleep. 반환: 벽시계 대비 프로세스 CPU 비율 (1.0 = 한 코어)"""
    w0, c0 = time.perf_counter(), time.process_time()
    for _ in range(n):
        t = time.perf_counter(); fn(); time.sleep(max(0.0, 1 / 60 - (time.perf_counter() - t)))
    return round((time.process_time() - c0) / (time.perf_counter() - w0), 2)
ri = RobotInertiaNP(); q = np.array([0, -1.3, 0, -2.5, 0, 1.0, 0.0])
cfg = {}
try:
    import numpy.__config__ as C; cfg = {k: v for k, v in C.CONFIG.get("Build Dependencies", {}).get("blas", {}).items() if k in ("name", "version", "openblas configuration")}
except Exception as e:
    cfg = repr(e)
print("[spin] numpy", np.__version__, np.__file__, "blas", cfg, "OPENBLAS_NUM_THREADS", os.environ.get("OPENBLAS_NUM_THREADS"), "OMP_NUM_THREADS", os.environ.get("OMP_NUM_THREADS"), "cores", os.cpu_count(), flush=True)
print("[spin] threads start", nthreads(), "sleep only cpu/wall", idle_cpu(lambda: None), flush=True)
A = np.random.rand(61, 3, 7); M = np.random.rand(7, 7) + 7 * np.eye(7); B = np.random.rand(7, 30)
for label, fn in [("inv 7x7", lambda: np.linalg.inv(M)), ("A@A.T 7x30", lambda: B @ B.T), ("batched (61,3,7)@(7,7)@(61,7,3)", lambda: A @ M @ A.transpose(0, 2, 1)),
                  ("einsum lki,lij,lmj", lambda: np.einsum("lki,lij,lmj->km", np.random.rand(10, 7, 3), np.random.rand(10, 3, 3), np.random.rand(10, 7, 3))),
                  ("RobotInertiaNP.compute", lambda: ri.compute(q))]:
    r = idle_cpu(fn, label=label); print(f"[spin] {label:34s} threads {nthreads():3d}  cpu/wall {r}", flush=True)
import torch, warp
print("[spin] after torch/warp import threads", nthreads(), "RobotInertiaNP cpu/wall", idle_cpu(lambda: ri.compute(q)), "torch threads", torch.get_num_threads(), flush=True)
# 대안 후보 (진단만, 측정 설정에 적용 안 함)
def chol_inv_np(M):
    """LAPACK 없이 7x7 대칭 양정치 역행렬 (Cholesky, 파이썬 반복 7회 + 배열 연산)"""
    n = M.shape[0]; L = np.zeros_like(M)
    for j in range(n):
        d = M[j, j] - L[j, :j] @ L[j, :j]; L[j, j] = np.sqrt(d)
        L[j + 1:, j] = (M[j + 1:, j] - L[j + 1:, :j] @ L[j, :j]) / L[j, j]
    Li = np.zeros_like(M)
    for j in range(n):
        e = np.zeros(n); e[j] = 1.0
        for i in range(j, n):
            e[i] = (e[i] - L[i, j:i] @ Li[j:i, j]) / L[i, i] if i > j else 1.0 / L[i, i]
            Li[i, j] = e[i]
    return Li.T @ Li
Ms = M @ M.T + np.eye(7)
for label, fn in [("np.linalg.solve(M, I)", lambda: np.linalg.solve(Ms, np.eye(7))), ("np.linalg.cholesky", lambda: np.linalg.cholesky(Ms)),
                  ("chol_inv_np (LAPACK 없음)", lambda: chol_inv_np(Ms))]:
    r = idle_cpu(fn); t = time.perf_counter(); [fn() for _ in range(200)]; us = (time.perf_counter() - t) / 200 * 1e6
    print(f"[spin] alt {label:28s} cpu/wall {r}  {us:.1f} us/call", flush=True)
print("[spin] chol_inv_np vs inv max rel", float(np.abs(chol_inv_np(Ms) - np.linalg.inv(Ms)).max() / np.abs(np.linalg.inv(Ms)).max()), flush=True)
