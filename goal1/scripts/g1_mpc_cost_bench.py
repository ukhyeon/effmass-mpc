"""g1_mpc_cost_bench.py — 1-2a-3 조사용 시간·메모리 추정 (구현 아님). Isaac Python, torch CUDA, SimulationApp 없음
  rollout 상태 B = 400 x 30 = 12,000 개에 대해:
   (R) fr3v2_1 질량행렬 M(q) (7x7, URDF 관성), 로봇 충돌 구 중심의 선형 야코비안 J_s (B,S,3,7), 역관성 A_s = J_s M^-1 J_s^T (B,S,3,3)
   (H) 로봇 구 x 사람 19 분절 거리 → 가장 가까운 분절, u, m_h(u) = 1/(u^T Λh^-1 u) (Λh^-1: 19 x 3x3 고정 텐서), m_r(-u) = 1/(u^T A_s u), μ
  검증: M(q) ↔ numpy 구현(g1_variant_meff, Pinocchio 와 1e-15 일치) 3 상태, 구 중심 ↔ cuRobo robot_spheres 3 상태"""
import os, sys, json, time, numpy as np, torch, yaml
import xml.etree.ElementTree as ET
os.environ["STEP_ROBOT"] = "fr3v2_1"
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts"))
import step2_scene as S
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
dev = torch.device("cuda"); dt = torch.float32
URDF = os.path.expanduser("~/effmass_mpc/goal1/fr3v2_1/fr3v2_1_curobo_narrow.urdf")
SPH = yaml.safe_load(open(os.path.expanduser("~/effmass_mpc/goal1/fr3v2_1/spheres/fr3v2_1_port.yml")))["collision_spheres"]
g = {}; exec(open(os.path.expanduser("~/effmass_mpc/goal1/scripts/g1_variant_meff.py")).read().split("models = {m: load(m)")[0], g)
g["V"] = os.path.dirname(URDF); joints, inert, _ = g["load"](os.path.basename(URDF)[:-5])
# 위상 순서
order = []; done = {"base"}
while len(order) < len(joints):
    for c, j in joints.items():
        if c not in done and j["parent"] in done: order.append(c); done.add(c)
T = lambda a: torch.tensor(np.asarray(a), device=dev, dtype=dt)
JT = {c: T(joints[c]["T"]) for c in order}
EYE4 = torch.eye(4, device=dev, dtype=dt)
PRIS = {c: (lambda P: (P.__setitem__((slice(0, 3), 3), T(joints[c]["axis"]) * 0.04), P)[1])(torch.eye(4, device=dev, dtype=dt)) for c in order if joints[c]["type"] == "prismatic"}
def fk(q):  # q (B,7)  (상수 텐서는 미리 만들어 둠: CUDA graph 캡처 중 CPU→GPU 복사 없음)
    B = q.shape[0]; W = {"base": EYE4.expand(B, 4, 4)}; jo = {}
    for c in order:
        j = joints[c]; A = W[j["parent"]] @ JT[c]
        if j["type"] == "revolute":
            k = int(j["name"][-1]) - 1; ax = T(j["axis"]); th = q[:, k]
            Rj = EYE4.repeat(B, 1, 1)
            cth, sth = torch.cos(th), torch.sin(th)
            Rj[:, 0, 0] = cth; Rj[:, 0, 1] = -sth; Rj[:, 1, 0] = sth; Rj[:, 1, 1] = cth   # 축 z (FR3 관절 모두 z, 아래에서 확인)
            jo[k] = (A[:, :3, 3], A[:, :3, 2]); A = A @ Rj
        elif j["type"] == "prismatic":
            A = A @ PRIS[c]
        W[c] = A
    return W, jo
assert all(np.allclose(j["axis"], [0, 0, 1]) for j in joints.values() if j["type"] == "revolute")
def chain_mask(link):
    m = torch.zeros(7, device=dev, dtype=dt); c = link
    while c in joints:
        if joints[c]["type"] == "revolute": m[int(joints[c]["name"][-1]) - 1] = 1
        c = joints[c]["parent"]
    return m
def jac(jo, p, mask):  # p (B,...,3) → (B,...,3,7)
    O = torch.stack([jo[k][0] for k in range(7)], 1); Z = torch.stack([jo[k][1] for k in range(7)], 1)   # (B,7,3)
    shp = p.shape[1:-1]; Ov = O.view(O.shape[0], *([1] * len(shp)), 7, 3); Zv = Z.view(Z.shape[0], *([1] * len(shp)), 7, 3)
    Jv = torch.cross(Zv.expand(*p.shape[:-1], 7, 3), p.unsqueeze(-2) - Ov, dim=-1).transpose(-1, -2) * mask.unsqueeze(-2)
    return Jv
LINKS = [l for l in inert if l in joints]
LM = torch.stack([chain_mask(l) for l in LINKS]); MASS = T([inert[l]["m"] for l in LINKS]); IN = T(np.stack([inert[l]["I"] for l in LINKS])); CT = T(np.stack([inert[l]["T"] for l in LINKS]))
SL = [l for l in SPH for _ in SPH[l]]; SC = T([s["center"] + [1.0] for l in SPH for s in SPH[l]]); SM = torch.stack([chain_mask(l) for l in SL])
def mass_matrix(q):
    W, jo = fk(q); B = q.shape[0]
    A = torch.stack([W[l] for l in LINKS], 1) @ CT                 # (B,L,4,4) COM 프레임
    p = A[..., :3, 3]; R = A[..., :3, :3]
    Jv = jac(jo, p, LM)                                             # (B,L,3,7)
    Z = torch.stack([jo[k][1] for k in range(7)], 1)                 # (B,7,3)
    Jw = Z.transpose(1, 2).unsqueeze(1) * LM.view(1, len(LINKS), 1, 7)
    Iw = R @ IN @ R.transpose(-1, -2)
    M = (MASS.view(1, -1, 1, 1) * Jv.transpose(-1, -2) @ Jv + Jw.transpose(-1, -2) @ Iw @ Jw).sum(1)
    return M, W, jo
def robot_inv_inertia(q):
    M, W, jo = mass_matrix(q)
    ps = (torch.stack([W[l] for l in SL], 1) @ SC.view(1, -1, 4, 1)).squeeze(-1)[..., :3]   # (B,S,3)
    Js = jac(jo, ps, SM)                                                                        # (B,S,3,7)
    Lc, _info = torch.linalg.cholesky_ex(M)
    X = torch.cholesky_solve(Js.transpose(-1, -2).permute(0, 2, 1, 3).reshape(M.shape[0], 7, -1), Lc)   # M^-1 [J_1^T ... J_S^T] (B,7,3S)
    X = X.view(M.shape[0], 7, len(SL), 3).permute(0, 2, 1, 3)                                    # (B,S,7,3)
    A = Js @ X                                                                                   # (B,S,3,3) 구마다 J M^-1 J^T
    return A, ps, M
def human_terms(ps, A, Xh, Lh):
    d = torch.cdist(ps.reshape(-1, 3), Xh).view(ps.shape[0], ps.shape[1], -1)                 # (B,S,19)
    k = d.argmin(-1); xk = Xh[k]; u = xk - ps; u = u / (u.norm(dim=-1, keepdim=True) + 1e-9)
    mh = 1.0 / (u.unsqueeze(-2) @ Lh[k] @ u.unsqueeze(-1)).squeeze(-1).squeeze(-1)
    mr = 1.0 / (u.unsqueeze(-2) @ A @ u.unsqueeze(-1)).squeeze(-1).squeeze(-1)
    return mh * mr / (mh + mr), d.gather(-1, k.unsqueeze(-1)).squeeze(-1), mh, mr
ta = TensorDeviceType(); cfg = S.build_robot_cfg(); rc = RobotConfig.from_dict(cfg, ta); km = CudaRobotModel(rc.kinematics)
jl = rc.kinematics.get_joint_limits().position
B = 400 * 30; torch.manual_seed(0)
q = (jl[0] + (jl[1] - jl[0]) * torch.rand(B, 7, device=dev)).to(dt)
# 검증
M3, W3, jo3 = mass_matrix(q[:3]); errM = max(float(np.abs(M3[i].cpu().numpy() - g["mass_matrix"]((joints, inert, "fr3v2_1_"), q[i].cpu().numpy().astype(float))[0]).max()) for i in range(3))
_, ps3, _ = robot_inv_inertia(q[:3]); st = km.get_state(q[:3].contiguous()); rs = st.link_spheres_tensor[..., :3]
errS = float((rs[:, :ps3.shape[1]] - ps3).norm(dim=-1).max()) if rs.shape[1] >= ps3.shape[1] else None
print(f"[bench] 검증: M(q) torch fp32 ↔ numpy fp64 최대 차 {errM:.2e} (M 최대 원소 {float(M3.abs().max()):.3f}), 구 중심 ↔ cuRobo robot_spheres 최대 차 {errS*1000 if errS is not None else 'n/a':.4f} mm (구 {ps3.shape[1]}개, cuRobo {rs.shape[1]}개)", flush=True)
Xh = torch.tensor(np.load(os.path.expanduser("~/effmass_mpc/env_check/step4/replay_com_isaac.npy"))[0], device=dev, dtype=dt)
Lr = torch.randn(19, 3, 3, device=dev, dtype=dt); Lh = Lr @ Lr.transpose(1, 2) * 0.01 + 0.05 * torch.eye(3, device=dev)
def run():
    A, ps, M = robot_inv_inertia(q); return human_terms(ps, A, Xh, Lh), A
for _ in range(3): run()
torch.cuda.synchronize(); torch.cuda.reset_peak_memory_stats(); base = torch.cuda.memory_allocated()
e0, e1, e2 = [torch.cuda.Event(enable_timing=True) for _ in range(3)]; tR, tH = [], []
for _ in range(20):
    e0.record(); A, ps, M = robot_inv_inertia(q); e1.record(); out = human_terms(ps, A, Xh, Lh); e2.record(); torch.cuda.synchronize()
    tR.append(e0.elapsed_time(e1)); tH.append(e1.elapsed_time(e2))
peak = (torch.cuda.max_memory_allocated() - base) / 2**20
mu, dmin, mh, mr = out
print(f"[bench] B={B} 상태, 로봇 구 {ps.shape[1]}개, 사람 분절 19: (R) M·J·A {np.median(tR):.2f} ms (중앙, 20회), (H) 거리·u·m_h·m_r·μ {np.median(tH):.2f} ms, 추가 GPU 메모리 최대 {peak:.0f} MiB", flush=True)
base_sph = torch.tensor([l.endswith("link0") for l in SL], device=dev)
mrv = mr[:, ~base_sph]
print(f"[bench] m_r 범위 (link0 구 {int(base_sph.sum())}개 제외: 야코비안 0 → 무한대): {float(mrv.min()):.3f}~{float(mrv.max()):.1f} kg, 중앙 {float(mrv.median()):.2f}; link0 구 m_r 유한 비율 {float(torch.isfinite(mr[:, base_sph]).float().mean()):.2f}", flush=True)
# 현재 상태 1개만 (rollout 전체 대신)
q1 = q[:1]
for _ in range(3): robot_inv_inertia(q1)
torch.cuda.synchronize(); t = []
for _ in range(20):
    e0.record(); robot_inv_inertia(q1); e1.record(); torch.cuda.synchronize(); t.append(e0.elapsed_time(e1))
print(f"[bench] 현재 상태 1개 (R): {np.median(t):.2f} ms (커널 실행 수 지배)", flush=True)
json.dump(dict(B=B, spheres=int(ps.shape[1]), robot_ms=float(np.median(tR)), human_ms=float(np.median(tH)), peak_mib=peak, errM=errM, errS_mm=errS * 1000 if errS else None, single_state_ms=float(np.median(t))),
          open(os.path.expanduser("~/effmass_mpc/goal1/logs/g1_mpc_cost_bench.json"), "w"), indent=1)
# 단계별 eager 시간 (B=12,000)
def _t(fn, n=10):
    for _ in range(2): fn()
    torch.cuda.synchronize(); r = []
    for _ in range(n):
        e0.record(); fn(); e1.record(); torch.cuda.synchronize(); r.append(e0.elapsed_time(e1))
    return float(np.median(r))
tf = _t(lambda: fk(q)); tm = _t(lambda: mass_matrix(q)); tr = _t(lambda: robot_inv_inertia(q))
print(f"[bench] 단계별 eager (B=12,000): fk {tf:.2f} ms, fk+M(q) {tm:.2f} ms, fk+M+구 야코비안+J M^-1 J^T {tr:.2f} ms", flush=True)
for hb in (1, 5, 10):
    qh = q[: 400 * hb].clone(); th_ = _t(lambda: (lambda A_ps_M: human_terms(A_ps_M[1], A_ps_M[0], Xh, Lh))(robot_inv_inertia(qh)))
    print(f"[bench] horizon 앞 {hb}스텝만 (B={400*hb}): (R)+(H) {th_:.2f} ms", flush=True)
import sys as _s; _s.exit(0)
# 부분별 캡처 가능 여부
def _try(name, fn):
    st2 = torch.cuda.Stream(); st2.wait_stream(torch.cuda.current_stream())
    with torch.cuda.stream(st2):
        for _ in range(2): fn()
    torch.cuda.current_stream().wait_stream(st2); torch.cuda.synchronize()
    gg = torch.cuda.CUDAGraph()
    try:
        with torch.cuda.graph(gg, stream=st2):
            fn()
        torch.cuda.synchronize(); print(f"[bench] 캡처 가능: {name}", flush=True); return True
    except Exception as ex:
        print(f"[bench] 캡처 실패: {name}: {str(ex).splitlines()[0][:150]}", flush=True); return False
_qq = q[:64].clone()
_try("fk", lambda: fk(_qq))
_try("mass_matrix", lambda: mass_matrix(_qq))
_try("robot_inv_inertia", lambda: robot_inv_inertia(_qq))
_A, _ps, _M = robot_inv_inertia(_qq)
_try("cdist", lambda: torch.cdist(_ps.reshape(-1, 3), Xh))
_try("human_terms", lambda: human_terms(_ps, _A, Xh, Lh))
# CUDA graph 캡처 (MPC solver 처럼 고정 텐서·고정 크기) 후 재생 시간
try:
    sq = q.clone(); s_out = {}
    st_ = torch.cuda.Stream(); st_.wait_stream(torch.cuda.current_stream())
    with torch.cuda.stream(st_):
        for _ in range(3):
            A_, ps_, M_ = robot_inv_inertia(sq); o_ = human_terms(ps_, A_, Xh, Lh)
    torch.cuda.current_stream().wait_stream(st_)
    gr = torch.cuda.CUDAGraph()
    with torch.cuda.graph(gr, stream=st_):
        A_, ps_, M_ = robot_inv_inertia(sq); o_ = human_terms(ps_, A_, Xh, Lh)
    torch.cuda.synchronize(); tg = []
    for _ in range(30):
        e0.record(); gr.replay(); e1.record(); torch.cuda.synchronize(); tg.append(e0.elapsed_time(e1))
    print(f"[bench] CUDA graph 재생 (R)+(H) 12,000 상태: 중앙 {np.median(tg):.2f} ms, 최대 {np.max(tg):.2f} ms", flush=True)
    sq1 = q[:1].clone(); gr1 = torch.cuda.CUDAGraph()
    with torch.cuda.stream(st_):
        for _ in range(3): robot_inv_inertia(sq1)
    torch.cuda.current_stream().wait_stream(st_)
    with torch.cuda.graph(gr1, stream=st_):
        A1_, _, _ = robot_inv_inertia(sq1)
    torch.cuda.synchronize(); tg1 = []
    for _ in range(30):
        e0.record(); gr1.replay(); e1.record(); torch.cuda.synchronize(); tg1.append(e0.elapsed_time(e1))
    print(f"[bench] CUDA graph 재생 (R) 현재 상태 1개: 중앙 {np.median(tg1):.3f} ms", flush=True)
    for sub in (30, 10, 5):
        pass
except Exception as ex:
    print("[bench] CUDA graph 캡처 실패", repr(ex)[:200], flush=True)
