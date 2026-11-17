"""b2a_pose.py — B2a 정지 자세: 기록 재생 첫 프레임의 선 자세 + 오른팔 앞으로 수평으로 뻗음 (g1_run_human.sh py 로 실행)
SMPL 포즈(축-각, 22관절×3, 0 = 전역 방향은 카메라 좌표 그대로)에서 오른 어깨(17)만 최적화, 오른 팔꿈치(19)·오른 손목(21) = 0.
목표: 위팔·아래팔 방향 = 몸 정면 수평(골반 정면을 수평면에 투영), 손바닥 법선(손 분절 −y) = 아래(−z). 계산은 effmass_human HumanInertiaCore 그대로.
출력: goal1/data/human_b2a_reach.npz (precomputed 형식, 1 프레임, 속도 0), goal1/b2a/pose_report.json"""
import json, os
import numpy as np, torch
from scipy.optimize import minimize
torch.set_num_threads(1)
from effmass_human.human_inertia_core import HumanInertiaCore, SEGMENT_NAMES
H = os.path.expanduser("~")
z = np.load(f"{H}/hrc_ws/pose_npz/human_state_1781524848.npz"); P0 = z["poses"][0].astype(np.float32).reshape(22, 3); T0 = z["trans"][0].astype(np.float32)
core = HumanInertiaCore("cpu"); core.set_betas(np.zeros(10, np.float32))
RS, RE, RW = 17, 19, 21          # SMPL 오른 어깨·팔꿈치·손목
f32 = lambda x: x.detach().cpu().numpy().astype(np.float64)


EL = [np.zeros(3, np.float32)]   # 팔꿈치 축-각 (2단계에서 실제 각도 0° 가 되게 최적화)


def run(sh):
    p = P0.copy(); p[RS] = sh; p[RE] = EL[0]; p[RW] = 0
    return p, core.compute(p.reshape(-1), T0, prev=None, dt=None)


_, o0 = run(P0[RS])
R0 = f32(o0["world_rot"][0])[0]; fwd = R0 @ np.array([0, 0, 1.0]); fwd[2] = 0; fwd /= np.linalg.norm(fwd)   # 골반 정면 (SMPL +z) 의 수평 성분 (노드 world z 위)
ang = lambda a, b: np.arccos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))


def geom(o):
    J = f32(o["joints"][0]); R = f32(o["world_rot"][0]); D = np.asarray(o["distal"], float)
    sh, el, wr = J[14], J[16], J[18]; tip = D[18]
    return dict(J=J, R=R, sh=sh, el=el, wr=wr, tip=tip, ua=el - sh, fa=wr - el, palm=R[18] @ np.array([0, -1.0, 0]))


def obj(x):
    _, o = run(x.astype(np.float32)); g = geom(o)
    return ang(g["ua"], fwd) ** 2 + ang(g["fa"], fwd) ** 2 + ang(g["palm"], np.array([0, 0, -1.0])) ** 2


best = None
for x0 in ([0, 1.57, 0], [0, -1.57, 0], [1.57, 0, 0], [-1.57, 0, 0], [0, 0, 1.57], [0, 0, -1.57], list(P0[RS])):
    r = minimize(obj, np.array(x0, float), method="Nelder-Mead", options=dict(xatol=1e-5, fatol=1e-10, maxiter=4000))
    if best is None or r.fun < best.fun:
        best = r
# 2단계: 어깨 + 팔꿈치 (6) — 팔꿈치 실제 각도 0°, 같은 방향·손바닥 조건
def obj2(x):
    EL[0] = x[3:].astype(np.float32); _, o = run(x[:3].astype(np.float32)); g = geom(o)
    return ang(g["ua"], fwd) ** 2 + ang(g["fa"], fwd) ** 2 + ang(g["palm"], np.array([0, 0, -1.0])) ** 2 + 4 * ang(g["ua"], g["fa"]) ** 2
r2 = minimize(obj2, np.r_[best.x, 0, 0, 0], method="Nelder-Mead", options=dict(xatol=1e-6, fatol=1e-12, maxiter=20000, maxfev=20000))
EL[0] = r2.x[3:].astype(np.float32); best_sh = r2.x[:3]
p, o = run(best_sh.astype(np.float32)); g = geom(o)
spine = g["J"][9] - g["J"][0]            # 골반 관절 → 목 관절 (몸통 축, 위)
elbow_deg = np.degrees(ang(g["ua"], g["fa"]))
flex_deg = np.degrees(ang(g["ua"], -spine))
# 손 형상 ↔ 몸통 형상 최소 표면 거리
sh_ = o["shapes"]; P0s, P1s, RAD = sh_["p0"], sh_["p1"], sh_["radius"]


def segseg(p1, q1, p2, q2, n=201):
    t = np.linspace(0, 1, n); A = p1 + np.outer(t, q1 - p1); B = p2 + np.outer(t, q2 - p2)
    return np.linalg.norm(A[:, None] - B[None], axis=2).min()


HAND = [22, 23]; TORSO = [0, 1, 4, 5, 10, 11]; FORE = [19]; HEAD = [15]
dmin = lambda A, B: min(segseg(P0s[a], P1s[a], P0s[b], P1s[b]) - RAD[a] - RAD[b] for a in A for b in B)
per_shape = {f"{a}-{b}": float(segseg(P0s[a], P1s[a], P0s[b], P1s[b]) - RAD[a] - RAD[b]) for a in HAND for b in TORSO + [14, 17, 12]}
rep = dict(per_shape_gap=per_shape, hand_radius=[float(RAD[22]), float(RAD[23])], shoulder_axis_angle_rad=best_sh.round(5).tolist(), elbow_axis_angle_rad=r2.x[3:].round(5).tolist(), objective=float(r2.fun), elbow_flex_deg=float(elbow_deg), shoulder_flexion_deg=float(flex_deg),
           upperarm_vs_forward_deg=float(np.degrees(ang(g["ua"], fwd))), forearm_vs_forward_deg=float(np.degrees(ang(g["fa"], fwd))),
           palm_vs_down_deg=float(np.degrees(ang(g["palm"], np.array([0, 0, -1.0])))), upperarm_elev_deg=float(np.degrees(np.arcsin(g["ua"][2] / np.linalg.norm(g["ua"])))),
           forearm_elev_deg=float(np.degrees(np.arcsin(g["fa"][2] / np.linalg.norm(g["fa"])))),
           hand_torso_min_gap_m=float(dmin(HAND, TORSO)), hand_head_min_gap_m=float(dmin(HAND, HEAD)), forward_node_world=fwd.tolist(),
           shoulder=g["sh"].tolist(), elbow=g["el"].tolist(), wrist=g["wr"].tolist(), hand_tip=g["tip"].tolist(), hand_com=f32(o["com"][0])[18].tolist(),
           pose_changed={"17 R_shoulder": [P0[RS].tolist(), best_sh.tolist()], "19 R_elbow": [P0[RE].tolist(), r2.x[3:].tolist()], "21 R_wrist": [P0[RW].tolist(), [0, 0, 0]]},
           trans=T0.tolist(), betas=[0.0] * 10, frame0_source="human_state_1781524848.npz frame 0")
print(json.dumps(rep, indent=1))
sv = lambda x: f32(x[0]).astype(np.float32)[None]
np.savez(f"{H}/effmass_mpc/goal1/data/human_b2a_reach.npz", input_time=np.zeros(1), frame=np.zeros(1, int), mass=f32(core.mass_19_body[0]).astype(np.float32),
         shape_type=np.asarray(sh_["type"], np.int64), shape_segment=np.asarray(sh_["segment"], np.int64), shape_radius=RAD.astype(np.float32), segment_names=np.array(SEGMENT_NAMES),
         rate=30.0, betas=np.zeros(10, np.float32), source_npz="b2a static reach", vdt_from="", com=sv(o["com"]), vel=np.zeros((1, 19, 3), np.float32), rot=sv(o["world_rot"]),
         inv=sv(o["inv_inertia"]), jprox=sv(o["joints"]), jdist=np.asarray(o["distal"], np.float32)[None], p0=P0s.astype(np.float32)[None], p1=P1s.astype(np.float32)[None],
         vdt=np.zeros(1, np.float32), vvalid=np.zeros(1, bool), compute_ms=np.zeros(1), pose66=p.reshape(1, -1))
json.dump(rep, open(f"{H}/effmass_mpc/goal1/b2a/pose_report.json", "w"), indent=1)
