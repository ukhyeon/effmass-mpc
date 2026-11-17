"""step4_make_transform.py — 임시 변환(node world → Isaac 로봇 기준) 계산과 겹침 확인 (시스템 python3 -B, numpy 만)

  임시 변환 (사용자 지시 4-0 결정 6):
    - 사람 정면(골반 world 회전의 SMPL +z 축)이 로봇 쪽(−x)을 보도록 z축 회전 ψ (전 프레임 중앙값 기준)
    - 골반 COM xy 중앙값 → (0.9, 0)  (로봇 베이스 앞 0.9 m)
    - 바닥 = 로봇 베이스 아래 0.9 m (z = −0.9). 발 COM z 중앙값 = 바닥 + h_foot
      h_foot = 정지 자세(betas 0)에서 발 분절(SMPL 7·10, 19분절 6) 정점 최저점 → 발 COM 높이
  출력: step4/temp_transform.json (R, t, 정보), 확인 수치 출력
"""
import json, os, pickle, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step4_ref_fk as F

NPZ = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/hrc_ws/pose_npz/human_state_1781524848.npz")
FLOOR_Z = -0.9
RAD = [0.12, 0.08, 0.08, 0.11, 0.06, 0.06, 0.05, 0.05, 0.12, 0.06, 0.06, 0.06, 0.10, 0.05, 0.05, 0.045, 0.045, 0.045, 0.045]

c = F.rest_constants(np.zeros(10))
z = np.load(NPZ)
com, J, W = F.com_world(z["poses"], z["trans"], c)
N = len(com)
# 정면: SMPL 몸 정면 = 몸 좌표 +z
fw = W[:, 0] @ np.array([0, 0, 1.0])
yaw_now = np.median(np.arctan2(fw[:, 1], fw[:, 0]))
psi = np.pi - yaw_now
Rz = np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
# h_foot: 정지 자세 발 정점 최저점 → 발 COM (SMPL-H y 가 위쪽)
h = np.load(F.SMPLH_M_PATH); vsh = h["v_template"].astype(np.float64)
phys = pickle.load(open(F.PHYSICS_PATH, "rb"), encoding="latin1")
foot_v = np.concatenate([np.asarray(phys["bodypart_vertexid"][b]) for b in (7, 10)])
foot_v = foot_v[foot_v < 6890]
h_foot = float(c["centroid_19"][6][1] - vsh[foot_v, 1].min())
P = com @ Rz.T
pel_xy = np.median(P[:, 0, :2], axis=0)
foot_z = np.median(P[:, [6, 7], 2])
t = np.array([0.9 - pel_xy[0], 0.0 - pel_xy[1], FLOOR_Z + h_foot - foot_z])
X = P + t                                                     # Isaac 로봇 기준 COM (N,19,3)
out = dict(R=Rz.tolist(), t=t.tolist(), psi_deg=float(np.degrees(psi)), facing_yaw_node_world_deg=float(np.degrees(yaw_now)),
           floor_z=FLOOR_Z, h_foot=h_foot, npz=NPZ, n=N, rule="p_isaac = R @ p_nodeworld + t ; v_isaac = R @ v_nodeworld")
os.makedirs(os.path.expanduser("~/effmass_mpc/env_check/step4"), exist_ok=True)
json.dump(out, open(os.path.expanduser("~/effmass_mpc/env_check/step4/temp_transform.json"), "w"), indent=1)
print(f"[tf] 임시 변환: ψ {np.degrees(psi):.1f} deg (node world 정면 yaw 중앙값 {np.degrees(yaw_now):.1f} deg), t {np.round(t, 4).tolist()}, h_foot {h_foot:.4f} m")

med = np.median(X, axis=0)
names = F.NAMES_19
print("[tf] 변환 후 분절 COM 중앙값 (x, y, z) m:")
for b in (0, 3, 8, 9, 12, 13, 14, 15, 16, 17, 18, 6, 7):
    print(f"[tf]   {names[b]:15s} {np.round(med[b], 3).tolist()}")
fz = X[:, [6, 7], 2]; print(f"[tf] 발 COM z: 중앙 {np.median(fz):.3f}, 최소 {fz.min():.3f}, 최대 {fz.max():.3f} (바닥 {FLOOR_Z} + h_foot {h_foot:.3f})")
# Franka 작업 공간: 어깨(joint2) 높이 z 0.333, 도달 반경 약 0.855 m (Franka Panda 사양)
SH = np.array([0, 0, 0.333]); REACH = 0.855
d = np.linalg.norm(X - SH, axis=2)
upper = list(range(8, 19)) + [3]
print(f"[tf] Franka 어깨(0,0,0.333)에서 거리: 상체·팔 분절 최소 {d[:, upper].min():.3f} m, 중앙 {np.median(d[:, upper]):.3f} m; "
      f"도달 반경 {REACH} m 안에 든 상체·팔 COM 비율 {(d[:, upper] < REACH).mean() * 100:.1f}%")
for b in (12, 8, 15, 16, 17, 18):
    print(f"[tf]   {names[b]:15s} z 범위 {X[:, b, 2].min():.3f}~{X[:, b, 2].max():.3f} m, 어깨 거리 최소 {d[:, b].min():.3f} m")
G = np.array([[0.4, -0.5, 0.4], [0.4, 0.5, 0.4]])
dg = np.linalg.norm(X[:, None, :, :] - G[None, :, None, :], axis=3).min(axis=(0, 2))
print(f"[tf] MPC 목표 (0.4,∓0.5,0.4) 까지 사람 분절 COM 최소 거리: {np.round(dg, 3).tolist()} m")
# 2단계 장면 정적 cuboid (collision_wall.yml table·wall, v2 블록, mover 왕복 범위)와 겹침 (구 반지름 = 표시용 RAD)
def sbd(p, c, d):
    q = np.abs(p - c) - np.asarray(d) / 2
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)
cubs = {"table(2.2x2.2x0.2, top z -0.01)": ([0, 0, -0.11], [2.2, 2.2, 0.2]), "wall cube4": ([-0.5, 0, 0.3], [0.05, 2.0, 2.0]),
        "block_low": ([0.55, 0, 0.15], [0.10, 0.10, 0.30]), "block_high": ([0.30, 0, 0.85], [0.10, 0.40, 0.10]),
        "mover 왕복 범위(v2, x 0.65~0.85)": ([0.75, 0, 0.45], [0.20 + 0.08, 0.08, 0.08])}
for name, (cc, dd) in cubs.items():
    clr = sbd(X, np.asarray(cc, float), dd) - np.asarray(RAD)[None, :]
    nb = np.where((clr < 0).any(0))[0]
    print(f"[tf] {name:34s} 최소 여유 {clr.min():+.3f} m, 겹치는 프레임 {int((clr < 0).any(1).sum())}/{N}" +
          (f", 겹치는 분절 {[names[b] for b in nb]}" if len(nb) else ""))
np.save(os.path.expanduser("~/effmass_mpc/env_check/step4/replay_com_isaac.npy"), X.astype(np.float32))
