"""step5_make_transform.py <pose_rec.npz> [첫 프레임 수, 기본 60] — 5단계 Isaac 임시 변환 (4단계 방식, 실제 sensing 기록·CLIFF betas)
  - 첫 N 프레임(받은 HumanPose)의 노드 world 분절 COM 을 독립 계산 (step4_ref_fk, 받은 betas)
  - 정면 yaw 중앙값 → 로봇 쪽(−x)을 보게 ψ, 골반 COM xy 중앙값 → (0.9, 0), 발 COM z 중앙값 = −0.9 + h_foot(betas)
  출력: step5/temp_transform.json, step5/mesh_transform.json
    mesh_transform: A = cam_R Rᵀ, b_rigid = t_cam Rᵀ + t, 노드 골반 정지 위치 J0_노드(betas) = j0_a + j0_B·betas (SMPL neutral 회귀 × SMPL-H male)
    Isaac 은 첫 메시지로 b_node = b_rigid + (J0_노드(betas) − cam_Rᵀ (smpl_joint[0] − global_trans)) Rᵀ 계산 (과제 7 노드 관례)
"""
import json, os, pickle, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step4_ref_fk as F
D = os.path.expanduser("~/effmass_mpc/env_check/step5")
Rr = np.load(sys.argv[1]); NF = int(sys.argv[2]) if len(sys.argv) > 2 else 60
betas = Rr["betas"][0].astype(np.float64)
assert np.allclose(Rr["betas"], Rr["betas"][0]), "betas 가 바뀜"
c = F.rest_constants(betas)
P66, T3 = Rr["pose"][:NF].astype(np.float64), Rr["trans"][:NF].astype(np.float64)
com, J, W = F.com_world(P66, T3, c)
fw = W[:, 0] @ np.array([0, 0, 1.0]); yaw_now = np.median(np.arctan2(fw[:, 1], fw[:, 0])); psi = np.pi - yaw_now
Rz = np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
h = np.load(F.SMPLH_M_PATH); vsh = h["v_template"].astype(np.float64) + h["shapedirs"].astype(np.float64)[:, :, :10] @ betas
phys = pickle.load(open(F.PHYSICS_PATH, "rb"), encoding="latin1")
fv = np.concatenate([np.asarray(phys["bodypart_vertexid"][b]) for b in (7, 10)]); fv = fv[fv < 6890]
h_foot = float(c["centroid_19"][6][1] - vsh[fv, 1].min())
Pp = com @ Rz.T; pel = np.median(Pp[:, 0, :2], 0); fz = np.median(Pp[:, [6, 7], 2])
t = np.array([0.9 - pel[0], -pel[1], -0.9 + h_foot - fz])
json.dump(dict(R=Rz.tolist(), t=t.tolist(), psi_deg=float(np.degrees(psi)), facing_yaw_node_world_deg=float(np.degrees(yaw_now)), floor_z=-0.9,
               h_foot=h_foot, betas=betas.tolist(), source=sys.argv[1], first_frames=NF, rule="p_isaac = R @ p_nodeworld + t ; v_isaac = R @ v_nodeworld"),
          open(f"{D}/temp_transform.json", "w"), indent=1)
# 메시 변환
smpl = pickle.load(open(F.SMPL_MODEL_PATH, "rb"), encoding="latin1")
REG = np.asarray(smpl["V_regressor"].todense() if hasattr(smpl["V_regressor"], "todense") else smpl["V_regressor"], np.float64)
j0a = REG[0] @ h["v_template"].astype(np.float64); j0B = np.einsum("v,vck->ck", REG[0], h["shapedirs"].astype(np.float64)[:, :, :10])
C = F.cam_R(); A = C @ Rz.T; b_rigid = F.CAM_T @ Rz.T + t
json.dump(dict(A=A.tolist(), b_rigid=b_rigid.tolist(), cam_R=C.tolist(), R=Rz.tolist(), j0_a=j0a.tolist(), j0_B=j0B.tolist(),
               rule="p = v_cam @ A + b; b_node = b_rigid + (j0_a + j0_B·betas − cam_Rᵀ (smpl_joint[0:3] − global_trans)) @ Rᵀ"),
          open(f"{D}/mesh_transform.json", "w"), indent=1)
X = Pp + t
print(f"[tf5] betas {np.round(betas, 3).tolist()}, 첫 {NF} 프레임, ψ {np.degrees(psi):.1f}° (정면 yaw {np.degrees(yaw_now):.1f}°), t {np.round(t, 4).tolist()}, h_foot {h_foot:.4f}")
print(f"[tf5] J0_노드(betas) {np.round(j0a + j0B @ betas, 4).tolist()} == rest_constants 골반 {np.round(c['rest_joints_19'][0], 4).tolist()}")
# 전 기록에 같은 변환 적용 시 범위
comA, _, _ = F.com_world(Rr["pose"].astype(np.float64), Rr["trans"].astype(np.float64), c); XA = comA @ Rz.T + t
print(f"[tf5] 전 기록 {len(XA)}프레임: 골반 COM x {XA[:, 0, 0].min():.3f}~{XA[:, 0, 0].max():.3f}, y {XA[:, 0, 1].min():.3f}~{XA[:, 0, 1].max():.3f}, "
      f"발 COM z {XA[:, [6, 7], 2].min():.3f}~{XA[:, [6, 7], 2].max():.3f} (바닥 −0.9 + h_foot {h_foot:.3f})")
# 첫 메시지 메시 확인 (b_node)
fv0 = Rr["first_vertices"].astype(np.float64)
if len(fv0):
    j0m = Rr["joint0"][0].astype(np.float64) - Rr["trans"][0].astype(np.float64)
    bn = b_rigid + ((j0a + j0B @ betas) - C.T @ j0m) @ Rz.T
    V = fv0 @ A + bn; c0 = X[0] if NF > 0 else None
    print(f"[tf5] 첫 메시지 메시 (b_node): 정점 중심 {np.round(V.mean(0), 3).tolist()} ↔ 분절 COM 평균 {np.round(c0.mean(0), 3).tolist()}, "
          f"차 {np.linalg.norm(V.mean(0) - c0.mean(0)) * 1000:.0f} mm; 정점 z 최저 {V[:, 2].min():.3f}; b_node − b_rigid 크기 {np.linalg.norm(bn - b_rigid) * 1000:.0f} mm")
