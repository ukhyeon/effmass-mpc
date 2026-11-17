"""step4_mesh_prep.py — 4-5 준비 (시스템 Python, ~/.local smplx 읽기만, 모델 파일은 복사하지 않음)
  1) SMPL neutral 면 (13776,3) 을 step4/smpl_neutral_faces.npy 로 내보냄 (모델 파생 데이터, 4-0 결정 5)
  2) 메시 변환: HumanPose.smpl_vertices 는 카메라 좌표 (sensing_node 1130행 "카메라 기준") →
       node world: v_w = cam_Rᵀ v + t_cam (노드와 같은 카메라 상수, step4_ref_fk) → Isaac: p = R v_w + t (temp_transform.json)
     행 벡터 표기 p = v @ A + b, A = cam_R @ Rᵀ, b = t_cam @ Rᵀ + t  → step4/mesh_transform.json
  자세: 저장 66개(SMPL-H 몸 22관절) + SMPL 손 관절 22·23 은 0
  3) 확인용: 재생 프레임 0 의 정점을 계산해 Isaac 좌표 최저 z (바닥 -0.9 기준) 와 정점 중심 ↔ 노드 분절 COM (독립 계산) 평균 거리 기록
"""
import json, os, sys, time
import numpy as np, torch, smplx
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step4_ref_fk as F
D = os.path.expanduser("~/effmass_mpc/env_check/step4")
m = smplx.create(os.path.expanduser("~/realtime_smpl/pykinect/body_models"), model_type="smpl", gender="neutral", batch_size=1, create_transl=False)
faces = np.asarray(m.faces, dtype=np.int32)
np.save(f"{D}/smpl_neutral_faces.npy", faces)
tf = json.load(open(f"{D}/temp_transform.json")); R = np.array(tf["R"]); t = np.array(tf["t"])
C = F.cam_R(); A = C @ R.T; b = F.CAM_T @ R.T + t
json.dump(dict(A=A.tolist(), b=b.tolist(), rule="p_isaac = v_cam @ A + b (행 벡터), A = cam_R @ R^T, b = t_cam @ R^T + t",
               cam_roll_deg=F.CAM_ROLL_DEG, cam_t=F.CAM_T.tolist(), from_transform="temp_transform.json"), open(f"{D}/mesh_transform.json", "w"), indent=1)
z = np.load(tf["npz"]); P = z["poses"].astype(np.float32); T = z["trans"].astype(np.float32)
with torch.no_grad():
    o = m(betas=torch.zeros(1, 10), body_pose=torch.from_numpy(np.concatenate([P[:1, 3:], np.zeros((1, 6), np.float32)], 1)), global_orient=torch.from_numpy(P[:1, :3]), transl=torch.from_numpy(T[:1]))
V = o.vertices[0].numpy().astype(np.float64)
Pi = V @ A + b
com, _, _ = F.com_world(P[:1], T[:1], F.rest_constants(np.zeros(10)))
ci = com[0] @ R.T + t
print(f"[prep] 정점 {V.shape}, 면 {faces.shape} {faces.dtype} → {D}/smpl_neutral_faces.npy, 변환 → {D}/mesh_transform.json")
print(f"[prep] 프레임 0: Isaac 정점 z 최저 {Pi[:, 2].min():.4f} (바닥 -0.9), 최고 {Pi[:, 2].max():.4f}; 정점 중심 {Pi.mean(0).round(4)} ↔ 분절 COM 평균 {ci.mean(0).round(4)}, "
      f"차 {np.linalg.norm(Pi.mean(0) - ci.mean(0)) * 1000:.1f} mm; 정점별 최근접 분절 COM 거리 중앙 {np.median(np.linalg.norm(Pi[:, None] - ci[None], axis=2).min(1)) * 1000:.1f} mm")
