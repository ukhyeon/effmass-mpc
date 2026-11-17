"""step4_5_seg_analysis.py — 4-5 추가 분석 (새 측정 없음): 노드 분절 COM(구) ↔ 같은 프레임 메시의 분절 무게중심 (physics.pkl 분할)

모든 위치는 node world (노드 관례 배치). 벡터 표는 Isaac 좌표 (임시 변환 R 적용, 사람은 로봇 쪽 −x 방향을 봄).
정의 (모두 노드와 같은 분절 무게중심 정의: physics.pkl 부위별 [정점+관절] 평균, 19분절 병합은 부위 중심의 단순 평균):
  N   노드 COM = step4_ref_fk.com_world (노드와 0.001 mm 일치, 4-5 2회차 수신값과도 비교)
  X1  노드 19분절 트리 + 노드가 버리는 관절(척추3 = SMPL 9, 왼발 = SMPL 10, 트리에서 빠지는 관절) 그대로, 오일러 왕복 대신 축-각 회전 직접 사용
  X2  SMPL 24관절 전체 트리로 강체 FK (모든 관절 회전 사용), 같은 모델(SMPL-H male)·같은 정지 오프셋
  H   SMPL-H male 메시 (LBS, 손가락 0) 의 분절 무게중심, 노드 관례 배치
  S   표시한 SMPL neutral 메시 (smplx, 재생과 같은 계산) 의 분절 무게중심, 노드 관례 배치 (= GUI 메시)
  차이 분해: N − S = (N − X1) [3a 오일러 왕복] + (X1 − X2) [3b 버리는 관절] + (X2 − H) [2 강체 COM vs LBS 메시] + (H − S) [1 몸 모델]
"""
import json, os, pickle, sys
import numpy as np, torch, smplx
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step4_ref_fk as F
D = os.path.expanduser("~/effmass_mpc/env_check")
c = F.rest_constants(np.zeros(10)); C = F.cam_R()
tf = json.load(open(f"{D}/step4/temp_transform.json")); RI = np.array(tf["R"])
z = np.load(tf["npz"]); P = z["poses"].astype(np.float64); T = z["trans"].astype(np.float64); N = len(P)
phys = pickle.load(open(F.PHYSICS_PATH, "rb"), encoding="latin1"); PID = [np.asarray(phys["bodypart_vertexid"][b]) for b in range(24)]
smpl = pickle.load(open(F.SMPL_MODEL_PATH, "rb"), encoding="latin1")
REG = np.asarray(smpl["V_regressor"].todense() if hasattr(smpl["V_regressor"], "todense") else smpl["V_regressor"], np.float64)
h = np.load(F.SMPLH_M_PATH)
PAR24 = [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21]

def cent19(V):
    """V (n,6890,3) → 19분절 무게중심 (노드 정의) 과 손 '보이는 위치'(부위 20+22 정점 합집합 평균)"""
    vall = np.concatenate([V, np.einsum("jv,nvc->njc", REG, V)], 1)
    c24 = np.stack([vall[:, PID[b]].mean(1) for b in range(24)], 1)
    out = np.zeros((len(V), 19, 3))
    for n, o in F.KEEP_19.items(): out[:, n] = c24[:, o]
    for n, ol in F.MERGE_19.items(): out[:, n] = c24[:, ol].mean(1)
    vis = {s: vall[:, np.unique(np.concatenate([PID[a] for a in F.MERGE_19[s]]))].mean(1) for s in (17, 18)}
    return out, vis

# ---- N: 노드 COM ----
NC, J19, W19 = F.com_world(P, T, c)
tw = T @ C + F.CAM_T
# ---- 축-각 직접 회전 ----
R22 = F.rodrigues(P.reshape(N, 22, 3)); R22w = R22.copy(); R22w[:, 0] = C.T @ R22[:, 0]
Rl24 = np.tile(np.eye(3), (N, 24, 1, 1)); Rl24[:, :22] = R22w
def fk19(Rl):
    W = np.zeros((N, 19, 3, 3))
    for j in F.TOPO_19:
        p = F.PARENT_19[j]; W[:, j] = Rl[:, F.IDX_24_TO_19[j]] if p < 0 else W[:, p] @ Rl[:, F.IDX_24_TO_19[j]]
    J = np.zeros((N, 19, 3)); J[:, 0] = c["rest_joints_19"][0]
    for j in F.TOPO_19:
        p = F.PARENT_19[j]
        if p >= 0: J[:, j] = J[:, p] + np.einsum("nij,j->ni", W[:, p], c["rest_joint_offsets_19"][j])
    J += tw[:, None]
    return J + np.einsum("nbij,bj->nbi", W, c["com_local_offsets_19"]), J, W
RlX1 = Rl24.copy(); RlX1[:, 9] = np.eye(3); RlX1[:, 10] = np.eye(3)     # 노드처럼 척추3·왼발 회전 0
X1, _, _ = fk19(RlX1)
# X2: 24관절 전체 트리 강체 FK
rest24 = REG @ (h["v_template"].astype(np.float64))
W24 = np.zeros((N, 24, 3, 3)); J24 = np.zeros((N, 24, 3))
for j in range(24):
    p = PAR24[j]
    W24[:, j] = Rl24[:, j] if p < 0 else W24[:, p] @ Rl24[:, j]
    J24[:, j] = rest24[0] + tw if p < 0 else J24[:, p] + np.einsum("nij,j->ni", W24[:, p], rest24[j] - rest24[p])
X2 = np.stack([J24[:, F.IDX_24_TO_19[s]] + np.einsum("nij,j->ni", W24[:, F.IDX_24_TO_19[s]], c["com_local_offsets_19"][s]) for s in range(19)], 1)
# ---- H: SMPL-H male LBS (손가락 0, betas 0), 카메라 → node world 노드 관례 ----
vt = h["v_template"].astype(np.float64); JR = h["J_regressor"].astype(np.float64); Jh = JR @ vt
kt = h["kintree_table"][0].astype(int); kt[0] = -1; Wt = h["weights"].astype(np.float64); PD = h["posedirs"].astype(np.float64).reshape(-1, 459)
offH = c["rest_joints_19"][0] - C.T @ Jh[0]
H = np.zeros((N, 19, 3)); visH = {17: np.zeros((N, 3)), 18: np.zeros((N, 3))}
for s0 in range(0, N, 100):
    s1 = min(N, s0 + 100); n = s1 - s0
    Rf = np.tile(np.eye(3), (n, 52, 1, 1)); Rf[:, :22] = R22[s0:s1]
    vp = vt + (PD @ (Rf[:, 1:] - np.eye(3)).reshape(n, -1).T).T.reshape(n, 6890, 3)
    G = np.zeros((n, 52, 4, 4)); G[:, :, 3, 3] = 1
    for j in range(52):
        p = kt[j]; L = np.zeros((n, 4, 4)); L[:, :3, :3] = Rf[:, j]; L[:, :3, 3] = Jh[j] - (0 if p < 0 else Jh[p]); L[:, 3, 3] = 1
        G[:, j] = L if p < 0 else G[:, p] @ L
    A = G.copy(); A[:, :, :3, 3] -= np.einsum("njab,jb->nja", G[:, :, :3, :3], Jh)
    Tv = np.einsum("vj,njab->nvab", Wt, A)
    V = np.einsum("nvab,nvb->nva", Tv[..., :3, :3], vp) + Tv[..., :3, 3] + T[s0:s1, None]
    Vw = V @ C + F.CAM_T + offH
    cc, vis = cent19(Vw); H[s0:s1] = cc
    for k in (17, 18): visH[k][s0:s1] = vis[k]
# ---- S: SMPL neutral (smplx, 재생과 같음) ----
m = smplx.create(os.path.expanduser("~/realtime_smpl/pykinect/body_models"), model_type="smpl", gender="neutral", batch_size=1, create_transl=False)
with torch.no_grad(): J0s = m(betas=torch.zeros(1, 10)).joints[0, 0].numpy().astype(np.float64); Js_rest = m(betas=torch.zeros(1, 10)).joints[0, :24].numpy().astype(np.float64)
offS = c["rest_joints_19"][0] - C.T @ J0s
S = np.zeros((N, 19, 3)); visS = {17: np.zeros((N, 3)), 18: np.zeros((N, 3))}
P32 = P.astype(np.float32); T32 = T.astype(np.float32)
with torch.no_grad():
    for s0 in range(0, N, 64):
        s1 = min(N, s0 + 64); n = s1 - s0
        o = m(betas=torch.zeros(n, 10), body_pose=torch.from_numpy(np.concatenate([P32[s0:s1, 3:], np.zeros((n, 6), np.float32)], 1)),
              global_orient=torch.from_numpy(P32[s0:s1, :3]), transl=torch.from_numpy(T32[s0:s1]))
        Vw = o.vertices.numpy().astype(np.float64) @ C + F.CAM_T + offS
        cc, vis = cent19(Vw); S[s0:s1] = cc
        for k in (17, 18): visS[k][s0:s1] = vis[k]
# ---- 기록된 구 위치와 N 비교 ----
R2 = np.load(f"{D}/step4/step4_5_run2_human_rec.npz"); dp = R2["dyn_pos"].astype(np.float64)
Dm = np.linalg.norm(dp[:, None] - NC[None], axis=3).max(2); fi = Dm.argmin(1); ex = Dm[np.arange(len(fi)), fi] < 1e-5
print(f"[seg] 4-5 2회차 수신 구 위치 {len(dp)}개 중 {ex.sum()}개가 N 과 0.01 mm 이내 (최대 {Dm[np.arange(len(fi)), fi][ex].max() * 1000:.4f} mm) → N 을 구 위치로 사용, 프레임 {N}개 전부")
iso = lambda v: v @ RI.T
res = {}
def stats(d):
    mag = np.linalg.norm(d, axis=-1)
    return mag.mean(0), mag.max(0), iso(d.mean(0))
tot = NC - S
comp = {"3a 오일러 왕복 (N−X1)": NC - X1, "3b 버리는 관절 (X1−X2)": X1 - X2, "2 강체 COM vs LBS 메시 (X2−H)": X2 - H, "1 몸 모델 (H−S)": H - S}
print(f"[seg] 분해 합 검산: |N−S − Σ| 최대 {np.abs(tot - sum(comp.values())).max():.2e} m")
mean_all = iso(tot.mean((0, 1)))
print(f"[seg] (4) 공통: 19분절·전 프레임 평균 차 벡터 (Isaac) {np.round(mean_all * 1000, 1)} mm, 크기 {np.linalg.norm(mean_all) * 1000:.1f} mm")
print(f"[seg] (4) 메시에 더한 노드 관례 오프셋 (node world): neutral {np.round(offS * 1000, 1)} mm |{np.linalg.norm(offS) * 1000:.1f}|, "
      f"그중 관례 (I−cam_Rᵀ)J0_노드 {np.round((c['rest_joints_19'][0] - C.T @ c['rest_joints_19'][0]) * 1000, 1)} |{np.linalg.norm(c['rest_joints_19'][0] - C.T @ c['rest_joints_19'][0]) * 1000:.1f}| mm, "
      f"Isaac 좌표 {np.round(iso(offS) * 1000, 1)} mm")
# 축 방향 성분: 분절 관절 → 노드 COM 방향
ax = NC - J19; axn = ax / np.maximum(np.linalg.norm(ax, axis=-1, keepdims=True), 1e-9)
par = (tot * axn).sum(-1); perp = np.linalg.norm(tot - par[..., None] * axn, axis=-1)
mm = lambda x: x * 1000
print("\n| 분절 | 차 크기 평균 / 최대 (mm) | 평균 차 벡터 Isaac (x, y, z) mm | 관절→COM 축 방향 성분 평균 / 수직 성분 평균 (mm) | " + " | ".join(k + " 평균 크기 (mm)" for k in comp) + " |")
print("|---" * (5 + len(comp)) + "|")
order = [0, 3, 8, 9, 12, 10, 11, 13, 14, 15, 16, 17, 18, 1, 2, 4, 5, 6, 7]
for s in order:
    a, b, v = stats(tot[:, s])
    cs = [np.linalg.norm(d[:, s], axis=-1).mean() for d in comp.values()]
    print(f"| {F.NAMES_19[s]} | {mm(a):.1f} / {mm(b):.1f} | ({mm(v[0]):+.1f}, {mm(v[1]):+.1f}, {mm(v[2]):+.1f}) | {mm(par[:, s].mean()):+.1f} / {mm(perp[:, s].mean()):.1f} | "
          + " | ".join(f"{mm(x):.1f}" for x in cs) + " |")
    res[F.NAMES_19[s]] = dict(mean_mm=mm(a), max_mm=mm(b), vec_isaac_mm=(v * 1000).tolist(), axial_mm=mm(par[:, s].mean()), perp_mm=mm(perp[:, s].mean()),
                              comp_mm={k: mm(x) for k, x in zip(comp, cs)})
print("\n(2) 손 무게중심 정의 vs 보이는 위치 (부위 20·22 중심의 단순 평균 vs 두 부위 정점 합집합 평균), mm:")
for s in (17, 18):
    dH = np.linalg.norm(H[:, s] - visH[s], axis=-1); dS = np.linalg.norm(S[:, s] - visS[s], axis=-1); dN = np.linalg.norm(NC[:, s] - visS[s], axis=-1)
    print(f"  {F.NAMES_19[s]}: SMPL-H 메시 평균 {mm(dH.mean()):.1f} 최대 {mm(dH.max()):.1f} / SMPL neutral 메시 평균 {mm(dS.mean()):.1f} 최대 {mm(dS.max()):.1f}; "
          f"구(노드) ↔ 표시 메시 손 정점 평균: 평균 {mm(dN.mean()):.1f} 최대 {mm(dN.max()):.1f}, 평균 벡터 Isaac {np.round(iso((NC[:, s] - visS[s]).mean(0)) * 1000, 1)}")
    res[F.NAMES_19[s]]["visible_vs_node_mm"] = dict(mean=mm(dN.mean()), max=mm(dN.max()))
print("\n(1) 몸 모델 정지 자세 (betas 0): 노드 관절 = SMPL neutral V_regressor × SMPL-H male 정점, 메시 관절 = smplx SMPL neutral")
nm = {0: "골반", 16: "왼어깨", 17: "오른어깨", 18: "왼팔꿈치", 19: "오른팔꿈치", 20: "왼손목", 21: "오른손목"}
for j, nmj in nm.items():
    print(f"  {nmj}(SMPL {j}): 노드 {np.round(rest24[j] * 1000, 1)} 메시 {np.round(Js_rest[j] * 1000, 1)} 차 {np.linalg.norm(rest24[j] - Js_rest[j]) * 1000:.1f} mm")
for side, (sh, el, wr) in {"왼": (16, 18, 20), "오른": (17, 19, 21)}.items():
    Lu = lambda J: np.linalg.norm(J[el] - J[sh]) * 1000; Lf = lambda J: np.linalg.norm(J[wr] - J[el]) * 1000
    print(f"  {side}팔 위팔 길이 노드 {Lu(rest24):.1f} / 메시 {Lu(Js_rest):.1f} mm, 아래팔 노드 {Lf(rest24):.1f} / 메시 {Lf(Js_rest):.1f} mm, "
          f"어깨→손목 노드 {(np.linalg.norm(rest24[wr] - rest24[sh])) * 1000:.1f} / 메시 {np.linalg.norm(Js_rest[wr] - Js_rest[sh]) * 1000:.1f} mm")
json.dump(res, open(f"{D}/step4/step4_5_seg_analysis.json", "w"), indent=1, default=float)
