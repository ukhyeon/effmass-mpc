"""step4_ref_fk.py — human_dynamics_node 의 19분절 COM 위치를 numpy 로 독립 계산 (torch·노드 코드 import 없음)

근거: 설치본 ~/hrc_ws/install/hrc_body_tracking/lib/python3.10/site-packages/hrc_body_tracking/
  human_dynamics_node.py  aa2euler(), timer_callback(), rigid_fk_19(), mass_inertia()
  utils.py                batch_rodrigues(), rotmat2euler{yzx,zyx,zxy}(), batch_euler2mat{yzx,zyx,zxy}(), batch_roteulerSMPL()
  physcis.py              SMPLH.forward(), Physics.compute_massinertia(), merge_parts_24_to_19_vertex_avg(),
                          get_smpl19_kinematic_tree(), topo_order_from_parent_array(), compute_Jr_19_smpl()
  constants.py            G2Hpose_idx
읽는 모델 파일 (읽기만): SMPL-H male.npz, SMPL neutral.pkl(V_regressor), physics.pkl(분할·질량)

계산은 float64. 노드는 float32(GPU) 이므로 차이는 1e-5 m 수준 이하로 예상.
"""
import pickle

import numpy as np

SMPLH_M_PATH = "/home/robotics/hrc_ws/src/body_models/smplh/male.npz"
SMPL_MODEL_PATH = "/home/robotics/python/physpt/assets/data/smpl/neutral.pkl"
PHYSICS_PATH = "/home/robotics/hrc_ws/src/hrc_body_tracking/server/physics.pkl"

CAM_ROLL_DEG, CAM_T = 110.8, np.array([-0.80, -2.3, 1.55])
IDX_24_TO_19 = [0, 1, 2, 3, 4, 5, 7, 8, 6, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]
PARENT_19 = [-1, 0, 0, 0, 1, 2, 4, 5, 3, 8, 8, 8, 9, 10, 11, 13, 14, 15, 16]
ORDER_24 = ["yzx"] + ["zyx"] * 15 + ["zxy"] * 2 + ["yzx"] * 2 + ["zyx"] * 4
G2HPOSE_IDX = list(range(0, 27)) + list(range(33, 66))
KEEP_19 = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 9: 12, 10: 13, 11: 14, 12: 15, 13: 16, 14: 17, 15: 18, 16: 19}
MERGE_19 = {8: [6, 9], 6: [7, 10], 7: [8, 11], 17: [20, 22], 18: [21, 23]}
NAMES_19 = ["pelvis", "left_thigh", "right_thigh", "abdomen", "left_calf", "right_calf", "left_foot", "right_foot",
            "chest", "neck", "left_shoulder", "right_shoulder", "head", "left_upper_arm", "right_upper_arm",
            "left_lower_arm", "right_lower_arm", "left_hand", "right_hand"]


def topo_order(parent):
    ch = [[] for _ in parent]
    for i, p in enumerate(parent):
        if p >= 0:
            ch[p].append(i)
    q = [i for i, p in enumerate(parent) if p == -1]
    out = []
    while q:
        u = q.pop(0); out.append(u); q += ch[u]
    return out


TOPO_19 = topo_order(PARENT_19)


def _rx(a):
    c, s = np.cos(a), np.sin(a); o, z = np.ones_like(a), np.zeros_like(a)
    return np.stack([np.stack([o, z, z], -1), np.stack([z, c, -s], -1), np.stack([z, s, c], -1)], -2)


def _ry(a):
    c, s = np.cos(a), np.sin(a); o, z = np.ones_like(a), np.zeros_like(a)
    return np.stack([np.stack([c, z, s], -1), np.stack([z, o, z], -1), np.stack([-s, z, c], -1)], -2)


def _rz(a):
    c, s = np.cos(a), np.sin(a); o, z = np.ones_like(a), np.zeros_like(a)
    return np.stack([np.stack([c, -s, z], -1), np.stack([s, c, z], -1), np.stack([z, z, o], -1)], -2)


def cam_R():
    """utils.camera_to_world_rotmat(roll=110.8, 0, 0) = Rx(roll) @ Ry(0) @ Rz(0)"""
    return _rx(np.deg2rad(CAM_ROLL_DEG))


def rodrigues(theta):
    """utils.batch_rodrigues: angle = ||theta + 1e-8||, r = theta/angle, R = cos I + (1-cos) r rᵀ + sin [r]x"""
    angle = np.linalg.norm(theta + 1e-8, axis=-1, keepdims=True)
    r = theta / angle
    c, s = np.cos(angle)[..., None], np.sin(angle)[..., None]
    K2 = r[..., :, None] * r[..., None, :]
    z = np.zeros(r.shape[:-1])
    Kx = np.stack([np.stack([z, -r[..., 2], r[..., 1]], -1), np.stack([r[..., 2], z, -r[..., 0]], -1),
                   np.stack([-r[..., 1], r[..., 0], z], -1)], -2)
    return c * np.eye(3) + (1 - c) * K2 + s * Kx


def mat2euler(R, order):
    """utils.rotmat2euler{zyx,yzx,zxy} (임계값 1e-8 분기 포함). 반환 [x, y, z]"""
    r11, r12, r13 = R[..., 0, 0], R[..., 0, 1], R[..., 0, 2]
    r21, r22, r23 = R[..., 1, 0], R[..., 1, 1], R[..., 1, 2]
    r32, r33 = R[..., 2, 1], R[..., 2, 2]
    e = np.zeros(R.shape[:-2] + (3,))
    if order == "zyx":
        cy = np.sqrt(r33 * r33 + r23 * r23); e[..., 1] = np.arctan2(r13, cy); m = cy > 1e-8
        e[..., 2] = np.where(m, np.arctan2(-r12, r11), np.arctan2(r21, r22)); e[..., 0] = np.where(m, np.arctan2(-r23, r33), 0)
    elif order == "yzx":
        cz = np.sqrt(r11 * r11 + r13 * r13); e[..., 2] = np.arctan2(-r12, cz); m = cz > 1e-8
        e[..., 1] = np.where(m, np.arctan2(r13, r11), 0); e[..., 0] = np.where(m, np.arctan2(r32, r22), np.arctan2(-r23, r33))
    elif order == "zxy":
        cx = np.sqrt(r21 * r21 + r22 * r22); e[..., 0] = np.arctan2(-r23, cx); m = cx > 1e-8
        e[..., 2] = np.where(m, np.arctan2(r21, r22), np.arctan2(-r12, r11)); e[..., 1] = np.where(m, np.arctan2(r13, r33), 0)
    return e


def euler2individual(e, order):
    """batch_euler2mat{yzx,zyx,zxy} 의 rotMat_individual [S0, S1, S2] (R_local = S2 @ S1 @ S0)"""
    x, y, z = e[..., 0], e[..., 1], e[..., 2]
    X, Y, Z = _rx(x), _ry(y), _rz(z)
    return {"yzx": (Y, Z, X), "zyx": (Z, Y, X), "zxy": (Z, X, Y)}[order]


# ---------------- 정지 자세 상수 (i): 모델 파일에서 numpy 로 ----------------
def rest_constants(betas):
    betas = np.asarray(betas, dtype=np.float64).reshape(10)
    h = np.load(SMPLH_M_PATH)
    v_template = h["v_template"].astype(np.float64)
    shapedirs = h["shapedirs"].astype(np.float64)[:, :, :10]           # (6890,3,10)
    smpl = pickle.load(open(SMPL_MODEL_PATH, "rb"), encoding="latin1")
    reg = np.asarray(smpl["V_regressor"].todense() if hasattr(smpl["V_regressor"], "todense") else smpl["V_regressor"],
                     dtype=np.float64)                                    # (24,6890)
    v_shaped = v_template + shapedirs @ betas                            # SMPLH.forward: v_template + betas·shapedirs
    joints24 = reg @ v_shaped                                            # joint_smpl_shaped (= joints_smpl at zero pose)
    phys = pickle.load(open(PHYSICS_PATH, "rb"), encoding="latin1")
    vall = np.concatenate([v_shaped, joints24], 0)                       # compute_massinertia: [v_shaped, joints_smpl]
    cen24 = np.stack([vall[np.asarray(phys["bodypart_vertexid"][b])].mean(0) for b in range(24)])
    # 질량 (위치 계산에는 쓰이지 않지만 같은 식으로 계산해 둠)
    mass24 = np.zeros(24); vol24 = np.zeros(24)
    for b in range(24):
        tri_idx = np.concatenate(phys["bodypart_faceid"][b])
        tri = (vall[tri_idx] - cen24[b]).reshape(-1, 3, 3)
        vol24[b] = (np.abs(np.linalg.det(tri)) / 6).sum()
        mass24[b] = (vol24[b] / phys["volumn"][b] - 1) * phys["mass"][b] * phys["mass_scaling"][b] + phys["mass"][b]
    cen19 = np.zeros((19, 3)); mass24m = mass24.copy(); mass24m[3] += mass24[6] / 2; mass24m[9] -= mass24[6] / 2
    mass19 = np.zeros(19)
    for n, o in KEEP_19.items():
        cen19[n] = cen24[o]; mass19[n] = mass24m[o]
    for n, olds in MERGE_19.items():
        cen19[n] = cen24[olds].mean(0); mass19[n] = mass24m[olds].sum()
    rest19 = joints24[IDX_24_TO_19]
    off19 = np.zeros((19, 3))
    for j in TOPO_19:
        if PARENT_19[j] >= 0:
            off19[j] = rest19[j] - rest19[PARENT_19[j]]
    return dict(rest_joints_19=rest19, rest_joint_offsets_19=off19, com_local_offsets_19=cen19 - rest19,
                centroid_19=cen19, mass_19=mass19)


# ---------------- 자세 의존 부분 ----------------
def com_world(poses_cam, trans_cam, consts):
    """poses_cam (N,66) 카메라 축-각, trans_cam (N,3) → 19분절 COM (N,19,3), node world"""
    P = np.asarray(poses_cam, np.float64).reshape(-1, 22, 3); T = np.asarray(trans_cam, np.float64).reshape(-1, 3)
    N = P.shape[0]
    R = rodrigues(P)                                                     # (N,22,3,3)
    C = cam_R()
    R[:, 0] = C.T @ R[:, 0]                                              # 루트 회전만 world 로
    tw = T @ C + CAM_T                                                   # 행 벡터 표기: (cam_Rᵀ tᵀ)ᵀ = T @ cam_R, + t_cam
    # aa2euler: 관절별 순서로 오일러
    eul = np.zeros((N, 22, 3))
    for j in range(22):
        o = "yzx" if j == 0 else ("zyx" if j <= 15 else ("zxy" if j <= 17 else ("yzx" if j <= 19 else "zyx")))
        eul[:, j] = mat2euler(R[:, j], o)
    keep = [j for j in range(22) if j not in (9, 10)]
    e60 = np.remainder(eul[:, keep].reshape(N, 60), 2 * np.pi)
    e60[e60 > np.pi] -= 2 * np.pi
    full = np.zeros((N, 72)); full[:, G2HPOSE_IDX] = e60
    E = full.reshape(N, 24, 3)
    Rloc = np.zeros((N, 24, 3, 3))
    for j in range(24):
        S0, S1, S2 = euler2individual(E[:, j], ORDER_24[j])
        Rloc[:, j] = S2 @ S1 @ S0
    W = np.zeros((N, 19, 3, 3))
    for j in TOPO_19:
        p = PARENT_19[j]
        W[:, j] = Rloc[:, IDX_24_TO_19[j]] if p < 0 else W[:, p] @ Rloc[:, IDX_24_TO_19[j]]
    J19 = np.zeros((N, 19, 3)); J19[:, 0] = consts["rest_joints_19"][0]
    for j in TOPO_19:
        p = PARENT_19[j]
        if p >= 0:
            J19[:, j] = J19[:, p] + np.einsum("nij,j->ni", W[:, p], consts["rest_joint_offsets_19"][j])
    J19 += tw[:, None, :]
    return J19 + np.einsum("nbij,bj->nbi", W, consts["com_local_offsets_19"]), J19, W


if __name__ == "__main__":
    import json, sys
    c = rest_constants(np.zeros(10))
    print(json.dumps({k: np.round(v, 5).tolist() for k, v in c.items() if k in ("mass_19",)}))
    print("rest_joints_19[0]", c["rest_joints_19"][0], "com_local_offsets_19[12]", c["com_local_offsets_19"][12])
