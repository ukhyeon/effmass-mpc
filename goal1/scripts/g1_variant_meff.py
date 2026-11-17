"""g1_variant_meff.py — fr3 / fr3v2 / fr3v2_1 URDF 링크 질량·관성 차이가 로봇 유효질량에 주는 영향 (numpy, 시스템 python3 -B, 새 설치 없음)
  M(q) = Σ_links m Jvᵀ Jv + Jωᵀ (R I Rᵀ) Jω (링크 COM 기준, 7 회전 관절, 손가락 0.04 고정·손에 붙은 강체로 취급)
  m_r(u) = 1 / (uᵀ Jp M⁻¹ Jpᵀ u), Jp = 점의 선형 야코비안. 점: fr3_hand_tcp 원점, fr3_link4 원점(팔꿈치)
  자세 6개(retract 포함, 좁은 한계 안 무작위), 방향 ±x ±y ±z 와 무작위 20개"""
import json, os, numpy as np
import xml.etree.ElementTree as ET
V = os.path.expanduser("~/effmass_mpc/goal1/fr3/variants")
def rpy2R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr], [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], [-sp, cp*sr, cp*cr]])
def T(xyz, rpy):
    A = np.eye(4); A[:3, :3] = rpy2R(*rpy); A[:3, 3] = xyz; return A
def load(m):
    r = ET.parse(f"{V}/{m}.urdf").getroot(); P = m + "_"
    joints = {}
    for j in r.findall("joint"):
        o = j.find("origin"); ax = j.find("axis")
        joints[j.find("child").get("link")] = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
            T=T([float(x) for x in o.get("xyz", "0 0 0").split()], [float(x) for x in o.get("rpy", "0 0 0").split()]) if o is not None else np.eye(4),
            axis=np.array([float(x) for x in ax.get("xyz").split()]) if ax is not None else np.array([0, 0, 1.0]))
    inert = {}
    for l in r.findall("link"):
        i = l.find("inertial")
        if i is None: continue
        o = i.find("origin"); I = i.find("inertia"); g = lambda k: float(I.get(k))
        inert[l.get("name")] = dict(m=float(i.find("mass").get("value")), T=T([float(x) for x in o.get("xyz", "0 0 0").split()], [float(x) for x in o.get("rpy", "0 0 0").split()]) if o is not None else np.eye(4),
                                    I=np.array([[g("ixx"), g("ixy"), g("ixz")], [g("ixy"), g("iyy"), g("iyz")], [g("ixz"), g("iyz"), g("izz")]]))
    return joints, inert, P
def fk(model, q):
    joints, inert, P = model; W = {"base": np.eye(4)}; jinfo = {}
    order = list(joints.keys()); done = True
    while len(W) < len(joints) + 1:
        for c, j in joints.items():
            if c in W or j["parent"] not in W: continue
            A = W[j["parent"]] @ j["T"]
            if j["type"] == "revolute":
                k = int(j["name"][-1]) - 1
                ax = j["axis"]; th = q[k]; K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                Rj = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K; B = np.eye(4); B[:3, :3] = Rj
                jinfo[k] = (A[:3, 3].copy(), A[:3, :3] @ ax); A = A @ B
            elif j["type"] == "prismatic":
                B = np.eye(4); B[:3, 3] = j["axis"] * 0.04; A = A @ B
            W[c] = A
    return W, jinfo
def chain(model, link):
    joints = model[0]; s = set(); c = link
    while c in joints:
        j = joints[c]
        if j["type"] == "revolute": s.add(int(j["name"][-1]) - 1)
        c = j["parent"]
    return s
def jac(jinfo, dep, p):
    Jv = np.zeros((3, 7)); Jw = np.zeros((3, 7))
    for k in dep:
        o, z = jinfo[k]; Jv[:, k] = np.cross(z, p - o); Jw[:, k] = z
    return Jv, Jw
def mass_matrix(model, q):
    W, ji = fk(model, q); M = np.zeros((7, 7))
    for l, d in model[1].items():
        if l not in W: continue
        A = W[l] @ d["T"]; dep = chain(model, l)
        Jv, Jw = jac(ji, dep, A[:3, 3]); R = A[:3, :3]
        M += d["m"] * Jv.T @ Jv + Jw.T @ (R @ d["I"] @ R.T) @ Jw
    return M, W, ji
models = {m: load(m) for m in ["fr3", "fr3v2", "fr3v2_1"]}
rng = np.random.default_rng(0)
lo = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]); hi = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
Q = [np.array([0, -1.3, 0, -2.5, 0, 1.0, 0])] + [lo + (hi - lo) * rng.uniform(0.15, 0.85, 7) for _ in range(5)]
U = [np.array(v, float) for v in [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]] + [v / np.linalg.norm(v) for v in rng.normal(size=(20, 3))]
out = {}
for pt in ["hand_tcp", "link4"]:
    vals = {m: [] for m in models}
    for q in Q:
        for m, mod in models.items():
            M, W, ji = mass_matrix(mod, q); link = mod[2] + pt
            Jv, _ = jac(ji, chain(mod, link), W[link][:3, 3]); Li = Jv @ np.linalg.solve(M, Jv.T)
            vals[m].append([1.0 / (u @ Li @ u) for u in U])
    A = {m: np.array(v) for m, v in vals.items()}
    rel = {m: (A[m] / A["fr3"] - 1) * 100 for m in models if m != "fr3"}
    out[pt] = dict(fr3_kg=dict(min=float(A["fr3"].min()), median=float(np.median(A["fr3"])), max=float(A["fr3"].max())),
                   **{f"{m}_vs_fr3_pct": dict(mean=float(r.mean()), min=float(r.min()), max=float(r.max())) for m, r in rel.items()})
    print(f"[meff] 점 {pt}: fr3 m_r 범위 {A['fr3'].min():.2f}~{A['fr3'].max():.2f} kg (중앙 {np.median(A['fr3']):.2f}); " +
          "; ".join(f"{m} − fr3 상대차 평균 {r.mean():+.1f}% (범위 {r.min():+.1f}~{r.max():+.1f}%)" for m, r in rel.items()))
    print(f"[meff]   retract, +x 방향: " + ", ".join(f"{m} {A[m][0][0]:.2f} kg" for m in models))
for m, mod in models.items():
    tot = sum(d["m"] for l, d in mod[1].items()); arm = sum(d["m"] for l, d in mod[1].items() if any(l.endswith(f"link{i}") for i in range(8)))
    print(f"[mass] {m}: 전체 {tot:.3f} kg, link0~7 {arm:.3f} kg")
json.dump(out, open(os.path.expanduser("~/effmass_mpc/goal1/logs/g1_variant_meff.json"), "w"), indent=1)
