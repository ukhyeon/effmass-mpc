"""g1_robot_validate.py — 1-2c 검증 (시스템 python3 + /opt/ros/humble Pinocchio 3.9.0; g1_run_human.sh py 로 실행)
 (1) Pinocchio 모듈 ↔ 독립 numpy 구현 (URDF XML 직접 파싱, 회전축 야코비안) : 구 중심 위치, 구별 3x3 역관성, 회전자 켬/끔
 (2) 회전자 켬/끔 m_r: 손 끝(fr3v2_1_hand_tcp), 팔꿈치(link4 원점), 링크별 구 중앙값
 (3) indy_ws robot_dynamics_node.cpp 정의 재현 (부모 관절 관성 lever 점, LOCAL_WORLD_ALIGNED, Jv_o − [R·lever]×Jω, M + diag(N²Jm), LDLT) ↔ 이 모듈 같은 점
 (4) 계산 시간: 구 전체, 한 상태
 (5) 손 끝 μ = m_r m_h/(m_r+m_h): m_h 는 사본 노드 실제 출력(logs/g1_live_nodrop/rec.npz) 의 Λ_s⁻¹, 같은 무작위 방향 u
"""
import json, os, time, xml.etree.ElementTree as ET
import numpy as np, pinocchio as pin
from effmass_robot.robot_inertia import RobotInertia, URDF, meff
G = os.path.expanduser("~/effmass_mpc/goal1"); rng = np.random.default_rng(1)
ri_on, ri_off = RobotInertia(rotor=True), RobotInertia(rotor=False)
S = len(ri_on.sphere_link)
lo = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]); hi = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
Q = [np.array([0, -1.3, 0, -2.5, 0, 1.0, 0])] + [lo + (hi - lo) * rng.uniform(0.05, 0.95, 7) for _ in range(200)]
res = dict(pinocchio=pin.__version__, pinocchio_file=pin.__file__, n_spheres=S, spheres_per_link={l: ri_on.sphere_link.count(l) for l in dict.fromkeys(ri_on.sphere_link)},
           gear_ratio=ri_on.gear_ratio.tolist(), motor_inertia=ri_on.motor_inertia.tolist(), rotor_diag=ri_on.rotor_diag.tolist(), n_states=len(Q))
# ---------- (1) numpy 독립 구현 ----------
def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr], [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], [-sp, cp*sr, cp*cr]])
def TT(o):
    A = np.eye(4)
    if o is not None:
        A[:3, :3] = rpy(*[float(x) for x in o.get("rpy", "0 0 0").split()]); A[:3, 3] = [float(x) for x in o.get("xyz", "0 0 0").split()]
    return A
root = ET.parse(URDF).getroot(); J = {}
for j in root.findall("joint"):
    ax = j.find("axis"); J[j.find("child").get("link")] = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"), T=TT(j.find("origin")),
                                                                axis=np.array([float(x) for x in ax.get("xyz").split()]) if ax is not None else np.zeros(3))
INR = {}
for l in root.findall("link"):
    i = l.find("inertial")
    if i is None: continue
    I = i.find("inertia"); g = lambda k: float(I.get(k))
    INR[l.get("name")] = (float(i.find("mass").get("value")), TT(i.find("origin")), np.array([[g("ixx"), g("ixy"), g("ixz")], [g("ixy"), g("iyy"), g("iyz")], [g("ixz"), g("iyz"), g("izz")]]))
ARM = [f"fr3v2_1_joint{i}" for i in range(1, 8)]
base = [l for l in {j["parent"] for j in J.values()} if l not in J][0]
def np_fk(q):
    W = {base: np.eye(4)}; ax = {}
    while len(W) < len(J) + 1:
        for c, j in J.items():
            if c in W or j["parent"] not in W: continue
            A = W[j["parent"]] @ j["T"]
            if j["name"] in ARM:
                k = ARM.index(j["name"]); a = j["axis"]; K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                B = np.eye(4); B[:3, :3] = np.eye(3) + np.sin(q[k]) * K + (1 - np.cos(q[k])) * K @ K; ax[k] = (A[:3, 3].copy(), A[:3, :3] @ a); A = A @ B
            elif j["type"] == "prismatic":
                B = np.eye(4); B[:3, 3] = j["axis"] * 0.04; A = A @ B
            W[c] = A
    return W, ax
def dep(link):
    s = []; c = link
    while c in J:
        if J[c]["name"] in ARM: s.append(ARM.index(J[c]["name"]))
        c = J[c]["parent"]
    return s
def np_jac(ax, d, p):
    Jv = np.zeros((3, 7)); Jw = np.zeros((3, 7))
    for k in d:
        o, z = ax[k]; Jv[:, k] = np.cross(z, p - o); Jw[:, k] = z
    return Jv, Jw
def np_all(q, rotor):
    W, ax = np_fk(q); M = np.zeros((7, 7))
    for l, (m, Ti, I) in INR.items():
        A = W[l] @ Ti; Jv, Jw = np_jac(ax, dep(l), A[:3, 3]); R = A[:3, :3]; M += m * Jv.T @ Jv + Jw.T @ R @ I @ R.T @ Jw
    if rotor: M = M + np.diag(ri_on.rotor_diag)
    Mi = np.linalg.inv(M); C = np.zeros((S, 3)); L = np.zeros((S, 3, 3))
    for i, (l, c) in enumerate(zip(ri_on.sphere_link, ri_on.sphere_local)):
        p = (W[l] @ np.r_[c, 1])[:3]; C[i] = p; Jv, _ = np_jac(ax, dep(l), p); L[i] = Jv @ Mi @ Jv.T
    return C, L, M, W
dpos = []; dL = []; dM = []
for q in Q[:50]:
    for rot, ri in ((True, ri_on), (False, ri_off)):
        C, L, M = ri.compute(q); Cn, Ln, Mn, _ = np_all(q, rot)
        dpos.append(np.abs(C - Cn).max()); dM.append(np.abs(M - Mn).max() / np.abs(Mn).max())
        nz = np.linalg.norm(Ln, axis=(1, 2)) > 0; dL.append((np.linalg.norm(L - Ln, axis=(1, 2))[nz] / np.linalg.norm(Ln, axis=(1, 2))[nz]).max())
        assert np.abs(L[~nz]).max() == 0.0
res["vs_numpy"] = dict(states=50, center_max_abs_m=float(max(dpos)), M_rel_max=float(max(dM)), inv_inertia_rel_max=float(max(dL)),
                       link0_spheres=int(sum(l == "fr3v2_1_link0" for l in ri_on.sphere_link)), link0_inv_inertia_all_zero=True)
# ---------- (2) 회전자 켬/끔 ----------
U = np.array([v / np.linalg.norm(v) for v in rng.normal(size=(30, 3))])
def pt(ri, q, frame):
    return ri.point_inv_inertia(q, frame)[1]
tab = {}
for name, frame in (("hand_tcp", "fr3v2_1_hand_tcp"), ("elbow_link4_origin", "fr3v2_1_link4")):
    on = np.array([[meff(pt(ri_on, q, frame), u) for u in U] for q in Q]); off = np.array([[meff(pt(ri_off, q, frame), u) for u in U] for q in Q])
    tab[name] = dict(off_median=float(np.median(off)), on_median=float(np.median(on)), off_p5=float(np.percentile(off, 5)), on_p5=float(np.percentile(on, 5)),
                     off_p95=float(np.percentile(off, 95)), on_p95=float(np.percentile(on, 95)), ratio_median=float(np.median(on / off)), ratio_min=float((on / off).min()))
per_link = {}
for l in dict.fromkeys(ri_on.sphere_link):
    if l == "fr3v2_1_link0": continue
    idx = [i for i, x in enumerate(ri_on.sphere_link) if x == l]; on = []; off = []
    for q in Q[::4]:
        Lon = ri_on.compute(q)[1]; Loff = ri_off.compute(q)[1]
        for i in idx:
            for u in U[:10]: on.append(meff(Lon[i], u)); off.append(meff(Loff[i], u))
    on, off = np.array(on), np.array(off); fin = np.isfinite(on) & np.isfinite(off)
    per_link[l] = dict(n=int(len(on)), n_infinite=int((~fin).sum()), off_median=float(np.median(off[fin])), on_median=float(np.median(on[fin])), ratio_median=float(np.median(on[fin] / off[fin])))
res["rotor_onoff"] = dict(points=tab, per_link_spheres=per_link)
# ---------- (3) indy_ws 정의 재현 ----------
def indy_style(ri, q, link):
    m, d = ri.model, ri.data
    pin.computeJointJacobians(m, d, q); pin.updateFramePlacements(m, d)
    fid = m.getFrameId(link, pin.BODY); jid = m.frames[fid].parentJoint; lever = m.inertias[jid].lever
    R = d.oMf[fid].rotation; r = R @ lever; J6 = pin.getFrameJacobian(m, d, fid, pin.LOCAL_WORLD_ALIGNED)
    Jv = J6[:3] - np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]]) @ J6[3:]
    pin.crba(m, d, q); M = np.triu(d.M) + np.triu(d.M, 1).T + (np.diag(ri.rotor_diag) if ri.rotor else 0)
    import scipy.linalg as sl
    return Jv @ sl.solve(M, Jv.T, assume_a="sym"), lever, (d.oMf[fid].translation - d.oMi[jid].translation)
ind = {}
for l in [f"fr3v2_1_link{i}" for i in range(1, 8)] + ["fr3v2_1_hand"]:
    diffs = []; off_frame = 0.0
    for q in Q[:40]:
        A, lever, dfo = indy_style(ri_on, q, l); B = ri_on.point_inv_inertia(q, l, lever)[1]
        diffs.append(np.linalg.norm(A - B) / np.linalg.norm(B)); off_frame = max(off_frame, float(np.linalg.norm(dfo)))
    ind[l] = dict(rel_max=float(max(diffs)), frame_vs_parent_joint_origin_m=off_frame)
res["indy_definition"] = ind
# ---------- (4) 시간 ----------
for ri, k in ((ri_on, "on"), (ri_off, "off")):
    for q in Q[:20]: ri.compute(q)
    ts = []
    for _ in range(5):
        t0 = time.perf_counter()
        for q in Q: ri.compute(q)
        ts.append((time.perf_counter() - t0) / len(Q) * 1e6)
    res[f"time_us_per_state_rotor_{k}"] = dict(min=float(min(ts)), median=float(np.median(ts)))
q = Q[0]; t0 = time.perf_counter()
for _ in range(2000): pin.crba(ri_on.model, ri_on.data, q)
res["time_us_crba"] = (time.perf_counter() - t0) / 2000 * 1e6
# ---------- (5) μ 표 ----------
rec = np.load(f"{G}/logs/g1_live_nodrop/rec.npz"); INV = rec["inv"].astype(np.float64)
SEG = {"right_hand": 18, "left_hand": 17, "right_lower_arm": 16, "head": 12, "chest": 8, "abdomen": 3, "pelvis": 0}
tcp_on = [pt(ri_on, q, "fr3v2_1_hand_tcp") for q in Q]; tcp_off = [pt(ri_off, q, "fr3v2_1_hand_tcp") for q in Q]
mu = {}
for nm, s in SEG.items():
    mh, mon, moff = [], [], []
    for _ in range(4000):
        u = rng.normal(size=3); u /= np.linalg.norm(u); f = rng.integers(len(INV)); qi = rng.integers(len(Q))
        h = 1.0 / (u @ INV[f, s] @ u); a = meff(tcp_on[qi], u); b = meff(tcp_off[qi], u)
        mh.append(h); mon.append(a * h / (a + h)); moff.append(b * h / (b + h))
    mh, mon, moff = map(np.array, (mh, mon, moff))
    mu[nm] = dict(m_h_median=float(np.median(mh)), mu_off_median=float(np.median(moff)), mu_on_median=float(np.median(mon)),
                  mu_over_mh_off_median=float(np.median(moff / mh)), mu_over_mh_on_median=float(np.median(mon / mh)))
res["mu_tcp"] = mu
json.dump(res, open(f"{G}/logs/g1_robot_validate.json", "w"), indent=1)
print(json.dumps(res, ensure_ascii=False, indent=1))
