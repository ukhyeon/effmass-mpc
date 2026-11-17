"""g1_fr3v21_usd_fk.py — fr3v2_1 USD 를 pxr 로만 읽어 (PhysX 없이) 관절 body0/body1·로컬 프레임·축으로 FK, URDF numpy FK 와 손(또는 link8) 위치 비교"""
import os, json, numpy as np
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdPhysics, UsdGeom, Gf
P = os.path.expanduser("~/effmass_mpc/ref/franka_simulation_release_v0.1.0/fr3v2_1/fr3v2_1.usda")
st = Usd.Stage.Open(P)
prims = [p for p in st.Traverse()]
print("[fk] 이름에 hand/finger/link8/tcp 포함 prim:", sorted({p.GetName() for p in prims if any(k in p.GetName() for k in ("hand", "finger", "link8", "tcp"))})[:20])
J = {}
for p in prims:
    if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.FixedJoint) or p.IsA(UsdPhysics.PrismaticJoint):
        j = UsdPhysics.Joint(p)
        b0 = j.GetBody0Rel().GetTargets(); b1 = j.GetBody1Rel().GetTargets()
        ax = p.GetAttribute("physics:axis").Get() if p.HasAttribute("physics:axis") else None
        J[p.GetName()] = dict(type=p.GetTypeName(), body0=[str(x) for x in b0], body1=[str(x) for x in b1], axis=ax,
                               lp0=j.GetLocalPos0Attr().Get(), lr0=j.GetLocalRot0Attr().Get(), lp1=j.GetLocalPos1Attr().Get(), lr1=j.GetLocalRot1Attr().Get())
for n, d in list(J.items())[:3]:
    print("[fk] 관절", n, d)
def Tm(pos, rot):
    A = np.eye(4); q = rot; w, x, y, z = q.GetReal(), *q.GetImaginary()
    A[:3, :3] = np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)], [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)], [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])
    A[:3, 3] = [pos[0], pos[1], pos[2]]; return A
def Rax(ax, th):
    v = {"X": [1, 0, 0], "Y": [0, 1, 0], "Z": [0, 0, 1]}[ax]; K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    A = np.eye(4); A[:3, :3] = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K; return A
def usd_fk(q):
    W = {}; names = [f"fr3v2_1_joint{i}" for i in range(1, 8)]
    base = J[names[0]]["body0"][0].split("/")[-1]; W[base] = np.eye(4)
    for k, n in enumerate(names):
        d = J[n]; b0 = d["body0"][0].split("/")[-1]; b1 = d["body1"][0].split("/")[-1]
        W[b1] = W[b0] @ Tm(d["lp0"], d["lr0"]) @ Rax(d["axis"], q[k]) @ np.linalg.inv(Tm(d["lp1"], d["lr1"]))
    return W
g = {}; exec(open(os.path.expanduser("~/effmass_mpc/goal1/scripts/g1_variant_meff.py")).read().split("models = {m: load(m)")[0], g)
mod = g["load"]("fr3v2_1")
rng = np.random.default_rng(0); errs = []
lo = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]); hi = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
for k in range(10):
    q = lo + (hi - lo) * rng.uniform(0.1, 0.9, 7)
    Wu = usd_fk(q); Wr, _ = g["fk"](mod, q)
    L7u = Wu["fr3v2_1_link7"]; L7r = Wr["fr3v2_1_link7"]
    errs.append(float(np.linalg.norm(L7u[:3, 3] - L7r[:3, 3])))
    if k == 0: print("[fk] 회전 차 (link7) 최대 성분", np.abs(L7u[:3, :3] - L7r[:3, :3]).max())
print(f"[fk] link7 원점 위치: USD(관절 프레임 numpy FK, PhysX 없음) ↔ URDF numpy FK 차 최대 {max(errs)*1000:.4f} mm (10 자세)")
json.dump(dict(joints={k: {kk: str(vv) for kk, vv in v.items()} for k, v in J.items()}, link7_fk_err_mm=[e * 1000 for e in errs]),
          open(os.path.expanduser("~/effmass_mpc/goal1/logs/g1_fr3v21_usd_fk.json"), "w"), indent=1)

app.close()
