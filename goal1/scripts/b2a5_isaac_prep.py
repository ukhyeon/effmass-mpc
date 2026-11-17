"""b2a5_isaac_prep.py <scenekey> <cond> [seed] - per-step data for the Isaac render (system python via g1_run_human.sh py): executed joint q, the hand
shape setting the cost at that step (closest), its capsule (p0, p1, radius) and the eta shell radius (radius + eta of the closest pair), closest robot
sphere centre / radius and closest point on the hand-shape axis, gap, mu, m_h, eta, u angle. Output goal1/b2a/render/<tag>_prep.npz (Isaac frame)."""
import json, os, sys, csv
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
key, cond = sys.argv[1], sys.argv[2]; seed = int(sys.argv[3]) if len(sys.argv) > 3 else 0
tag = f"b2a5_{key}_{cond}_s{seed}"; A = dict(eta0=0.10, muref=2.0, p=0.5, emin=0.05, emax=0.20)
eta = lambda mu: float(np.clip(A["eta0"] * (mu / A["muref"]) ** A["p"], A["emin"], A["emax"]))
r = BA.result(tag); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))])
z1 = np.load(f"{BA.O}/{tag}_b1.npz"); B = z1["rows"]; eta0 = float(z1["cost_par"][1]); out = {k: [] for k in ("k", "shell_r", "c", "cr", "qp", "gap", "mu", "mh", "eta", "u", "tcp")}
for i, q in enumerate(Q):
    C, Lr, _ = BA.RI.compute(q); D = B[i, 12:36][BA.HAND]; jj = int(np.argmin(D)); k = BA.HAND[jj]; s = int(B[i, 36 + k]); mu = float(B[i, 60 + k]); mr = float(B[i, 108 + k])
    u, _, qp = BA.shape_u(C[s]); mh = 1 / float(u @ BA.LH @ u)
    e = eta(mu) if cond == "massA" else eta(1 / (1 / 0.6 + (1 / mr if np.isfinite(mr) and mr > 0 else 0))) if cond == "iso" else eta0 if cond in ("unim", "uni10") else 0.0
    for kk, v in (("k", k), ("shell_r", BA.RAD[k] + e), ("c", C[s]), ("cr", BA.RI.sphere_radius[s]), ("qp", qp), ("gap", float(D[jj])), ("mu", mu), ("mh", mh), ("eta", e),
                  ("u", float(np.degrees(np.arccos(np.clip(u @ (-BA.AX), -1, 1))))), ("tcp", B[i, 8:11] - np.array([0, 0, 0.1034]))):
        out[kk].append(v)
os.makedirs(f"{BA.OUT}/render", exist_ok=True)
tf = json.load(open(f"{BA.G}/temp_transform.json")); MV = np.load(os.path.expanduser("~/effmass_mpc/goal1/data/human_b2a_reach_verts.npz"))
np.savez(f"{BA.OUT}/render/{tag}_prep.npz", q=Q, p0=BA.P0, p1=BA.P1, rad=BA.RAD, hand=np.array(BA.HAND), verts=MV["verts"][0].astype(float) @ np.array(tf["R"]).T + np.array(tf["t"]),
         faces=MV["faces"], **{k: np.array(v) for k, v in out.items()}, cond=cond, tag=tag)
print("wrote", tag, len(Q), "steps; min gap %.2f cm at step %d" % (100 * min(out["gap"]), int(np.argmin(out["gap"]))))
