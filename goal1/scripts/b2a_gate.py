"""b2a_gate.py - offline gate for B2a (run with g1_run_human.sh py). For each orientation and line: along the 21 IK poses, the MPC robot spheres
(yml radius + 0.02, RobotInertiaNP centres) vs the v2 right-hand shapes; the closest moment and sphere; u = sphere centre -> right-hand COM
(kernel definition 'com'); m_h = 1/(u' Lh u) (static pose, Isaac frame), m_r = 1/(u' Lr u) at that pose, mu, eta (set A); iso: m_h = 0.6 kg.
Gate: eta_front - eta_top >= 0.06 and eta_front - eta_side >= 0.06 (each judged)."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import shape_metrics as SM
from effmass_robot.robot_inertia_np import RobotInertiaNP
H = os.path.expanduser("~"); G = f"{H}/effmass_mpc/goal1/b2a/g2"
IK = json.load(open(f"{G}/ik.json")); L = json.load(open(f"{G}/lines.json")); tf = json.load(open(f"{G}/temp_transform.json")); R, t = np.array(tf["R"]), np.array(tf["t"])
Z = np.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_v2.npz"); seg = Z["shape_segment"]; rad = Z["shape_radius"].astype(float)
p0 = Z["p0"][0].astype(float) @ R.T + t; p1 = Z["p1"][0].astype(float) @ R.T + t
COM = Z["com"][0][18].astype(float) @ R.T + t; Lh = R @ Z["inv"][0][18].astype(float) @ R.T
HAND = [k for k in range(24) if seg[k] == 18 and rad[k] > 0]; ALLH = [k for k in range(24) if rad[k] > 0]
a = np.array(L["arm_axis"]); RI = RobotInertiaNP(rotor=True); RS = RI.sphere_radius + 0.02
A = dict(eta0=0.10, muref=2.0, p=0.5, emin=0.05, emax=0.20)
eta = lambda mu: float(np.clip(A["eta0"] * (mu / A["muref"]) ** A["p"], A["emin"], A["emax"]))
flag = lambda e: "eta_max" if e >= A["emax"] - 1e-9 else "eta_min" if e <= A["emin"] + 1e-9 else ""
res = {}
for on, lines in IK["ik"].items():
    res[on] = {}
    for nm, d in lines.items():
        best = None
        for i, q in enumerate(d["q"]):
            C, Lr, _ = RI.compute(np.array(q))
            G_ = np.stack([SM.segdist(C, p0[k], p1[k]) - rad[k] for k in HAND], 1) - RS[:, None]       # (61, nh) MPC sphere surface gaps
            j, kk = np.unravel_index(np.argmin(G_), G_.shape)
            if best is None or G_[j, kk] < best[0]:
                best = (float(G_[j, kk]), i, int(j), HAND[kk], C, Lr)
        g, i, j, k, C, Lr = best
        ab = p1[k] - p0[k]; tt = np.clip(((C[j] - p0[k]) @ ab) / max(ab @ ab, 1e-12), 0, 1); Q = p0[k] + tt * ab; nd = (C[j] - Q) / np.linalg.norm(C[j] - Q)
        UDEF = os.environ.get("B2A_UDEF", "nearest")
        fbk = bool(np.linalg.norm(Q - C[j]) < 1e-3)
        u = (COM if (UDEF == "com" or fbk) else Q) - C[j]; u /= np.linalg.norm(u); ah = float(u @ Lh @ u); ar = float(u @ Lr[j] @ u)
        mh, mr = 1 / ah, (1 / ar if ar > 0 else float("inf")); mu = 1 / (ah + ar); mui = 1 / (1 / 0.6 + ar)
        tcp_gap = float(min((SM.segdist(np.array([d["tcp"][i]]), p0[kk_], p1[kk_]) - rad[kk_]).min() for kk_ in HAND))
        res[on][nm] = dict(u_def=UDEF, fallback=fbk, closest_point_index=i, tcp=d["tcp"][i], tcp_gap_to_hand_shape_m=tcp_gap, sphere=int(j), sphere_link=RI.sphere_link[j], sphere_gap_m=g, hand_shape=int(k),
                           nearest_dir=nd.tolist(), nearest_dir_vs_arm_axis_deg=float(np.degrees(np.arccos(abs(nd @ a)))), u=u.tolist(),
                           u_vs_arm_axis_deg=float(np.degrees(np.arccos(np.clip(u @ (-a), -1, 1)))), m_h=mh, m_r=mr, mu=mu, eta=eta(mu), eta_flag=flag(eta(mu)),
                           mu_iso=mui, eta_iso=eta(mui), eta_iso_flag=flag(eta(mui)), q=d["q"][i])
    f = res[on]["front"]["eta"]
    res[on]["gate_front_mass_minus_iso"] = res[on]["front"]["eta"] - res[on]["front"]["eta_iso"]
    res[on]["gate"] = dict(front_minus_top=f - res[on]["top"]["eta"], front_minus_side=f - res[on]["side"]["eta"],
                           top_pass=f - res[on]["top"]["eta"] >= 0.06, side_pass=f - res[on]["side"]["eta"] >= 0.06,
                           iso_front_minus_top=res[on]["front"]["eta_iso"] - res[on]["top"]["eta_iso"], iso_front_minus_side=res[on]["front"]["eta_iso"] - res[on]["side"]["eta_iso"])
json.dump(res, open(f"{G}/gate_{os.environ.get('B2A_UDEF', 'nearest')}.json", "w"), indent=1)
for on in res:
    print("==", on, "front mass-iso eta", round(res[on]["gate_front_mass_minus_iso"], 4), res[on]["gate"])
    for nm in ("front", "top", "side"):
        r = res[on][nm]
        print(f"  {nm:5s} idx {r['closest_point_index']:2d} sphere {r['sphere']:2d} {r['sphere_link']:22s} gap {r['sphere_gap_m']*100:6.2f} cm (tcp {r['tcp_gap_to_hand_shape_m']*100:5.2f})"
              f" dir-vs-axis {r['nearest_dir_vs_arm_axis_deg']:5.1f}  u-vs-front {r['u_vs_arm_axis_deg']:5.1f}  m_h {r['m_h']:6.3f} m_r {r['m_r']:6.3f} mu {r['mu']:5.3f} eta {r['eta']:.4f}{r['eta_flag']}"
              f" | iso mu {r['mu_iso']:.3f} eta {r['eta_iso']:.4f}{r['eta_iso_flag']}")
