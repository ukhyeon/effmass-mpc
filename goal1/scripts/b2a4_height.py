"""b2a4_height.py - offline IK only: how the closest-moment u and the gate values change if the front-cal line (x 0.31) is moved by dz
(system python via g1_run_human.sh py). Assumption (not run): the off pass moves rigidly with the line height, so the closest-moment hand-link pose
of the off run (seed 0, and seed 1 for reference) is shifted by dz with the same orientation (including the yaw error), and solved by damped
least squares IK (numpy FK, start = executed q, 6D pose error). Also the same positions with the goal orientation (gripper down, no yaw error).
Gate values as b2a4_gate.at_pose."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import b2a4_gate as GT
import b2a2_halflen as HL
RI = BA.RI; b, Tl = RI.body_of["fr3v2_1_hand"]


def fk(q):
    W = RI._fk(np.asarray(q, float)); return W[b + 1] @ Tl


def logR(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1); a = np.arccos(c)
    if a < 1e-9:
        return np.zeros(3)
    return a / (2 * np.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def ik(q0, Tt, it=200):
    q = np.array(q0, float)
    for _ in range(it):
        F = fk(q); e = np.r_[Tt[:3, 3] - F[:3, 3], logR(Tt[:3, :3] @ F[:3, :3].T)]
        if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-4:
            break
        J = np.zeros((6, 7))
        for i in range(7):
            dq = np.zeros(7); dq[i] = 1e-6; F2 = fk(q + dq)
            J[:3, i] = (F2[:3, 3] - F[:3, 3]) / 1e-6; J[3:, i] = logR(F2[:3, :3] @ F[:3, :3].T) / 1e-6
        q = q + J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), e)
    F = fk(q); return q, float(np.linalg.norm(Tt[:3, 3] - F[:3, 3])), float(np.linalg.norm(logR(Tt[:3, :3] @ F[:3, :3].T)))


out = []
for seed in (0, 1):
    m = json.load(open(f"{BA.O}/b2a4_cal/b2a4_front_0.31_off_s{seed}_cal.json")); q0 = np.array(m["q_at_min"]); F0 = fk(q0)
    Rdown = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)    # quaternion (0, 1, 0, 0): gripper down, goal yaw
    for orient in ("executed", "goal"):
        for dz in (-0.02, -0.01, 0.0, 0.01, 0.02):
            Tt = F0.copy(); Tt[2, 3] += dz
            if orient == "goal":
                Tt[:3, :3] = Rdown
            q, ep, er = ik(q0, Tt); r = GT.at_pose(q); marg = float(np.min(np.minimum(q - HL.LO, HL.HI - q)))
            row = dict(seed=seed, orientation=orient, dz=dz, tcp=(fk(q)[:3, 3] + fk(q)[:3, 2] * 0.1034).round(4).tolist(), ik_pos_err=ep, ik_rot_err=er, joint_margin=marg,
                       **{k: v for k, v in r.items() if k != "u"}, d_eta=r["eta_massA"] - r["eta_iso"])
            out.append(row)
            print(f"s{seed} {orient:8s} dz {dz:+.2f} gap {r['gap']*100:5.2f} cm {r['sphere_link']:20s} u {r['u_vs_front_deg']:5.1f} m_h {r['m_h']:5.2f} m_r {r['m_r']:5.2f} mu {r['mu']:.3f} "
                  f"eta A {r['eta_massA']:.4f} iso {r['eta_iso']:.4f} d {100*(r['eta_massA']-r['eta_iso']):5.2f} cm | ik err {ep:.1e} {er:.1e} margin {marg:.2f}", flush=True)
json.dump(out, open(f"{BA.G}/b2a4_height.json", "w"), indent=1)
