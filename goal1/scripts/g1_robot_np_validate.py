"""g1_robot_np_validate.py — 1-2d 결정 1: numpy Λ_r (RobotInertiaNP) ↔ Pinocchio (RobotInertia), 1-2c 와 같은 상태 201개, 시간 내역 (시스템 python3)"""
import json, os, time, numpy as np
from effmass_robot.robot_inertia import RobotInertia
from effmass_robot.robot_inertia_np import RobotInertiaNP, spd_inv
rng = np.random.default_rng(1)        # g1_robot_validate.py 와 같은 생성 순서
lo = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]); hi = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
Q = [np.array([0, -1.3, 0, -2.5, 0, 1.0, 0])] + [lo + (hi - lo) * rng.uniform(0.05, 0.95, 7) for _ in range(200)]
res = {}

for rot in (True, False):
    P, N = RobotInertia(rotor=rot), RobotInertiaNP(rotor=rot)
    assert P.sphere_link == N.sphere_link and np.allclose(P.sphere_radius, N.sphere_radius)
    rel, cen, mrel, zero_same = [], [], [], True
    for q in Q:
        Cp, Lp, Mp = P.compute(q); Cn, Ln, Mn = N.compute(q)
        cen.append(np.abs(Cp - Cn).max()); mrel.append(np.abs(Mp - Mn).max() / np.abs(Mp).max())
        nz = np.linalg.norm(Lp, axis=(1, 2)) > 0
        zero_same &= bool(np.array_equal(np.nonzero(~nz)[0], np.nonzero(np.linalg.norm(Ln, axis=(1, 2)) == 0)[0])) and np.abs(Ln[~nz]).max() == 0.0
        rel.append((np.linalg.norm(Lp - Ln, axis=(1, 2))[nz] / np.linalg.norm(Lp, axis=(1, 2))[nz]).max())
    res[f"rotor_{'on' if rot else 'off'}"] = dict(states=len(Q), inv_inertia_rel_max=float(max(rel)), center_abs_max_m=float(max(cen)), M_rel_max=float(max(mrel)),
                                                  zero_spheres_same=bool(zero_same), zero_spheres=np.nonzero(np.linalg.norm(N.compute(Q[0])[1], axis=(1, 2)) == 0)[0].tolist())
N = RobotInertiaNP(rotor=True)
for q in Q[:50]: N.compute(q, timed=True)
T = {k: [] for k in N.timing}; tot = []
for rep in range(5):
    for q in Q:
        N.compute(q, timed=True)
        for k, v in N.timing.items(): T[k].append(v)
    t = time.perf_counter()
    for q in Q: N.compute(q)
    tot.append((time.perf_counter() - t) / len(Q) * 1e6)
res["time_us_median"] = {k: round(float(np.median(v)), 1) for k, v in T.items()}
res["time_us_p99"] = {k: round(float(np.percentile(v, 99)), 1) for k, v in T.items()}
res["time_us_untimed_per_state"] = dict(min=round(min(tot), 1), median=round(float(np.median(tot)), 1))
P = RobotInertia(rotor=True)
t = time.perf_counter()
for q in Q: P.compute(q)
res["pinocchio_module_us_per_state"] = round((time.perf_counter() - t) / len(Q) * 1e6, 1)
for rot in (True, False):   # spd_inv ↔ np.linalg.inv (같은 M, 201 상태)
    N = RobotInertiaNP(rotor=rot); dd = []
    for q in Q:
        M = N.compute(q)[2]; A, B = spd_inv(M), np.linalg.inv(M); dd.append(np.abs(A - B).max() / np.abs(B).max())
    res[f"spd_inv_vs_np_inv_rel_max_rotor_{'on' if rot else 'off'}"] = float(max(dd))
json.dump(res, open(os.path.expanduser("~/effmass_mpc/goal1/logs/g1_robot_np_validate.json"), "w"), indent=1)
print(json.dumps(res, ensure_ascii=False, indent=1))
