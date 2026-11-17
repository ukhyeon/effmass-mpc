"""b2a5_metrics2.py - danger index (Kulic & Croft type) and SSM protective-distance violation from saved runs only (system python via g1_run_human.sh py).
Per step: robot sphere s = the sphere closest to the v2 hand shapes (kernel shape gap from the B1 rows, MPC radius); d = hand-mesh gap of that sphere
(signed, MPC radius yml + 0.02); u = nearest-definition direction from s to the hand shape; v_n = max(0, v_s . u) with v_s the central finite difference
of the executed sphere centre (60 Hz); mu_personal = 1 / (u'Lh u + u'Lr u) (static B2a hand inertia, robot inverse inertia at the executed q),
mu_ISO = 1 / (1/0.6 + u'Lr u).
DI = f_d * f_v * f_I, f_d = clip((D_max - d) / D_max, 0, 1)^2 (D_max 0.20 / 0.30 / 0.40 m), f_v = clip(v_n / 0.5, 0, 1), f_I = mu / 4.6 (lin) or sqrt(mu / 4.6) (sqrt).
Per run: DI max and DI integral (sum x 1/60 s). SSM (assumed values): S_p = v_r T_r + v_r^2 / (2 a_b) + Z_d + Z_r, v_r = v_n, T_r 0.1 s, a_b 2.0 m/s^2,
Z_d 0.03 m, Z_r 0.01 m; violation time (d < S_p) and min(d - S_p). Paired differences, interaction, percent. Output logs/meff_isaac/b2a5_metrics2.json."""
import json, os, sys, csv
import numpy as np
from scipy.stats import binomtest
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
O = BA.O; MU_MAX = 4.6; VCAP = 0.5; TR, AB, ZD, ZR = 0.1, 2.0, 0.03, 0.01; DMAX = (0.20, 0.30, 0.40)
SETS = {"b2a5": {"front": "front_0.38h1", "top": "top_0.455"}, "b2a4": {"front": "front_0.31h1", "top": "top_0.514"}}
CONDS = {"b2a5": ["off", "iso", "massA", "unim"], "b2a4": ["off", "iso", "massA", "unim", "uni10"]}


def one(job):
    st, sc, cond, seed = job; tag = f"{st}_{SETS[st][sc]}_{cond}_s{seed}"
    r = BA.result(tag); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))]); n = len(Q)
    B = np.load(f"{O}/{tag}_b1.npz")["rows"]; RL = [BA.RI.compute(q) for q in Q]; Cs = np.stack([x[0] for x in RL]); V = np.gradient(Cs, 1 / 60.0, axis=0)
    d = np.zeros(n); vn = np.zeros(n); mup = np.zeros(n); mui = np.zeros(n); sid = np.zeros(n, int)
    for i in range(n):
        D = B[i, 12:36][BA.HAND]; k = BA.HAND[int(np.argmin(D))]; s = int(B[i, 36 + k]); c = Cs[i, s]; sid[i] = s
        dm, _ = BA.hand_mesh_signed(c[None]); d[i] = float(dm[0] - BA.RS[s])
        u, _, _ = BA.shape_u(c); ar = float(u @ RL[i][1][s] @ u); ah = float(u @ BA.LH @ u)
        vn[i] = max(0.0, float(V[i, s] @ u)); mup[i] = 1 / (ah + ar); mui[i] = 1 / (1 / 0.6 + ar)
    fv = np.clip(vn / VCAP, 0, 1); out = dict(tag=tag, set=st, scene=sc, cond=cond, seed=seed, steps=n)
    for Dm in DMAX:
        fd = np.clip((Dm - d) / Dm, 0, 1) ** 2
        for mn, mu in (("personal", mup), ("iso", mui)):
            for form, fI in (("lin", mu / MU_MAX), ("sqrt", np.sqrt(mu / MU_MAX))):
                DI = fd * fv * fI; key = f"DI_{mn}_{form}_D{int(Dm*100)}"
                out[key + "_max"] = float(DI.max()); out[key + "_int"] = float(DI.sum() / 60.0)
    Sp = vn * TR + vn ** 2 / (2 * AB) + ZD + ZR; viol = d < Sp
    out.update(ssm_viol_s=float(viol.sum() / 60.0), ssm_min_margin=float((d - Sp).min()), d_min=float(d.min()), vn_max=float(vn.max()),
               mu_personal_at_dmin=float(mup[int(np.argmin(d))]), mu_iso_at_dmin=float(mui[int(np.argmin(d))]))
    return out


def stats(x):
    x = np.array([v for v in x if v is not None and np.isfinite(v)], float)
    if not len(x):
        return None
    m = float(x.mean()); sg = np.sign(m) if m != 0 else 1.0; nz = x[x != 0]; k = int((np.sign(nz) == sg).sum())
    return dict(mean=m, same_sign=k, n_nonzero=int(len(nz)), p_sign=float(binomtest(k, len(nz), 0.5).pvalue) if len(nz) else None, values=x.tolist())


if __name__ == "__main__":
    from multiprocessing import Pool
    jobs = [(st, sc, c, s) for st in SETS for sc in SETS[st] for c in CONDS[st] for s in range(8) if os.path.exists(f"{O}/{st}_{SETS[st][sc]}_{c}_s{s}_b1.npz")]
    with Pool(10) as p:
        R = p.map(one, jobs)
    KEYS = [k for k in R[0] if k.startswith("DI_") or k.startswith("ssm_")] + ["d_min", "vn_max"]
    res = {"runs": R, "by_condition": {}, "paired": {}, "interaction": {}, "pct": {}}
    get = lambda st, sc, c, s, k: next((r[k] for r in R if r["set"] == st and r["scene"] == sc and r["cond"] == c and r["seed"] == s), None)
    for st in SETS:
        for sc in SETS[st]:
            for c in CONDS[st]:
                rs = [r for r in R if r["set"] == st and r["scene"] == sc and r["cond"] == c]
                if rs:
                    res["by_condition"][f"{st}/{sc}/{c}"] = {k: float(np.mean([r[k] for r in rs])) for k in KEYS}
            for b in ("iso", "unim"):
                res["paired"][f"{st}/{sc}/massA-{b}"] = {k: stats([get(st, sc, "massA", s, k) - get(st, sc, b, s, k) if get(st, sc, b, s, k) is not None else None for s in range(8)]) for k in KEYS}
                res["pct"][f"{st}/{sc}/massA-vs-{b}"] = {k: (100 * (res["by_condition"][f"{st}/{sc}/massA"][k] / res["by_condition"][f"{st}/{sc}/{b}"][k] - 1)
                                                         if res["by_condition"][f"{st}/{sc}/{b}"][k] != 0 else None) for k in KEYS}
        res["interaction"][st] = {k: stats([(get(st, "front", "massA", s, k) - get(st, "front", "iso", s, k)) - (get(st, "top", "massA", s, k) - get(st, "top", "iso", s, k)) for s in range(8)]) for k in KEYS}
    res["assumptions"] = dict(mu_max=MU_MAX, v_cap=VCAP, D_max=DMAX, T_r=TR, a_b=AB, Z_d=ZD, Z_r=ZR, note="SSM values are assumptions; human static (no human motion term)")
    json.dump(res, open(f"{O}/b2a5_metrics2.json", "w"), indent=1)
    print("runs", len(R))
