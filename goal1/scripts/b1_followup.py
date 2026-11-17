"""b1_followup.py — B1 후속 분석 (저장된 기록만, 시스템 python3 -B) → logs/meff_isaac/b1_followup.json
기록: 실행별 _b1.npz (스텝별 형상 24개 각각의 로봇 구 전체 최소 간격·그 구·μ·m_h·m_r), mass A·iso 의 _near.npz (0.3 m 안 모든 (구, 형상) 쌍).
off·uniform 은 근접 기록이 없어 (구, 형상) 전체 쌍이 필요한 항목은 '기록 없음'. 모든 값은 실행된 상태 (rollout 아님)."""
import json, os, sys, collections
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b1_analyze as B
O = B.O; SEG = list(B.PRE["segment_names"]); SHSEG = B.SHSEG; GSH = B.GSH
SH_G = {k: g for g, ks in GSH.items() for k in ks}
CONDS = ["off", "uni10", "unim", "iso", "massA"]; SEEDS = range(8)
EMIN, EMAX = B.A["eta_min"], B.A["eta_max"]
LIM_LO = np.array([-2.744, -1.784, -2.901, -3.042, -2.806, 0.544, -3.016]); LIM_HI = np.array([2.744, 1.784, 2.901, -0.152, 2.806, 4.517, 3.016])
out = {}


def cost(d, eta):
    return np.where(d < 0, 0.5 * eta - d, np.where(d < eta, (eta - d) ** 2 * (0.5 / eta), 0.0))


def eta_of(cond, mu, seg, mr, eta0=None):
    if cond == "massA":
        return B.eta_A(mu)
    if cond == "iso":
        return B.eta_A(B.mu_iso(seg, mr))
    return np.full(np.shape(mu), eta0, float)


S = json.load(open(f"{O}/b1_summary.json"))
R = S["runs"]
# ---------------- 1. 선택성
sel = {}
for c in CONDS:
    for s in SEEDS:
        r = R[c][str(s)]
        sel[(c, s)] = dict(C=r["torso"]["min_gap"] - r["hand"]["min_gap"], C2=r["torso"]["min_gap"] - r["forearm"]["min_gap"], C3=r["torso"]["min_gap"] - r["upperarm"]["min_gap"])
o1 = {"per_cond_mean": {c: {k: float(np.mean([sel[(c, s)][k] for s in SEEDS])) for k in ("C", "C2", "C3")} for c in CONDS}, "paired": {}}
for a, b in [("massA", "unim"), ("massA", "uni10"), ("massA", "iso"), ("iso", "unim")]:
    for k in ("C", "C2", "C3"):
        d = np.array([sel[(a, s)][k] - sel[(b, s)][k] for s in SEEDS])
        o1["paired"][f"{a}-{b}:{k}"] = dict(mean=float(d.mean()), std=float(d.std(ddof=1)), n_pos=int((d > 0).sum()), n_neg=int((d < 0).sum()), sign_p=B.sign_p(d),
                                             per_seed=d.round(5).tolist())
out["1_selectivity"] = o1

# ---------------- 2. 손 최근접 순간
HAND = GSH["hand"]; TORSO = GSH["torso"]
o2 = {}
for c in CONDS:
    acc = collections.Counter(); hd_me = []; td_me = []; n_hand = 0; n_steps = 0; n_nocost = 0; td_out = 0; per_seed = {}
    for s in SEEDS:
        tag = f"b1_{c}_s{s}"; z = np.load(f"{O}/{tag}_b1.npz"); X = z["rows"]; par = z["cost_par"]
        D = X[:, 12:36]; SM = X[:, 36:60].astype(int); MU = X[:, 60:84]; MR = X[:, 108:132]; st = X[:, 0].astype(int)
        kmin = D.argmin(1); hs = np.isin(kmin, HAND); n_steps += len(D); n_hand += int(hs.sum())
        per_seed[s] = int(hs.sum())
        if c in ("massA", "iso"):
            N = np.load(f"{O}/{tag}_near.npz")["near"]; key = N[:, 0].astype(np.int64) * 100 + N[:, 1].astype(np.int64)
            order = np.argsort(key, kind="stable"); Ns = N[order]; ks = key[order]
            for i in np.nonzero(hs)[0]:
                k = kmin[i]; sph = SM[i, k]; kk = st[i] * 100 + sph
                lo, hi = np.searchsorted(ks, kk), np.searchsorted(ks, kk, side="right"); rows = Ns[lo:hi]
                if not len(rows):
                    continue
                e = eta_of(c, rows[:, 7], rows[:, 3].astype(int), rows[:, 6]); cc = cost(rows[:, 4], e)
                if cc.max() <= 0:
                    acc["비용 없음"] += 1; n_nocost += 1
                else:
                    acc[SH_G[int(rows[np.argmax(cc), 2])]] += 1
                hm = np.isin(rows[:, 2].astype(int), HAND); tm = np.isin(rows[:, 2].astype(int), TORSO)
                j = np.argmin(np.where(hm, rows[:, 4], 9)); hd_me.append(rows[j, 4] - e[j])
                if tm.any():
                    j = np.argmin(np.where(tm, rows[:, 4], 9)); td_me.append(rows[j, 4] - e[j])
                else:
                    td_out += 1
        else:
            eta0 = float(par[1]) if c != "off" else None
            if c != "off":
                dh = D[hs, kmin[hs]]; hd_me += (dh - eta0).tolist()
                acc["손"] += int((dh < eta0).sum()); acc["비용 없음"] += int((dh >= eta0).sum())
                # 같은 로봇 구의 몸통 간격은 기록 없음 (형상마다 로봇 구 전체 최솟값만 저장)
    q = lambda v: dict(n=len(v), p5=float(np.percentile(v, 5)), median=float(np.median(v)), p95=float(np.percentile(v, 95))) if len(v) else None
    tot = sum(acc.values())
    o2[c] = dict(steps_total=n_steps, steps_hand_nearest=n_hand, per_seed=per_seed, cost_shape_group_frac={k: v / tot for k, v in acc.most_common()} if tot else None,
                 hand_gap_minus_eta_cm=q(np.array(hd_me) * 100) if hd_me else None, torso_gap_minus_eta_cm=q(np.array(td_me) * 100) if td_me else None,
                 torso_outside_0p3=td_out if c in ("massA", "iso") else None,
                 note=None if c in ("massA", "iso") else ("off: 비용 없음" if c == "off" else "uniform: η 같아 최근접(손)이 비용을 정함. 같은 로봇 구의 몸통 간격은 기록 없음"))
# 사람 자세 자체: 손 형상 ↔ 가장 가까운 몸통 형상 (캡슐 표면 간 거리), 재생 전 프레임
def segseg(p1, q1, p2, q2):
    # 선분-선분 최근접 거리 (표본 21×21 + 국소 정밀화 대신 해석해)
    d1 = q1 - p1; d2 = q2 - p2; r = p1 - p2; a = d1 @ d1; e = d2 @ d2; f = d2 @ r
    if a <= 1e-12 and e <= 1e-12:
        return np.linalg.norm(p1 - p2)
    if a <= 1e-12:
        s = 0.0; t = np.clip(f / e, 0, 1)
    else:
        cc = d1 @ r
        if e <= 1e-12:
            t = 0.0; s = np.clip(-cc / a, 0, 1)
        else:
            b = d1 @ d2; den = a * e - b * b
            s = np.clip((b * f - cc * e) / den, 0, 1) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0:
                t = 0.0; s = np.clip(-cc / a, 0, 1)
            elif t > 1:
                t = 1.0; s = np.clip((b - cc) / a, 0, 1)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))
P0, P1, RAD = B.PRE["p0"], B.PRE["p1"], B.PRE["shape_radius"]
F = len(P0); ht = {}
for side, ks in (("left", [20, 21]), ("right", [22, 23])):
    v = np.array([min(segseg(P0[f, h], P1[f, h], P0[f, t], P1[f, t]) - RAD[h] - RAD[t] for h in ks for t in TORSO) for f in range(F)])
    ht[side] = v
hb = np.minimum(ht["left"], ht["right"])
o2["human_pose_hand_torso_gap"] = {k: dict(p5=float(np.percentile(v, 5)), median=float(np.median(v)), p95=float(np.percentile(v, 95)), min=float(v.min()),
                                           frac_ge_eta_max=float((v >= EMAX).mean()), frac_ge_0p10=float((v >= 0.10).mean()), frac_lt_0p05=float((v < 0.05).mean()))
                                   for k, v in (("left", ht["left"]), ("right", ht["right"]), ("nearer_hand", hb))}
o2["human_pose_hand_torso_gap"]["frames"] = F
out["2_hand_nearest"] = o2

# ---------------- 3. 포화
o3 = {}
for c in ("massA", "iso"):
    N = np.concatenate([np.load(f"{O}/b1_{c}_s{s}_near.npz")["near"] for s in SEEDS]); links = np.load(f"{O}/b1_{c}_s0_near.npz")["sphere_link"]
    sg = N[:, 3].astype(int); k = N[:, 2].astype(int); mh = N[:, 5]; mr = N[:, 6]; mu = N[:, 7]
    mu_used = mu if c == "massA" else B.mu_iso(sg, mr)
    mh_used = mh if c == "massA" else B.M_ISO[sg]
    e = B.eta_A(mu_used); fin = np.isfinite(mr)
    grp = np.array([SH_G[x] for x in k])
    o3[c] = {"by_group": {}, "by_link": {}}
    for g in GSH:
        m = grp == g
        if not m.any():
            o3[c]["by_group"][g] = None; continue
        rh = mu_used[m] / mh_used[m]; rr = np.where(fin[m], mu_used[m] / np.where(fin[m], mr[m], 1), 0.0)
        o3[c]["by_group"][g] = dict(n=int(m.sum()), frac_eta_max=float((e[m] >= EMAX - 1e-9).mean()), frac_eta_min=float((e[m] <= EMIN + 1e-9).mean()),
                                    mu_over_mh=dict(p5=float(np.percentile(rh, 5)), median=float(np.median(rh)), p95=float(np.percentile(rh, 95))),
                                    mu_over_mr=dict(p5=float(np.percentile(rr, 5)), median=float(np.median(rr)), p95=float(np.percentile(rr, 95))),
                                    frac_robot_side_dominant=float((rr > 0.5).mean()))
    o3[c]["all"] = dict(n=int(len(N)), frac_eta_max=float((e >= EMAX - 1e-9).mean()), frac_eta_min=float((e <= EMIN + 1e-9).mean()))
    # 링크별: 0.3 m 안에 들어온 스텝 비율, m_r 중앙
    sl = np.array([str(links[int(x)]).replace("fr3v2_1_", "") for x in N[:, 1]])
    nst = 0; st_link = collections.defaultdict(int)
    for s in SEEDS:
        Ns = np.load(f"{O}/b1_{c}_s{s}_near.npz")["near"]; L = np.array([str(links[int(x)]).replace("fr3v2_1_", "") for x in Ns[:, 1]])
        nst += len(np.load(f"{O}/b1_{c}_s{s}_b1.npz")["rows"])
        for ln in np.unique(L):
            st_link[ln] += len(np.unique(Ns[L == ln, 0]))
    for ln in sorted(set(sl)):
        m = sl == ln; f = fin[m]
        o3[c]["by_link"][ln] = dict(rows=int(m.sum()), frac_steps_within_0p3=st_link[ln] / nst, mr_median=float(np.median(mr[m][f])) if f.any() else None,
                                    mr_inf_frac=float((~f).mean()))
out["3_saturation"] = o3

# ---------------- 4. infeasible·겹침
o4 = {"infeasible": [], "off_overlap": []}
for c in ("massA", "iso"):
    for s in SEEDS:
        z = np.load(f"{O}/b1_{c}_s{s}_b1.npz")["rows"]; f = z[:, 11]; bad = np.nonzero(f == 0)[0]
        if not len(bad):
            continue
        i = int(bad[0]); q = z[i, 1:8]; D = z[i, 12:36]; k = int(D.argmin())
        ml = q - LIM_LO; mh_ = LIM_HI - q; j = int(np.argmin(np.minimum(ml, mh_)))
        segs = np.split(bad, np.nonzero(np.diff(bad) > 1)[0] + 1)
        o4["infeasible"].append(dict(cond=c, seed=s, start_step=int(z[i, 0]), end_step=int(bad[-1]), n=int(len(bad)), segments=[[int(x[0]), int(x[-1])] for x in segs],
                                     human_frame=int(np.floor((z[i, 0] + 1) / 2 + 1e-6)), joint=j + 1, q=float(q[j]), limit=float(LIM_LO[j] if ml[j] < mh_[j] else LIM_HI[j]),
                                     margin=float(min(ml[j], mh_[j])), nearest_group=SH_G[k], nearest_seg=SEG[SHSEG[k]], nearest_gap=float(D[k]),
                                     min_joint_margin_all=np.round(np.minimum(ml, mh_), 4).tolist()))
for s in SEEDS:
    z = np.load(f"{O}/b1_off_s{s}_b1.npz")["rows"]; D = z[:, 12:36]; neg = np.nonzero(D.min(1) < 0)[0]
    segs = np.split(neg, np.nonzero(np.diff(neg) > 1)[0] + 1) if len(neg) else []
    for sgm in segs:
        Dg = D[sgm]; grp = sorted({SH_G[int(k)] for row in Dg for k in np.nonzero(row < 0)[0]})
        o4["off_overlap"].append(dict(seed=s, start_step=int(z[sgm[0], 0]), end_step=int(z[sgm[-1], 0]), frames=[int((z[sgm[0], 0] + 1) // 2), int((z[sgm[-1], 0] + 1) // 2)],
                                      groups=grp, min_gap=float(Dg.min())))
out["4_infeasible_overlap"] = o4

# ---------------- 5. η_matched 대안
N = np.load(f"{O}/b1_massA_s0_near.npz")["near"]; e = B.eta_A(N[:, 7]); cc = cost(N[:, 4], e)
key = N[:, 0].astype(np.int64) * 100 + N[:, 1].astype(np.int64); vals = []
for kk in np.unique(key[cc > 0]):
    m = key == kk; j = np.argmax(np.where(m, cc, -1)); vals.append(e[j])
out["5_eta_matched_alt"] = dict(definition="mass A seed 0 근접 기록, 비용 > 0 인 (스텝, 로봇 구)마다 max_all 비용을 낸 형상의 η_A 평균", n_step_sphere=len(vals),
                                mean=float(np.mean(vals)), median=float(np.median(vals)), current_eta_matched=0.1272)
json.dump(out, open(f"{O}/b1_followup.json", "w"), indent=1, ensure_ascii=False, default=lambda x: int(x) if isinstance(x, np.integer) else float(x))
print("ok")
