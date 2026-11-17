"""b1_analyze.py — 1-3 B1 (부위 효과 짝 비교) 분석 (시스템 python3 -B)
  eta_matched <near_tag>          : mass A 실행 근접 기록(0.3 m)에서 (스텝, 로봇 구)마다 가장 가까운 형상의 η_A 평균
  run <tag...>                    : 실행별 지표 → json 한 줄씩
  all                             : 조건 4 × seed 0~7 지표·짝 차이·범위·과제 표 → logs/meff_isaac/b1_summary.json, 그림 2개
간격 = MPC 로봇 구(yml 반지름 + 버퍼 0.02) 표면 ~ 사람 형상 표면 [m]. 버퍼 제외 간격 = 값 + 0.02
"""
import csv, json, os, re, sys
import numpy as np
O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac"); FIG = os.path.expanduser("~/effmass_mpc/goal1/b1")
PRE = np.load(os.path.expanduser("~/effmass_mpc/goal1/data/human_precomputed_1781524848.npz")); SHSEG = PRE["shape_segment"]
GROUPS = {"hand": [17, 18], "forearm": [15, 16], "upperarm": [13, 14], "torso": [0, 3, 8], "head": [12], "legs": [1, 2, 4, 5, 6, 7], "other_neck_shoulder": [9, 10, 11]}
GSH = {g: [k for k in range(24) if SHSEG[k] in segs] for g, segs in GROUPS.items()}
GKO = {"hand": "손", "forearm": "아래팔", "upperarm": "위팔", "torso": "몸통", "head": "머리", "legs": "다리", "other_neck_shoulder": "목·어깨"}
CONDS = ["off", "uni10", "unim", "iso", "massA"]; CKO = {"off": "off", "uni10": "uniform 0.10", "unim": "uniform η_matched", "iso": "iso (ISO 15066 고정)", "massA": "mass A"}
ISO_NM = ["pelvis", "left_thigh", "right_thigh", "abdomen", "left_calf", "right_calf", "left_calf", "right_calf", "chest", "neck", "left_shoulder", "right_shoulder", "head",
          "left_upper_arm", "right_upper_arm", "left_lower_arm", "right_lower_arm", "left_hand", "right_hand"]   # 발 → 종아리 (가정)
M_ISO = np.array([json.load(open(os.path.expanduser("~/effmass_mpc/goal1/data/iso_15066.json")))[n]["m_h_iso"] for n in ISO_NM], float)


def mu_iso(seg, mr):
    return 1.0 / (1.0 / M_ISO[np.asarray(seg, int)] + np.where(np.isfinite(mr) & (mr > 0), 1.0 / np.where(mr > 0, mr, 1.0), 0.0))
A = dict(eta0=0.10, muref=2.0, p=0.5, eta_min=0.05, eta_max=0.20)
DT = 1.0 / 60.0


def eta_A(mu):
    return np.clip(A["eta0"] * (np.asarray(mu, float) / A["muref"]) ** A["p"], A["eta_min"], A["eta_max"])


def result(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read()
    return json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))


def eta_matched(tag):
    z = np.load(f"{O}/{tag}_near.npz"); N = z["near"]            # (step, s, k, seg, d, mh, mr, mu, ux, uy, uz)
    key = N[:, 0] * 1000 + N[:, 1]
    order = np.lexsort((N[:, 4], key)); Ns = N[order]; ks = key[order]
    first = np.r_[True, ks[1:] != ks[:-1]]; Nn = Ns[first]          # (스텝, 구)마다 가장 가까운 형상 행
    e = eta_A(Nn[:, 7])
    act = N[N[:, 4] < eta_A(N[:, 7])]                              # 비용이 켜진 쌍 (d < η)
    out = dict(tag=tag, n_step_sphere=int(len(Nn)), n_steps=int(len(np.unique(Nn[:, 0]))), eta_matched=float(e.mean()), eta_median=float(np.median(e)),
               eta_p5_p95=np.percentile(e, [5, 95]).tolist(), frac_clip_min=float((e <= A["eta_min"] + 1e-12).mean()), frac_clip_max=float((e >= A["eta_max"] - 1e-12).mean()),
               alt_active_pairs=dict(n=int(len(act)), eta_mean=float(eta_A(act[:, 7]).mean()) if len(act) else None),
               by_group_mean={g: float(e[np.isin(Nn[:, 2].astype(int), GSH[g])].mean()) if np.isin(Nn[:, 2].astype(int), GSH[g]).any() else None for g in GSH})
    return out


def run_metrics(tag, cond):
    r = result(tag); z = np.load(f"{O}/{tag}_b1.npz"); R = z["rows"]; links = z["sphere_link"]; par = z["cost_par"]
    step = R[:, 0].astype(int); q = R[:, 1:8]; ee = R[:, 8:11]; D = R[:, 12:36]; Sm = R[:, 36:60].astype(int); MU = R[:, 60:84]; MH = R[:, 84:108]; MR = R[:, 108:132]
    out = dict(tag=tag, cond=cond, seed=int(z["mppi_seed"]), steps=int(len(R)), cost_par=par.tolist(), mppi_seed_info=r.get("mppi_seed_info"))
    for g, ks in GSH.items():
        sub = D[:, ks]; j = np.unravel_index(int(np.argmin(sub)), sub.shape); k = ks[j[1]]; i = j[0]
        mu = float(MU[i, k]); mui = float(mu_iso(SHSEG[k], MR[i, k]))
        eta = float(eta_A(mu)) if cond == "massA" else float(eta_A(mui)) if cond == "iso" else (float(par[1]) if cond in ("uni10", "unim") else None)
        out[g] = dict(min_gap=float(sub[j]), step=int(step[i]), t_s=float((step[i] + 1) * DT), shape=int(k), seg=int(SHSEG[k]), sphere=int(Sm[i, k]), sphere_link=str(links[Sm[i, k]]),
                      mu=mu, mu_mass=mu, mu_iso=mui, mh_dir=float(MH[i, k]), m_iso=float(M_ISO[SHSEG[k]]), mr=float(MR[i, k]), eta=eta)
    allmin = D.min(1)
    out["overall"] = dict(min_gap=float(allmin.min()), step=int(step[np.argmin(allmin)]), steps_negative=int((allmin < 0).sum()))
    arr = r.get("arrival_list", [])
    out["task"] = dict(arrivals=int(r["arrivals"]), arrival_t_s=[round((a["step"] + 1) * DT, 3) for a in arr], arrival_goal=[a["goal"] for a in arr],
                       ee_path_m=float(np.linalg.norm(np.diff(ee, axis=0), axis=1).sum()),
                       dq_mean=float(np.linalg.norm(np.diff(q, axis=0), axis=1).mean()), dq_p95=float(np.percentile(np.linalg.norm(np.diff(q, axis=0), axis=1), 95)),
                       infeasible=int(r["infeasible_steps"]), solver_mean=float(r["solver_ms_mean"]), solver_p95=float(r["solver_ms_p95"]), rtf=float(r["realtime_factor"]),
                       captures=r["graph_captures_by_stage"])
    return out


def sign_p(d):
    from scipy.stats import binomtest
    d = np.asarray(d); n = int((d != 0).sum()); k = int((d > 0).sum())
    return float(binomtest(k, n, 0.5).pvalue) if n else None


def all_(tags_fmt="b1_{c}_s{s}", seeds=range(8)):
    M = {c: {s: run_metrics(tags_fmt.format(c=c, s=s), c) for s in seeds if os.path.exists(f"{O}/{tags_fmt.format(c=c, s=s)}_b1.npz")} for c in CONDS}
    S = dict(runs=M, paired={}, ranges={}, task={})
    for a, b in [("massA", "unim"), ("massA", "uni10"), ("massA", "iso"), ("iso", "unim")]:
        for g in GSH:
            ss = sorted(set(M[a]) & set(M[b]))
            if not ss:
                continue
            d = np.array([M[a][s][g]["min_gap"] - M[b][s][g]["min_gap"] for s in ss])
            S["paired"][f"{a}-{b}:{g}"] = dict(n=len(ss), mean=float(d.mean()), std=float(d.std(ddof=1)) if len(d) > 1 else None, n_pos=int((d > 0).sum()), n_neg=int((d < 0).sum()),
                                               n_zero=int((d == 0).sum()), sign_p=sign_p(d), per_seed=dict(zip(map(int, ss), d.round(5).tolist())))
    for g in GSH:
        S["ranges"][g] = {c: [float(min(M[c][s][g]["min_gap"] for s in M[c])), float(max(M[c][s][g]["min_gap"] for s in M[c]))] for c in CONDS if M[c]}
    keys = ["arrivals", "ee_path_m", "dq_mean", "dq_p95", "infeasible", "solver_mean", "solver_p95"]
    for c in CONDS:
        if M[c]:
            S["task"][c] = {k: float(np.mean([M[c][s]["task"][k] for s in M[c]])) for k in keys}
            S["task"][c]["neg_steps"] = float(np.mean([M[c][s]["overall"]["steps_negative"] for s in M[c]]))
            S["task"][c]["arrival_first3_t_mean"] = [float(np.mean([M[c][s]["task"]["arrival_t_s"][j] for s in M[c] if len(M[c][s]["task"]["arrival_t_s"]) > j]))
                                                     if any(len(M[c][s]["task"]["arrival_t_s"]) > j for s in M[c]) else None for j in range(8)]
    for c in CONDS:
        if c in S["task"] and "off" in S["task"]:
            S["task"][c]["minus_off"] = {k: S["task"][c][k] - S["task"]["off"][k] for k in keys + ["neg_steps"]}
    # 해석: 최소 간격 순간 μ_mass vs μ_iso (같은 분절·같은 로봇 구 m_r), 조건 massA·iso 의 seed 중앙값
    S["mu_at_min"] = {g: {c: dict(mu_mass_median=float(np.median([M[c][s][g]["mu_mass"] for s in M[c]])), mu_iso_median=float(np.median([M[c][s][g]["mu_iso"] for s in M[c]])),
                                  mh_dir_median=float(np.median([M[c][s][g]["mh_dir"] for s in M[c]])), m_iso=sorted({M[c][s][g]["m_iso"] for s in M[c]}),
                                  segs=sorted({M[c][s][g]["seg"] for s in M[c]})) for c in ("massA", "iso") if M[c]} for g in GSH}
    # 해석: 근접 기록(0.3 m, mass A 전 seed)에서 분절별 방향 m_h vs ISO m_h_iso, μ_mass vs μ_iso
    rows = [np.load(f"{O}/b1_massA_s{s}_near.npz")["near"] for s in M["massA"] if os.path.exists(f"{O}/b1_massA_s{s}_near.npz")]
    if rows:
        N = np.concatenate(rows); sg = N[:, 3].astype(int); mh = N[:, 5]; mr = N[:, 6]; mu = N[:, 7]; mi = mu_iso(sg, mr)
        S["dir_vs_iso_by_segment"] = {PRE["segment_names"][g]: dict(n=int((sg == g).sum()), m_iso=float(M_ISO[g]), mh_median=float(np.median(mh[sg == g])),
                                      mh_p5_p95=np.percentile(mh[sg == g], [5, 95]).tolist(), frac_mh_gt_iso=float((mh[sg == g] > M_ISO[g]).mean()),
                                      mu_mass_median=float(np.median(mu[sg == g])), mu_iso_median=float(np.median(mi[sg == g])),
                                      eta_mass_median=float(np.median(eta_A(mu[sg == g]))), eta_iso_median=float(np.median(eta_A(mi[sg == g]))))
                                      for g in np.unique(sg)}
    json.dump(S, open(f"{O}/b1_summary.json", "w"), indent=1, ensure_ascii=False)
    figs(M)
    return S


def figs(M):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["DejaVu Sans"]
    gl = ["hand", "forearm", "upperarm", "torso", "head", "legs"]
    fig, ax = plt.subplots(1, len(gl), figsize=(3.0 * len(gl), 4.2), sharey=False)
    for a, g in zip(ax, gl):
        seeds = sorted(set().union(*[set(M[c]) for c in CONDS if M[c]]))
        for s in seeds:
            cc = [c for c in CONDS if s in M[c]]
            y = [M[c][s][g]["min_gap"] * 100 for c in cc]
            a.plot([CONDS.index(c) for c in cc], y, "-o", ms=4, lw=0.8, alpha=0.75, label=f"seed {s}")
        a.set_xticks(range(len(CONDS))); a.set_xticklabels(["off", "uni 0.10", "uni matched", "iso", "mass A"], rotation=35, fontsize=8)
        a.set_title(g); a.grid(alpha=0.3)
    ax[0].set_ylabel("min surface gap [cm] (MPC spheres)"); ax[-1].legend(fontsize=7, loc="best")
    fig.suptitle("B1: per-group minimum gap by condition (lines = same MPPI seed)"); fig.tight_layout()
    fig.savefig(f"{FIG}/b1_fig1_group_min_gap.png", dpi=130); plt.close(fig)
    z = np.load(f"{O}/b1_massA_s0_b1.npz"); R = z["rows"]; D = R[:, 12:36]; MU = R[:, 60:84]; t = (R[:, 0] + 1) * DT
    k = D.argmin(1); dmin = D[np.arange(len(D)), k]; eta = eta_A(MU[np.arange(len(D)), k])
    grp = np.array([next(g for g in GSH if kk in GSH[g]) for kk in k])
    fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    ax[0].plot(t, dmin * 100, lw=0.8, color="k", label="nearest gap (all shapes)")
    ax[0].plot(t, eta * 100, lw=0.8, color="tab:red", label="active eta (nearest pair, mass A)")
    ax[0].set_ylabel("cm"); ax[0].set_ylim(0, 60); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    cols = {"hand": "tab:orange", "forearm": "tab:olive", "upperarm": "tab:green", "torso": "tab:blue", "head": "tab:purple", "legs": "tab:gray", "other_neck_shoulder": "tab:brown"}
    for g in cols:
        m = grp == g
        if m.any():
            ax[1].scatter(t[m], (dmin - eta)[m] * 100, s=2, color=cols[g], label=g)
    ax[1].axhline(0, color="k", lw=0.6); ax[1].set_ylabel("gap - eta [cm]"); ax[1].set_xlabel("sim time [s]"); ax[1].set_ylim(-15, 50); ax[1].legend(fontsize=7, markerscale=4, ncol=4)
    ax[1].grid(alpha=0.3); fig.suptitle("mass A, seed 0: nearest gap and active eta over time"); fig.tight_layout()
    fig.savefig(f"{FIG}/b1_fig2_massA_s0_gap_eta.png", dpi=130); plt.close(fig)


if __name__ == "__main__":
    c = sys.argv[1]
    if c == "eta_matched":
        print(json.dumps(eta_matched(sys.argv[2]), ensure_ascii=False, indent=1))
    elif c == "run":
        for t in sys.argv[2:]:
            print(json.dumps(run_metrics(t, t.split("_")[1]), ensure_ascii=False))
    elif c == "all":
        S = all_(); print(json.dumps({k: S[k] for k in ("paired", "ranges", "task")}, ensure_ascii=False, indent=1))
