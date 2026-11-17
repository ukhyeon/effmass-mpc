"""g13_analyze.py — 1-3 A단계 분석 (시스템 python3 -B)
  사용: g13_analyze.py runs <tag...>          → 도달·infeasible·최소 이격·명령 비교
        g13_analyze.py mu <near_tag> [...]    → 실행된 상태에서 사람 형상 0.3 m 안 쌍의 μ 분포 (형상·분절별), 분절 안 방향 범위
        g13_analyze.py eta <near_tag> <json>  → 후보 세트별 η, Δη 표 (json: [{"name":..,"eta0":..,"muref":..,"p":..,"eta_min":..,"eta_max":..}, ...])
"""
import csv, json, os, re, sys
import numpy as np
O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac")
SEG = ["pelvis", "left_thigh", "right_thigh", "abdomen", "left_calf", "right_calf", "left_foot", "right_foot", "chest", "neck",
       "left_shoulder", "right_shoulder", "head", "left_upper_arm", "right_upper_arm", "left_lower_arm", "right_lower_arm", "left_hand", "right_hand"]
BUF = 0.02   # MPC 로봇 구 반지름 버퍼 (step2_scene.COLLISION_SPHERE_BUFFER)


def result(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read()
    rp = re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]
    return json.load(open(rp))


def qcmd(tag):
    r = result(tag); rows = list(csv.DictReader(open(r["steps_csv"])))
    return np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows])


def runs(tags):
    ref = qcmd("g1d_off_1"); base = None
    for t in tags:
        r = result(t); q = qcmd(t); n = min(len(q), len(ref)); m = r.get("meff", {}) or {}; nr = m.get("near") or {}
        out = dict(tag=t, cost=m.get("cost"), arrivals=r["arrivals"], infeasible=r["infeasible_steps"], rtf=r["realtime_factor"], solver_mean=r["solver_ms_mean"],
                   loop_mean=r["loop_ms_mean"], captures=r["graph_captures_by_stage"], ptr=(m.get("channel_pointers_unchanged"), m.get("inputs_pointers_unchanged")),
                   q_equal_g1d_off_1=bool(np.array_equal(q[:n], ref[:n])), q_maxdiff_g1d_off_1=float(np.abs(q[:n] - ref[:n]).max()))
        if os.path.exists(f"{O}/{t}_near.npz"):
            z = np.load(f"{O}/{t}_near.npz"); sm = z["step_min"]; j = int(np.argmin(sm[:, 1]))
            k = int(sm[j, 3]); s = int(sm[j, 2])
            out.update(min_sep_mpc_m=float(sm[j, 1]), min_sep_no_buffer_m=float(sm[j, 1] + BUF), min_step=int(sm[j, 0]), min_sphere=str(z["sphere_link"][s]), min_shape=k,
                       steps_below_0=int((sm[:, 1] < 0).sum()), steps_below_5cm=int((sm[:, 1] < 0.05).sum()), steps_below_10cm=int((sm[:, 1] < 0.10).sum()))
        print(json.dumps(out, ensure_ascii=False))


def load_near(tags):
    rows = []
    for t in tags:
        z = np.load(f"{O}/{t}_near.npz"); n = z["near"]
        if len(n):
            rows.append(n)
    return np.concatenate(rows) if rows else np.zeros((0, 11)), np.load(f"{O}/{tags[0]}_near.npz")["sphere_link"]


def mu(tags):
    N, links = load_near(tags)
    seq = np.load(os.path.expanduser("~/effmass_mpc/goal1/logs/g1_human_seq.npz")); shape_seg = seq["seg"]
    print(f"[mu] runs {tags}, 쌍 {len(N)}, 스텝 {len(np.unique(N[:, 0]))}")
    seg = N[:, 3].astype(int); k = N[:, 2].astype(int); muv = N[:, 7]; mh = N[:, 5]; mr = N[:, 6]
    out = {"by_segment": {}, "by_shape": {}}
    for g in np.unique(seg):
        m = seg == g; fin = np.isfinite(mr[m])
        out["by_segment"][SEG[g]] = dict(n=int(m.sum()), steps=int(len(np.unique(N[m, 0]))), mu_min=float(muv[m].min()), mu_median=float(np.median(muv[m])),
                                         mu_max=float(muv[m].max()), mu_max_over_min=float(muv[m].max() / muv[m].min()), mh_min=float(mh[m].min()), mh_median=float(np.median(mh[m])),
                                         mh_max=float(mh[m].max()), mh_max_over_min=float(mh[m].max() / mh[m].min()),
                                         mr_median_finite=float(np.median(mr[m][fin])) if fin.any() else None, d_min=float(N[m, 4].min()))
    for kk in np.unique(k):
        m = k == kk
        out["by_shape"][f"{kk}:{SEG[shape_seg[kk]]}"] = dict(n=int(m.sum()), mu_min=float(muv[m].min()), mu_median=float(np.median(muv[m])), mu_max=float(muv[m].max()))
    # 분절 관성만으로 본 방향 범위 (재생 전 프레임, Λ_h 고유값): m_h 최대/최소 = λ_max/λ_min
    lam = np.linalg.eigvalsh(seq["inv"].astype(np.float64))          # (F,19,3)
    out["mh_direction_range_all_frames"] = {SEG[g]: dict(mh_min_median=float(np.median(1 / lam[:, g, 2])), mh_max_median=float(np.median(1 / np.maximum(lam[:, g, 0], 1e-12))),
                                                          ratio_median=float(np.median(lam[:, g, 2] / np.maximum(lam[:, g, 0], 1e-12)))) for g in range(19)}
    json.dump(out, open(f"{O}/g13_mu_{'_'.join(tags)}.json", "w"), indent=1)
    print(json.dumps(out, ensure_ascii=False, indent=1))


def eta(tags, cand_json):
    N, links = load_near(tags); cands = json.load(open(cand_json))
    seg = N[:, 3].astype(int); muv = N[:, 7]
    med = lambda g: float(np.median(muv[seg == g])) if (seg == g).any() else None
    ext = lambda g, f: float(f(muv[seg == g])) if (seg == g).any() else None
    pairs = [("손(오른) vs 몸통(가슴)", ("med", 18), ("med", 8)), ("손(오른) vs 몸통(배)", ("med", 18), ("med", 3)), ("상완(오른) vs 몸통(가슴)", ("med", 14), ("med", 8)),
             ("아래팔(오른) vs 몸통(가슴)", ("med", 16), ("med", 8)), ("머리 vs 몸통(가슴)", ("med", 12), ("med", 8))]
    for g in (18, 16, 14, 12, 8, 3):
        pairs.append((f"{SEG[g]} 방향 최대 vs 최소", ("max", g), ("min", g)))
        pairs.append((f"{SEG[g]} 방향 p95 vs p5", ("p95", g), ("p5", g)))
    res = []
    for c in cands:
        f = lambda m: float(np.clip(c["eta0"] * (m / c["muref"]) ** c["p"], c["eta_min"], c["eta_max"]))
        rows = []
        for name, (ka, ga), (kb, gb) in pairs:
            pick = lambda k, g: med(g) if k == "med" else ext(g, {"max": np.max, "min": np.min, "p95": lambda x: np.percentile(x, 95), "p5": lambda x: np.percentile(x, 5)}[k])
            ma = pick(ka, ga); mb = pick(kb, gb)
            if ma is None or mb is None:
                rows.append(dict(pair=name, missing=True)); continue
            rows.append(dict(pair=name, mu_a=ma, mu_b=mb, eta_a=f(ma), eta_b=f(mb), d_eta_cm=100 * (f(mb) - f(ma)) if "vs 몸통" in name or "머리" in name else 100 * (f(ma) - f(mb))))
        pct = np.percentile(muv, [5, 50, 95]); clip_lo = float(np.mean([f(m) <= c["eta_min"] + 1e-12 for m in muv])); clip_hi = float(np.mean([f(m) >= c["eta_max"] - 1e-12 for m in muv]))
        res.append(dict(cand=c, mu_p5_p50_p95=pct.tolist(), eta_at=[f(m) for m in pct], frac_clip_min=clip_lo, frac_clip_max=clip_hi, pairs=rows))
    json.dump(res, open(cand_json.replace(".json", "_result_" + "_".join(tags) + ".json"), "w"), indent=1, ensure_ascii=False)
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "runs":
        runs(sys.argv[2:])
    elif cmd == "mu":
        mu(sys.argv[2:])
    elif cmd == "eta":
        eta(sys.argv[2:-1], sys.argv[-1])
