"""g1_meff_cpu_check.py <dump.npz ...> — GPU 채널 값 ↔ CPU 기준 (numpy float64 + 사본 노드 Λ_h(g1_human_seq) + Pinocchio Λ_r·구 중심)
실행: g1_run_human.sh py <이 파일> --dir com|closest --motion static|cv --out <json> dumps...
  CPU 기준은 GPU 코드(g1_meff_gpu)를 쓰지 않고 따로 구현: 구 중심은 rollout 관절값 q[b,h] 로 Pinocchio FK, 반지름 = yml + collision_sphere_buffer,
  사람은 노드 world 배열을 R,t 로 옮김, Λ_r 은 덤프의 현재 관절 상태로 Pinocchio 다시 계산 (horizon 동안 고정)."""
import argparse, json, os, numpy as np, yaml
from effmass_robot.robot_inertia import RobotInertia
ap = argparse.ArgumentParser(); ap.add_argument("--dir", default="com"); ap.add_argument("--motion", default="static"); ap.add_argument("--out", required=True)
ap.add_argument("dumps", nargs="+"); a = ap.parse_args()
H = os.path.expanduser("~"); ri = RobotInertia(rotor=True)
import re   # MPC 는 step2_scene.COLLISION_SPHERE_BUFFER 로 yml 값(0.004)을 덮어씀 (프로토타입 hand_experiment 와 같은 0.02)
buf = float(re.search(r"^COLLISION_SPHERE_BUFFER\s*=\s*([0-9.eE+-]+)", open(f"{H}/effmass_mpc/env_check/scripts/step2_scene.py").read(), re.M).group(1))
RAD = ri.sphere_radius + buf
seq = np.load(f"{H}/effmass_mpc/goal1/logs/g1_human_seq.npz"); tf = json.load(open(f"{H}/effmass_mpc/env_check/step4/temp_transform.json"))
Rm, tv = np.asarray(tf["R"], float), np.asarray(tf["t"], float)
def human(f):
    com = seq["com"][f].astype(float) @ Rm.T + tv; vel = seq["vel"][f].astype(float) @ Rm.T
    inv = np.einsum("ij,sjk,lk->sil", Rm, seq["inv"][f].astype(float), Rm)
    return com, vel, inv, seq["p0"][f].astype(float) @ Rm.T + tv, seq["p1"][f].astype(float) @ Rm.T + tv, seq["rad"].astype(float), seq["seg"]
E = {k: [] for k in ["cost_rel", "cost_abs", "cost_kc_eq", "cost_eta_rel", "cost_pos", "costbh_rel", "costbh_n",
                      "shape_eq", "seg_eq", "dist_abs", "dist_ref", "u_abs", "mh_rel", "mr_rel", "mu_rel", "center_abs", "mr_inf_both", "mr_inf_mismatch", "invr_rel"]}
for p in a.dumps:
    D = np.load(p); age = float(D["age"])
    if "h_com" in D.files:      # Isaac 실행 덤프: 사본 노드가 보낸 메시지 배열(노드 world) + R, t 를 여기서 따로 변환
        Rd, td = D["R"].astype(float), D["t"].astype(float)
        com = D["h_com"].astype(float).reshape(19, 3) @ Rd.T + td; vel = D["h_vel"].astype(float).reshape(19, 3) @ Rd.T
        inv_h = np.einsum("ij,sjk,lk->sil", Rd, D["h_inv"].astype(float).reshape(19, 3, 3), Rd)
        p0 = D["h_p0"].astype(float).reshape(-1, 3) @ Rd.T + td; p1 = D["h_p1"].astype(float).reshape(-1, 3) @ Rd.T + td; rad = D["h_rad"].astype(float); seg = D["h_seg"]
    else:
        com, vel, inv_h, p0, p1, rad, seg = human(int(D["frame"]))
    synth = "q" not in D.files            # g1_meff_near 합성 입력: 구·Λ_r 는 덤프 값 (float32 로 들어간 값을 float64 로)
    if synth:
        inv_r = D["inv_r"].astype(np.float32).astype(np.float64)
    else:
        inv_r = ri.compute(D["q_state"])[1]
        E["invr_rel"].append(float(np.abs(inv_r - D["inv_r"]).max() / np.abs(inv_r).max()))
    for i in range(len(D["b"])):
        if synth:
            C = D["sph"][i][:, :3].astype(np.float64); RADi = D["sph"][i][:, 3].astype(np.float64)
        else:
            C = ri.compute(D["q"][i].astype(float))[0]                      # (61,3) 구 중심 (rollout 관절값)
            E["center_abs"].append(float(np.abs(C - D["sph"][i][:, :3]).max())); RADi = RAD
        tt = age + float(np.sum(D["traj_dt"].astype(np.float64)[1:int(D["h"][i]) + 1])) if a.motion == "cv" else 0.0   # 상태 h 시각 = Σ_{k=1..h} traj_dt_k (cuRobo 적분 커널)
        P0 = p0 + vel[seg] * tt; P1 = p1 + vel[seg] * tt; COM = com + vel * tt
        d = P1 - P0; L2 = np.maximum((d * d).sum(1), 1e-12)
        t = np.clip(((C[:, None] - P0[None]) * d[None]).sum(-1) / L2, 0, 1)   # (61,24)
        Q = P0[None] + t[..., None] * d[None]
        sd = np.linalg.norm(Q - C[:, None], axis=-1) - rad[None] - RADi[:, None]
        k = sd.argmin(1); ar_ = np.arange(len(k)); sg = seg[k]
        fbn = np.linalg.norm(Q[ar_, k] - C, axis=1) < 1e-3        # nearest-point u: COM fallback within 1 mm of the axis
        tgt = COM[sg] if a.dir == "com" else np.where(fbn[:, None], COM[sg], Q[ar_, k])
        E.setdefault("fb_nearest", []).append(int(fbn.sum()) if a.dir != "com" else 0)
        u = tgt - C; u /= np.linalg.norm(u, axis=1, keepdims=True)
        ah = np.einsum("ni,nij,nj->n", u, inv_h[sg], u); arr = np.einsum("ni,nij,nj->n", u, inv_r, u)
        mh, mu = 1 / ah, 1 / (ah + arr)
        with np.errstate(divide="ignore"):
            mr = np.where(arr > 0, 1 / np.where(arr > 0, arr, 1), np.inf)
        g = {kk: D[kk][i] for kk in ["dist", "shape", "seg", "u", "mh", "mr", "mu"]}
        if "cost" in D.files and int(D["cost_mode"]) > 0:      # (1-3) 조합 1 비용: 모든 형상 η_k, c_k → 구별 최댓값(또는 가장 가까운 형상)
            w_, eta0, muref, pw, emin, emax = [float(x) for x in D["cost_par"]]; md, ag = int(D["cost_mode"]), int(D["cost_agg"])
            COMa = COM[seg][None].repeat(len(C), 0); fba = np.linalg.norm(Q - C[:, None], axis=2) < 1e-3
            tgt_all = COMa if a.dir == "com" else np.where(fba[..., None], COMa, Q)
            ua = tgt_all - C[:, None]; ua /= np.linalg.norm(ua, axis=-1, keepdims=True)
            ahk = np.einsum("nki,kij,nkj->nk", ua, inv_h[seg], ua); ark = np.einsum("nki,nij,nkj->nk", ua, inv_r, ua)
            if md == 3:      # (B1) iso: m_h = ISO 고정 (분절), 발 = 종아리 (가정)
                _j = json.load(open(f"{H}/effmass_mpc/goal1/data/iso_15066.json")); _nm = ["pelvis", "left_thigh", "right_thigh", "abdomen", "left_calf", "right_calf", "left_calf", "right_calf", "chest", "neck", "left_shoulder", "right_shoulder", "head", "left_upper_arm", "right_upper_arm", "left_lower_arm", "right_lower_arm", "left_hand", "right_hand"]
                ahk = np.broadcast_to((1.0 / np.array([_j[n]["m_h_iso"] for n in _nm]))[seg][None], ahk.shape)
            if md == 1:
                etak = np.full_like(sd, eta0)
            elif md == 4:   # (V1) gain mode: eta0 = d_base, pw = k_gain, emin = d_base
                etak = np.clip(eta0 + pw * np.maximum(np.sqrt(1.0 / (ahk + ark) / muref) - 1.0, 0.0), emin, emax)
            else:
                etak = np.clip(eta0 * (1.0 / (ahk + ark) / muref) ** pw, emin, emax)
            ck = np.where(sd < 0, 0.5 * etak - sd, np.where(sd < etak, (etak - sd) ** 2 * (0.5 / etak), 0.0))
            if ag == 1:
                cs = ck[ar_, k]; kc = np.where(cs > 0, k, -1); es = etak[ar_, k]
            else:
                kc = ck.argmax(1); cs = ck[ar_, kc]; es = etak[ar_, kc]; kc = np.where(cs > 0, kc, -1)
            cs = w_ * cs; gc = D["cost"][i].astype(np.float64)
            pos = cs > 0
            E["cost_abs"] += np.abs(gc - cs).tolist(); E["cost_rel"] += (np.abs(gc - cs)[pos] / cs[pos]).tolist(); E["cost_pos"].append(int(pos.sum()))
            big = cs >= 1.0                      # 가중 비용 1 이상 (w = 1e5 이면 c ≥ 1e-5)
            E.setdefault("cost_rel_ge1", []).extend((np.abs(gc - cs)[big] / cs[big]).tolist()); E.setdefault("cost_ref_max", []).append(float(cs.max()))
            E.setdefault("cost_rel_small_ref", []).extend(cs[pos][(np.abs(gc - cs)[pos] / cs[pos]) > 1e-4].tolist())
            E["cost_kc_eq"] += (D["kc"][i][pos] == kc[pos]).tolist(); E["cost_eta_rel"] += (np.abs(D["eta"][i][pos] - es[pos]) / es[pos]).tolist()
            if "cost_bh" in D.files and cs.sum() > 0:
                E["costbh_rel"].append(abs(float(D["cost_bh"][i]) - cs.sum()) / cs.sum())
                E.setdefault("costbh_ref", []).append(float(cs.sum()))   # (v2) reference sum, to see where the largest relative errors are
        E["shape_eq"] += (g["shape"] == k).tolist(); E["seg_eq"] += (g["seg"] == sg).tolist()
        m = g["shape"] == k
        E["dist_abs"] += np.abs(g["dist"] - sd[ar_, k])[m].tolist(); E["dist_ref"] += sd[ar_, k][m].tolist()
        E["u_abs"] += np.abs(g["u"] - u).max(1)[m].tolist()
        E["mh_rel"] += (np.abs(g["mh"] - mh) / mh)[m].tolist(); E["mu_rel"] += (np.abs(g["mu"] - mu) / mu)[m].tolist()
        fin = np.isfinite(mr) & np.isfinite(g["mr"])
        with np.errstate(invalid="ignore"):
            E["mr_rel"] += (np.abs(g["mr"] - mr) / mr)[m & fin].tolist()
        E["mr_inf_both"] += (~np.isfinite(mr) & ~np.isfinite(g["mr"]))[m].tolist(); E["mr_inf_mismatch"] += (np.isfinite(mr) != np.isfinite(g["mr"]))[m].tolist()
st = lambda v: dict(n=len(v), max=float(np.max(v)), p99=float(np.percentile(v, 99)), median=float(np.median(v))) if len(v) else None
dr = np.abs(np.array(E["dist_ref"])); da = np.array(E["dist_abs"])
res = dict(sphere_buffer_m=buf, dumps=a.dumps, dir=a.dir, motion=a.motion, samples=len(E["shape_eq"]), shape_match=float(np.mean(E["shape_eq"])), seg_match=float(np.mean(E["seg_eq"])),
           shape_mismatch=int(len(E["shape_eq"]) - np.sum(E["shape_eq"])), center_abs_m=st(E["center_abs"]), inv_r_rel=max(E["invr_rel"]) if E["invr_rel"] else None,
           dist_ref_min_m=float(np.min(E["dist_ref"])), dist_n_negative=int((np.array(E["dist_ref"]) < 0).sum()),
           dist_abs_m=st(E["dist_abs"]), dist_rel_where_ref_ge_1cm=st((da / np.maximum(dr, 1e-12))[dr >= 0.01]), dist_n_ref_lt_1cm=int((dr < 0.01).sum()),
           u_abs=st(E["u_abs"]), mh_rel=st(E["mh_rel"]), mr_rel_finite=st(E["mr_rel"]), mu_rel=st(E["mu_rel"]),
           mr_inf_both=int(np.sum(E["mr_inf_both"])), mr_inf_mismatch=int(np.sum(E["mr_inf_mismatch"])),
           cost=dict(n_positive=int(np.sum(E["cost_pos"])), rel=st(E["cost_rel"]), abs_all_max=float(max(E["cost_abs"])) if E["cost_abs"] else None,
                     shape_match_positive=float(np.mean(E["cost_kc_eq"])) if E["cost_kc_eq"] else None, eta_rel=st(E["cost_eta_rel"]),
                     sum_bh_rel=st(E["costbh_rel"]), sum_bh_worst=(lambda r, v: dict(rel=float(np.max(r)), ref_sum=float(v[int(np.argmax(r))]), rel_where_sum_ge_10=float(np.max(np.array(r)[np.array(v) >= 10])) if (np.array(v) >= 10).any() else None))(E["costbh_rel"], E.get("costbh_ref", [0])) if E["costbh_rel"] else None, rel_where_cost_ge_1=st(E.get("cost_rel_ge1", [])), cost_ref_max=max(E.get("cost_ref_max", [0])),
                     n_rel_gt_1e4=len(E.get("cost_rel_small_ref", [])), ref_cost_of_rel_gt_1e4_max=max(E.get("cost_rel_small_ref", [0]))) if E["cost_abs"] else None)
# 형상이 다르게 뽑힌 표본: 두 형상 거리 차 (동률 근처인지)
json.dump(res, open(a.out, "w"), indent=1)
print("[cpu-check]", json.dumps(res, ensure_ascii=False))
