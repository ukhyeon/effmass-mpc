"""g13_precompute_check.py — 미리 계산한 사람 프레임 ↔ 기록된 실제 노드 메시지 대조 (시스템 python3 -B)
  1) live 기록 (g1_human_rec, 노드 메시지 com·vel·inv, 입력 프레임 번호): 같은 입력 프레임끼리
  2) Isaac 덤프 h_* (Isaac 이 받은 노드 메시지 원본 배열, 형상 포함): com 이 가장 가까운 프레임으로 짝짓고 그 프레임과 전 항목 비교"""
import glob, json, os, sys
import numpy as np
G = os.path.expanduser("~/effmass_mpc/goal1")
pre = np.load(f"{G}/data/human_precomputed_1781524848.npz"); chk = np.load(f"{G}/data/check_vdt_nodrop.npz")
res = {}
def md(a, b):
    a = np.asarray(a, np.float64); b = np.asarray(b, np.float64); d = np.abs(a - b)
    return dict(max_abs=float(d.max()), max_rel_to_range=float(d.max() / max(np.abs(b).max(), 1e-30)), n_exact=int((d == 0).sum()), n=int(d.size))
for run in ["g1_live_nodrop", "g1_live_drop7"]:
    r = np.load(f"{G}/logs/{run}/rec.npz"); fr = r["frame"]
    ok = fr >= 0; fr = fr[ok]
    out = dict(messages=int(len(fr)), unique_frames=int(len(np.unique(fr))), com=md(pre["com"][fr], r["com"][ok]), inv=md(pre["inv"][fr], r["inv"][ok]))
    v = r["vvalid"][ok].astype(bool)
    out["vel_valid_both"] = int((v & pre["vvalid"][fr]).sum())
    out["vel_precomputed_dt_1_30"] = md(pre["vel"][fr][v], r["vel"][ok][v])
    out["node_vdt_minus_1_30_ms"] = dict(min=float((r["vdt"][ok][v] - 1 / 30).min() * 1e3), max=float((r["vdt"][ok][v] - 1 / 30).max() * 1e3))
    if run == "g1_live_nodrop":
        out["vel_same_dt_as_node"] = md(chk["vel"][fr][v], r["vel"][ok][v])
        out["vdt_same"] = md(chk["vdt"][fr][v], r["vdt"][ok][v])
    res[run] = out
dumps = sorted(glob.glob(f"{G}/logs/meff_isaac/g1d*_dump_*.npz"))
rows = []
for p in dumps:
    z = np.load(p)
    if "h_com" not in z.files:
        continue
    c = z["h_com"].reshape(19, 3)
    j = int(np.argmin(np.abs(pre["com"] - c[None]).reshape(len(pre["com"]), -1).max(1)))
    e = dict(dump=os.path.basename(p), frame=j)
    for k, kp in [("h_com", "com"), ("h_inv", "inv"), ("h_p0", "p0"), ("h_p1", "p1")]:
        e[k] = float(np.abs(z[k].reshape(pre[kp][j].shape).astype(np.float64) - pre[kp][j]).max())
    e["h_rad"] = float(np.abs(z["h_rad"] - pre["shape_radius"]).max()); e["seg_type_equal"] = bool(np.array_equal(z["h_seg"], pre["shape_segment"]) and np.array_equal(z["h_type"], pre["shape_type"]))
    e["h_vel_vs_dt_1_30"] = float(np.abs(z["h_vel"].reshape(19, 3) - pre["vel"][j]).max()); e["vel_scale"] = float(np.abs(pre["vel"][j]).max())
    rows.append(e)
res["isaac_dumps"] = rows
res["isaac_dumps_max"] = {k: max(r[k] for r in rows) for k in ["h_com", "h_inv", "h_p0", "h_p1", "h_rad"]}
json.dump(res, open(f"{G}/logs/g13_precompute_check.json", "w"), indent=1)
print(json.dumps(res, indent=1))
