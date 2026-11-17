"""g1_human_validate.py — 1-2b 검증 (재생 데이터, 한 프로세스 안에서 원본 노드와 사본 핵심을 같은 입력으로)
실행: g1_run_human.sh validate (env -i + /opt/ros + ~/hrc_ws/install + ~/effmass_mpc/ws/install, CUDA_VISIBLE_DEVICES=, OMP 1, PYTHONDONTWRITEBYTECODE=1)

  입력: human_state_1781524848.npz (4단계 재생과 같은 파일), betas 0. 프레임 순서에서 7번째마다 하나 건너뜀(건너뛴 자리 = 프레임 차 2)
  원본: hrc_body_tracking.human_dynamics_node.DynamicsNode (설치본, 고치지 않음). 64방향 계산만 인스턴스 속성으로 끔.
        PerfCsvLogger 경로는 goal1/logs 로 돌리고, ~/hrc_ws 아래 쓰기는 감사 훅으로 막음.
        pose_callback(msg) → timer_callback() 을 직접 호출, previous_com_world·link_velocity·latest_meff_snapshot 을 읽음
  사본: effmass_human.human_inertia_core.HumanInertiaCore(cpu) — dt = 실제 프레임 간격(프레임 차/30 s)
  확인:
   (1) 위치: 사본 ↔ 원본, 사본 ↔ 독립 계산(step4_ref_fk, numpy) 1 mm
   (2) 속도: 사본·원본 노드 속도 ↔ 독립 계산 COM 차분 ((COM_k − COM_k−g)·30/g), 프레임 차 g 별 크기 비
   (3) 역관성: 무작위 방향 u 에서 사본 m_h = 1/(uᵀ Λ_s⁻¹ u + 1e-6) ↔ 원본 physics.compute_meff_constrained_from_common(같은 스냅샷, j_eval=s)
       ROM 잠금 끔 = tol_deg −1e9 (near 판정이 항상 거짓), +2 kg 보정 없음. ROM 잠금 켬 (tol_deg 5.0, 원본 질의와 같음) 과의 차이는 기록만
"""
import json, os, sys, time
HOME = os.path.expanduser("~"); HRC = os.path.realpath(f"{HOME}/hrc_ws") + os.sep
LOG = f"{HOME}/effmass_mpc/goal1/logs"
BLOCKED = []
def _audit(event, args):
    try:
        if event == "open":
            path, mode = args[0], args[1]
            if isinstance(mode, str) and any(c in mode for c in "wax+") and isinstance(path, (str, bytes, os.PathLike)) and os.path.realpath(os.fsdecode(path)).startswith(HRC):
                BLOCKED.append(os.fsdecode(path)); raise PermissionError(f"blocked {path}")
        elif event in ("os.mkdir", "os.rename", "os.remove") and os.path.realpath(os.fsdecode(args[0])).startswith(HRC):
            BLOCKED.append(os.fsdecode(args[0])); raise PermissionError(f"blocked {args[0]}")
    except PermissionError:
        raise
    except Exception:
        pass
import hrc_body_tracking.perf_logger as _pl
_orig_init = _pl.PerfCsvLogger.__init__
_pl.PerfCsvLogger.__init__ = lambda self, path, fields, flush_every=30: _orig_init(self, os.path.join(LOG, "g1_validate_orig_" + os.path.basename(path)), fields, flush_every)
sys.addaudithook(_audit)
import numpy as np, torch, rclpy
torch.set_num_threads(1)
from hrc_interfaces.msg import HumanPose
import hrc_body_tracking.human_dynamics_node as ORIG
from effmass_human.human_inertia_core import HumanInertiaCore
sys.path.insert(0, f"{HOME}/effmass_mpc/env_check/scripts"); import step4_ref_fk as F
NPZ = f"{HOME}/hrc_ws/pose_npz/human_state_1781524848.npz"
z = np.load(NPZ); P = z["poses"].astype(np.float32); T = z["trans"].astype(np.float32); N = len(P)
frames = [k for k in range(N) if k % 7 != 6]                       # 7번째마다 건너뜀
rclpy.init()
orig = ORIG.DynamicsNode(); orig.find_worst_vis_enabled = False
core = HumanInertiaCore(device="cpu")
print(f"[val] 원본 device {orig.device}, 사본 device {core.device}, torch threads {torch.get_num_threads()}, 프레임 {len(frames)}/{N} (7번째마다 건너뜀)", flush=True)
betas = np.zeros(10, np.float32)
ref, _, _ = F.com_world(P, T, F.rest_constants(np.zeros(10)))
rng = np.random.default_rng(0)
R = dict(pos_vs_orig=[], pos_vs_ref=[], gap=[], vel_ours=[], vel_orig=[], vel_true=[], m_ours=[], m_off=[], m_on=[], t_ours=[], t_orig=[])
prev = None; prev_k = None
for i, k in enumerate(frames):
    msg = HumanPose(); msg.body_full_pose = P[k].tolist(); msg.global_trans = T[k].tolist(); msg.smpl_betas = betas.tolist()
    msg.header.stamp.sec = int(k // 30); msg.header.stamp.nanosec = int((k % 30) * 1e9 / 30); msg.header.frame_id = f"step4/pose/{k}"
    t0 = time.perf_counter(); orig.pose_callback(msg); orig.timer_callback(); t1 = time.perf_counter()
    if core.betas is None:
        core.set_betas(betas)
    dt = (k - prev_k) / 30.0 if prev_k is not None else None
    out = core.compute(P[k], T[k], prev=prev, dt=dt); t2 = time.perf_counter()
    R["t_orig"].append((t1 - t0) * 1e3); R["t_ours"].append((t2 - t1) * 1e3)
    com_o = orig.previous_com_world[0].cpu().numpy().astype(np.float64); com_s = out["com"][0].cpu().numpy().astype(np.float64)
    R["pos_vs_orig"].append(float(np.abs(com_o - com_s).max())); R["pos_vs_ref"].append(float(np.linalg.norm(com_s - ref[k], axis=1).max()))
    if prev is not None:
        g = k - prev_k; R["gap"].append(g)
        R["vel_true"].append(np.linalg.norm((ref[k] - ref[prev_k]) * 30.0 / g, axis=1))
        R["vel_ours"].append(np.linalg.norm(out["vel"][0].cpu().numpy(), axis=1)); R["vel_orig"].append(np.linalg.norm(orig.link_velocity[0].cpu().numpy(), axis=1))
    if i % 10 == 5:
        Minv, locked, Jp, Jr, pdeg = orig.latest_meff_snapshot
        Lam = out["inv_inertia"][0].cpu().numpy().astype(np.float64)
        for s in range(19):
            for _ in range(3):
                u = rng.normal(size=3); u /= np.linalg.norm(u)
                uf = torch.zeros((1, 19, 3)); uf[0, s] = torch.tensor(u, dtype=torch.float32)
                mo, _ = orig.physics.compute_meff_constrained_from_common(Minv, locked, Jp, Jr, uf, pdeg, orig.rom_cfg, j_eval=s, tol_deg=-1e9)
                mn, _ = orig.physics.compute_meff_constrained_from_common(Minv, locked, Jp, Jr, uf, pdeg, orig.rom_cfg, j_eval=s, tol_deg=5.0)
                uu = u.astype(np.float32).astype(np.float64)
                R["m_ours"].append((s, 1.0 / (uu @ Lam[s] @ uu + 1e-6))); R["m_off"].append(float(mo.reshape(-1)[0])); R["m_on"].append(float(mn.reshape(-1)[0]))
    prev = out; prev_k = k
st = lambda v: dict(n=len(v), mean=float(np.mean(v)), p95=float(np.percentile(v, 95)), max=float(np.max(v)))
res = dict(frames=len(frames), pos_vs_orig_max_m=float(np.max(R["pos_vs_orig"])), pos_vs_ref_max_m=float(np.max(R["pos_vs_ref"])),
           within_1mm=int(sum(x <= 1e-3 for x in R["pos_vs_ref"])), t_ours_ms=st(R["t_ours"]), t_orig_ms=st(R["t_orig"]), blocked=BLOCKED)
gap = np.array(R["gap"]); vt = np.array(R["vel_true"]); vs = np.array(R["vel_ours"]); vo = np.array(R["vel_orig"])
mask = vt > 0.02
for g in sorted(set(gap.tolist())):
    sel = (gap == g)[:, None] & mask
    res[f"vel_ratio_gap{g}"] = dict(pairs=int((gap == g).sum()), ours_median=float(np.median(vs[sel] / vt[sel])), ours_p5=float(np.percentile(vs[sel] / vt[sel], 5)),
                                    ours_p95=float(np.percentile(vs[sel] / vt[sel], 95)), orig_median=float(np.median(vo[sel] / vt[sel])))
mo = np.array(R["m_off"]); ms = np.array([x[1] for x in R["m_ours"]]); mn = np.array(R["m_on"]); seg = np.array([x[0] for x in R["m_ours"]])
rel = np.abs(ms - mo) / mo; relon = (mn - mo) / mo
res["meff_ours_vs_orig_rom_off"] = dict(n=int(len(ms)), rel_max=float(rel.max()), rel_p95=float(np.percentile(rel, 95)), abs_max_kg=float(np.abs(ms - mo).max()))
res["meff_rom_on_vs_off"] = dict(changed_frac=float((np.abs(relon) > 1e-4).mean()), rel_mean=float(relon.mean()), rel_min=float(relon.min()), rel_max=float(relon.max()),
                                 per_segment_mean={F.NAMES_19[s]: float(relon[seg == s].mean()) for s in range(19)})
res["shape_fit"] = {F.NAMES_19[s]: r for s, r in core.shape_fit_report.items()}
res["m_ours_range_kg"] = dict(min=float(ms.min()), median=float(np.median(ms)), max=float(ms.max()))
json.dump(res, open(f"{LOG}/g1_human_validate.json", "w"), indent=1, default=float)
print("[val] 위치: 사본↔원본 최대 차 %.3e m, 사본↔독립 계산 최대 %.4f mm, 1 mm 이내 %d/%d" % (res["pos_vs_orig_max_m"], res["pos_vs_ref_max_m"] * 1000, res["within_1mm"], len(frames)), flush=True)
for g in sorted(set(gap.tolist())):
    v = res[f"vel_ratio_gap{g}"]; print(f"[val] 속도 프레임 차 {g}: 쌍 {v['pairs']}, 사본/실제 중앙 {v['ours_median']:.3f} (p5 {v['ours_p5']:.3f}, p95 {v['ours_p95']:.3f}), 원본/실제 중앙 {v['orig_median']:.3f}", flush=True)
a = res["meff_ours_vs_orig_rom_off"]; b = res["meff_rom_on_vs_off"]
print(f"[val] m_h: 사본 ↔ 원본(ROM 끔) {a['n']}개, 상대 차 최대 {a['rel_max']:.2e} p95 {a['rel_p95']:.2e}, 절대 차 최대 {a['abs_max_kg']:.2e} kg; 값 범위 {res['m_ours_range_kg']}", flush=True)
print(f"[val] ROM 켬 − 끔: 달라진 비율 {100*b['changed_frac']:.1f}%, 상대 차 평균 {100*b['rel_mean']:+.2f}% (범위 {100*b['rel_min']:+.1f}~{100*b['rel_max']:+.1f}%)", flush=True)
print(f"[val] 계산 시간: 사본 {res['t_ours_ms']['mean']:.2f}/p95 {res['t_ours_ms']['p95']:.2f} ms, 원본 {res['t_orig_ms']['mean']:.2f}/p95 {res['t_orig_ms']['p95']:.2f} ms (CPU 1스레드), 막은 쓰기 {len(BLOCKED)}", flush=True)
for s, r in res["shape_fit"].items():
    print(f"[val] 형상 {s}: 형상 {r['n_shapes']}개 반지름 {r['radius_m']} m, 정점 밖 {100*r['outside_frac']:.1f}%, 밖 거리 p95 {1000*r['outside_excess_p95_m']:.1f} mm 최대 {1000*r['outside_excess_max_m']:.1f} mm", flush=True)
orig.destroy_node(); rclpy.shutdown()
