"""g13_human_precompute.py — 1-3 결정적 재생 (1안): 재생 파일 전 프레임의 HumanInertiaState 내용을 미리 계산해 저장.
실행: g1_run_human.sh py <이 파일> [--out F] [--vdt-from REC.npz]
계산: effmass_human 노드가 쓰는 HumanInertiaCore (설치본 import, 새 구현 없음). 메시지 채우기는 human_inertia_node.compute_callback 과 같은 순서·float32 변환.
  - betas = 재생이 보내는 값 (0 ×10, replay_meta), 첫 입력에서 set_betas 한 번 (노드와 같음)
  - 입력 시각 t_k = k / 30 (재생 일정). 속도 dt = t_k − t_{k−1} (노드: 입력 stamp 차이) → 1/30. 첫 프레임은 속도 0, 무효 (노드와 같음)
  --vdt-from: 대조용. 기록된 노드 메시지의 velocity_dt 를 그대로 써서 계산 (노드와 같은 dt 일 때 같은 값인지 확인)
"""
import argparse, os, sys, time
import numpy as np, torch
torch.set_num_threads(1)
import effmass_human, effmass_human.human_inertia_core as hic
from effmass_human.human_inertia_core import HumanInertiaCore, SEGMENT_NAMES
ap = argparse.ArgumentParser()
ap.add_argument("--npz", default=os.path.expanduser("~/hrc_ws/pose_npz/human_state_1781524848.npz"))
ap.add_argument("--out", default=os.path.expanduser("~/effmass_mpc/goal1/data/human_precomputed_1781524848.npz"))
ap.add_argument("--rate", type=float, default=30.0)
ap.add_argument("--vdt-from", default="")
ap.add_argument("--shapes", default="v1", choices=["v1", "v2"], help="v2: goal1/data/shapes_v2.json (21 + 3 inactive), same node pose path")
ap.add_argument("--pose66-from", default="", help="static pose: take pose66 (1 frame) from this npz and trans from replay frame 0")
a = ap.parse_args()
print("[pre] effmass_human 위치", os.path.dirname(effmass_human.__file__), flush=True)
z = np.load(a.npz); P, T = z["poses"], z["trans"]; F = len(P)
if a.pose66_from:
    P = np.load(a.pose66_from)["pose66"].astype(np.float32); T = T[:1]; F = 1
betas = np.zeros(10, np.float32)
vdt = None
if a.vdt_from:
    r = np.load(a.vdt_from); vdt = {int(f): float(d) for f, d in zip(r["frame"], r["vdt"])}
core = HumanInertiaCore(device="cpu")
if a.shapes == "v2":   # shapes v2: replace the fitted ShapeSet after set_betas (pose path unchanged)
    import json
    from effmass_human.human_shapes import ShapeSet
    core.set_betas(betas)
    V2 = json.load(open(os.path.expanduser("~/effmass_mpc/goal1/data/shapes_v2.json"))); E = V2["shapes"] + [V2["inactive"]] * (24 - len(V2["shapes"]))
    core.shapes = ShapeSet([1] * 24, [e["segment"] for e in E], [e["p0_local"] for e in E], [e["p1_local"] for e in E], [e["radius"] for e in E], core.shapes.distal_local)
f32 = lambda x: x.detach().cpu().numpy().astype(np.float32)
R = {k: [] for k in ["com", "vel", "rot", "inv", "jprox", "jdist", "p0", "p1", "rad", "stype", "sseg", "vdt", "vvalid", "compute_ms"]}
tin = np.arange(F, dtype=np.float64) / a.rate
prev = None; prev_t = None; t0 = time.perf_counter()
for k in range(F):
    tk = time.perf_counter()
    if core.betas is None:
        core.set_betas(betas)
    # 노드: snapshot = np.array(msg.body_full_pose, np.float32) 등 → core.compute(pose, trans, prev, dt)
    pose = np.array(P[k], np.float32); trans = np.array(T[k], np.float32)
    if vdt is not None:
        dt = vdt.get(k, 0.0) if k > 0 else None
        dt = dt if (dt is not None and dt > 0) else (None if k == 0 else (tin[k] - prev_t))
    else:
        dt = (tin[k] - prev_t) if prev_t is not None else None
    out = core.compute(pose, trans, prev=prev, dt=dt)
    sh = out["shapes"]
    R["com"].append(f32(out["com"][0])); R["vel"].append(f32(out["vel"][0])); R["rot"].append(f32(out["world_rot"][0])); R["inv"].append(f32(out["inv_inertia"][0]))
    R["jprox"].append(f32(out["joints"][0])); R["jdist"].append(np.asarray(out["distal"], np.float32))
    R["p0"].append(sh["p0"].astype(np.float32)); R["p1"].append(sh["p1"].astype(np.float32)); R["rad"].append(sh["radius"].astype(np.float32))
    R["stype"].append(np.asarray(sh["type"])); R["sseg"].append(np.asarray(sh["segment"]))
    R["vdt"].append(float(dt) if (dt is not None and out["vel_valid"]) else 0.0); R["vvalid"].append(bool(out["vel_valid"]))
    R["compute_ms"].append((time.perf_counter() - tk) * 1e3)
    prev = out; prev_t = tin[k]
st = R.pop("stype"); sg = R.pop("sseg"); rd = np.stack(R.pop("rad"))
assert all(np.array_equal(st[0], s) for s in st) and all(np.array_equal(sg[0], s) for s in sg) and np.all(rd == rd[0])
out = {k: np.asarray(v) for k, v in R.items()}
out["vdt"] = out["vdt"].astype(np.float32)
np.savez(a.out, input_time=tin, frame=np.arange(F), mass=f32(core.mass_19_body[0]), shape_type=np.asarray(st[0], np.int64), shape_segment=np.asarray(sg[0], np.int64),
         shape_radius=rd[0], segment_names=np.array(SEGMENT_NAMES), rate=a.rate, betas=betas, source_npz=a.npz, vdt_from=a.vdt_from, shapes_version=a.shapes,
         shape_active=(rd[0] > 0), **({"pose66": P} if a.pose66_from else {}), **out)
print(f"[pre] 프레임 {F}, 형상 {len(rd[0])}, 계산 {time.perf_counter() - t0:.1f} s (프레임당 평균 {np.mean(R['compute_ms']):.2f} ms) → {a.out}", flush=True)
