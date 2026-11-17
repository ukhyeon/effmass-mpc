"""g1_human_seq.py — 재생 파일 전 프레임을 사본 노드 핵심(HumanInertiaCore, CPU 1스레드)으로 계산해 HumanInertiaState 와 같은 배열로 저장 (노드 world)
실행: g1_run_human.sh py <이 파일> --out <npz>"""
import argparse, os, numpy as np, torch
torch.set_num_threads(1)
from effmass_human.human_inertia_core import HumanInertiaCore
ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); a = ap.parse_args()
z = np.load(os.path.expanduser("~/hrc_ws/pose_npz/human_state_1781524848.npz")); P, T = z["poses"].astype(np.float32), z["trans"].astype(np.float32)
c = HumanInertiaCore("cpu"); c.set_betas(np.zeros(10, np.float32)); prev = None; R = {k: [] for k in ["com", "vel", "inv", "p0", "p1"]}
for k in range(len(P)):
    o = c.compute(P[k], T[k], prev=prev, dt=1 / 30 if prev is not None else None); prev = o
    R["com"].append(o["com"][0].numpy()); R["vel"].append(o["vel"][0].numpy()); R["inv"].append(o["inv_inertia"][0].numpy())
    R["p0"].append(o["shapes"]["p0"]); R["p1"].append(o["shapes"]["p1"])
np.savez(a.out, **{k: np.asarray(v, np.float32) for k, v in R.items()}, rad=c.shapes.radius.astype(np.float32), seg=c.shapes.segs.astype(np.int64),
         type=c.shapes.types, mass=c.mass_19_body[0].numpy())
print("[seq] frames", len(P), "shapes", len(c.shapes.radius))
