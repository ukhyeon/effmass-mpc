"""mesh_gap.py - mesh-based gap for executed runs (offline; run with g1_run_human.sh py so effmass_robot is importable).
For each step: robot sphere centres from the measured joint values (steps csv) with RobotInertiaNP (same 61 spheres as the MPC channel),
human mesh = node SMPL-H vertices of the frame used at that step ({tag}_human_step_frames.npy), transformed with the run's temp_transform.
Gap = distance from the sphere centre to the body-part mesh surface minus the sphere radius (MPC radius = yml + 0.02 buffer; also without buffer).
Negative when the centre is inside the mesh (winding number). Body parts = node vertex partition (triangle -> majority vertex label).
Usage: mesh_gap.py <tag> [--human-dir DIR] [--verts FILE] -> logs/meff_isaac/<tag>_mesh_gap.json (+ .npz per step)"""
import argparse, csv, json, os, re, sys
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, os.path.dirname(__file__))
import shape_metrics as SM
from effmass_robot.robot_inertia_np import RobotInertiaNP
H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"
ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("--human-dir", default=""); ap.add_argument("--verts", default=""); ap.add_argument("--thr", type=float, default=0.3)
a = ap.parse_args()
log = open(f"{O}/{a.tag}.log", errors="replace").read()
res = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))
pre = re.findall(r"미리 계산한 프레임 (\S+) \(", log)[-1]; pre = os.path.expanduser(pre)
base = pre[:-len("_v2.npz")] + ".npz" if pre.endswith("_v2.npz") else pre
verts = a.verts or base.replace(".npz", "_verts.npz")
hd = a.human_dir or (re.findall(r"--human-dir (\S+)", log) or ["~/effmass_mpc/env_check/step4"])[-1]
tf = json.load(open(os.path.join(os.path.expanduser(hd), "temp_transform.json"))); Rm, tv = np.array(tf["R"]), np.array(tf["t"])
VV = np.load(verts, mmap_mode="r")["verts"]; FACES = np.load(verts)["faces"]
LAB = np.load(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy")
GROUPS = {"hand": [17, 18], "forearm": [15, 16], "upperarm": [13, 14], "torso": [0, 3, 8], "head": [12], "legs": [1, 2, 4, 5, 6, 7], "neck_shoulder": [9, 10, 11]}
tl = np.array([np.bincount(LAB[f][LAB[f] >= 0], minlength=19).argmax() if (LAB[f] >= 0).any() else -1 for f in FACES])      # triangle -> segment
VT = [[] for _ in range(len(LAB))]
for fi, f in enumerate(FACES):
    for v in f:
        VT[v].append(fi)
sf = np.load(f"{O}/{a.tag}_human_step_frames.npy")            # (step, frame, age), step -1 = before warm-up
fr_of = {int(s): int(f) for s, f, _ in sf}
rows = list(csv.DictReader(open(res["steps_csv"]))); Q = np.array([[float(r[f"q{i}"]) for i in range(7)] for r in rows]); steps = np.array([int(r["step"]) for r in rows])
RI = RobotInertiaNP(rotor=True); RAD = RI.sphere_radius + 0.02
out = {g: np.full(len(steps), np.inf) for g in GROUPS}; arg = {g: np.full(len(steps), -1) for g in GROUPS}; cache_f = None
for i, (st, q) in enumerate(zip(steps, Q)):
    f = fr_of.get(int(st), fr_of[max(fr_of)])
    if f != cache_f:
        V = np.asarray(VV[f], float) @ Rm.T + tv; TRI = V[FACES]; cache_f = f
        KDG = {g: (cKDTree(V[np.isin(LAB, ss)]), np.nonzero(np.isin(LAB, ss))[0]) for g, ss in GROUPS.items()}
    C = RI.compute(q)[0]
    for g, ss in GROUPS.items():
        kd, vid = KDG[g]; dv, _ = kd.query(C)
        near = np.nonzero(dv - RAD < a.thr)[0]
        if not len(near):
            continue
        _, idx = kd.query(C[near], k=12); cand = np.zeros((len(near), 64), np.int64)
        for j, row in enumerate(idx):
            t = sorted({x for v in vid[row] for x in VT[v] if tl[x] in ss})[:64] or sorted({x for v in vid[row] for x in VT[v]})[:64]
            cand[j, :len(t)] = t; cand[j, len(t):] = t[0]
        d = SM.pt_tri_batch(C[near], TRI[cand])
        close = d < RAD[near] + 0.05
        if close.any():
            ins = SM.winding(C[near][close], TRI) >= 0.5; dd = d[close]; dd[ins] = -dd[ins]; d[close] = dd
        gap = d - RAD[near]; j = int(np.argmin(gap)); out[g][i] = gap[j]; arg[g][i] = near[j]
summ = {}
for g in GROUPS:
    v = out[g]
    if np.isfinite(v).any():
        i = int(np.nanargmin(np.where(np.isfinite(v), v, np.nan))); summ[g] = dict(min_gap_m=float(v[i]), min_gap_no_buffer_m=float(v[i] + 0.02), step=int(steps[i]), sphere=str(RI.sphere_link[arg[g][i]]),
                                                                                  steps_negative=int((v < 0).sum()))
    else:
        summ[g] = None
allm = np.min(np.stack([out[g] for g in GROUPS]), 0)
summ["overall"] = dict(min_gap_m=float(allm.min()), step=int(steps[int(np.argmin(allm))]), steps_negative=int((allm < 0).sum()))
json.dump(dict(tag=a.tag, human_pre=pre, verts=verts, human_dir=hd, sphere_radius="yml + 0.02 (MPC)", summary=summ), open(f"{O}/{a.tag}_mesh_gap.json", "w"), indent=1)
np.savez(f"{O}/{a.tag}_mesh_gap.npz", steps=steps, **{g: out[g] for g in GROUPS})
print(json.dumps(summ))
