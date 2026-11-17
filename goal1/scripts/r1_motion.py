"""r1_motion.py - R1 moving-hand data (system python via g1_run_human.sh py). Hold pose = B2a static pose (human_b2a_reach_v2 pose66, wrist 0).
Retracted poses: the fingertip (node distal point of the right hand) moved back along the reach direction (shoulder -> fingertip of the hold pose) by s,
right shoulder + elbow (2 x 3 axis-angle) optimised with the node body model (HumanInertiaCore): fingertip on the line (dominant), palm orientation kept,
forearm toward the front (soft), continuity with the previous key pose. Key poses every 1 cm; per-frame poses interpolated linearly between key poses.
Retract distance D: smallest of 0.30, 0.32, ... m with every v2 hand shape >= 0.35 m from the robot spheres (MPC radius) of the V1 uniform seed 0 front run
at the retracted pose. Cycle: rest (retracted) 1.5 s -> reach (min-jerk, zero end speeds) -> hold 1.5 s -> retract -> ..., 60 s, 30 fps; reach time set
so that the fingertip peak speed is 0.25 / 0.5 / 1.0 m/s (one rescale from the measured peak). Frames recomputed with the node code (shapes v2), same
keys as the precomputed files + pose66 per frame + phase; meshes: full vertices (float16, display) and arm vertices (float32, gap).
Outputs goal1/data/human_r1_reach_v025.npz (and _v2.npz link, _verts.npz), logs/meff_isaac/r1_motion_check.json."""
import json, os, sys, time, csv
import numpy as np, torch
from scipy.optimize import minimize
torch.set_num_threads(1)
sys.path.insert(0, os.path.dirname(__file__))
from effmass_human.human_inertia_core import HumanInertiaCore, SEGMENT_NAMES
from effmass_human import constants
from effmass_human.utils import batch_roteulerSMPL
from effmass_human.human_shapes import ShapeSet
import b2a_analyze as BA
H = os.path.expanduser("~"); D_ = f"{H}/effmass_mpc/goal1/data"; O = BA.O; RATE = 30.0
P66 = np.load(f"{D_}/human_b2a_reach_v2.npz")["pose66"].astype(np.float32).reshape(22, 3); T0 = np.load(f"{H}/hrc_ws/pose_npz/human_state_1781524848.npz")["trans"][0].astype(np.float32)
core = HumanInertiaCore(device="cpu"); core.set_betas(np.zeros(10, np.float32))
V2 = json.load(open(f"{D_}/shapes_v2.json")); E = V2["shapes"] + [V2["inactive"]] * (24 - len(V2["shapes"]))
core.shapes = ShapeSet([1] * 24, [e["segment"] for e in E], [e["p0_local"] for e in E], [e["p1_local"] for e in E], [e["radius"] for e in E], core.shapes.distal_local)
RS, RE = 17, 19; f32 = lambda x: x.detach().cpu().numpy().astype(np.float64) if torch.is_tensor(x) else np.asarray(x, np.float64)
tf = json.load(open(f"{BA.G}/temp_transform.json")); Rm, tv = np.array(tf["R"]), np.array(tf["t"])


def pose_of(x):
    p = P66.copy(); p[RS] = x[:3]; p[RE] = x[3:6]; return p


def fk(x):
    o = core.compute(pose_of(x).reshape(-1), T0, prev=None, dt=None); J = f32(o["joints"][0]); Rw = f32(o["world_rot"][0]); Dd = np.asarray(o["distal"], float)
    return dict(tip=Dd[18], sh=J[14], el=J[16], wr=J[18], palm=Rw[18] @ np.array([0, -1.0, 0]), o=o)


x0 = np.r_[P66[RS], P66[RE]].astype(np.float64); g0 = fk(x0); tip0 = g0["tip"]; u = (tip0 - g0["sh"]) / np.linalg.norm(tip0 - g0["sh"]); fwd = (g0["wr"] - g0["el"]) / np.linalg.norm(g0["wr"] - g0["el"])
ang = lambda a, b: np.arccos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))


def solve(target, xp):
    def obj(x):
        g = fk(x); return 1e4 * np.sum((g["tip"] - target) ** 2) + 0.5 * ang(g["palm"], g0["palm"]) ** 2 + 0.3 * ang(g["wr"] - g["el"], fwd) ** 2 + 0.05 * np.sum((x - xp) ** 2)
    r = minimize(obj, xp, method="Nelder-Mead", options=dict(xatol=1e-6, fatol=1e-10, maxiter=6000, maxfev=6000)); return r.x


# robot spheres of the V1 uniform seed 0 front run (Isaac frame) for the retract-distance rule
r = BA.result("v1_front_0.38h1_u065_s0"); Q = np.array([[float(x_[f"q{i}"]) for i in range(7)] for x_ in csv.DictReader(open(r["steps_csv"]))])
RC = np.concatenate([BA.RI.compute(q)[0] for q in Q]); RRS = np.tile(BA.RS, len(Q))
def hand_path_gap(o):
    sh = o["shapes"]; P0 = sh["p0"] @ Rm.T + tv; P1 = sh["p1"] @ Rm.T + tv; rad = sh["radius"]; seg = np.asarray(sh["segment"])
    return float(min((BA.SM.segdist(RC, P0[k], P1[k]) - rad[k] - RRS).min() for k in range(24) if seg[k] == 18 and rad[k] > 0))


keys = [x0]; svals = [0.0]; t0 = time.perf_counter(); D = None
for i in range(1, 46):
    s = 0.01 * i; xs = solve(tip0 - s * u, keys[-1]); keys.append(xs); svals.append(s)
    if s >= 0.30 - 1e-9 and D is None:
        g = fk(xs); hp = hand_path_gap(g["o"])
        print(f"s {s:.2f}: tip err {np.linalg.norm(g['tip'] - (tip0 - s * u))*1000:.2f} mm, hand-path gap {hp:.3f} m, forearm vs fwd {np.degrees(ang(g['wr'] - g['el'], fwd)):.1f} deg, "
              f"palm {np.degrees(ang(g['palm'], g0['palm'])):.1f} deg", flush=True)
        if hp >= 0.35:
            D = s; break
print("key poses", len(keys), "time", round(time.perf_counter() - t0, 1), "s, D", D, flush=True)
if D is None:
    print("STOP: no retract distance up to 0.45 m gives a 0.35 m hand-path gap"); sys.exit(2)
KX = np.array(keys); KS = np.array(svals)
x_of = lambda s: np.array([np.interp(s, KS, KX[:, j]) for j in range(6)])
mj = lambda tau: 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5


def schedule(Tr, n, rest=1.5):
    P = rest + Tr + 1.5 + Tr; t = np.arange(n) / RATE; ph = np.mod(t, P); s = np.empty(n); state = np.empty(n, object)
    for k, tt in enumerate(ph):
        if tt < rest:
            s[k], state[k] = D, "rest"
        elif tt < rest + Tr:
            s[k], state[k] = D * (1 - mj((tt - rest) / Tr)), "reaching"
        elif tt < rest + 1.5 + Tr:
            s[k], state[k] = 0.0, "hold"
        else:
            s[k], state[k] = D * mj((tt - rest - 1.5 - Tr) / Tr), "retracting"
    return s, state, P


@torch.inference_mode()
def verts(pose66):
    pw, tw = core.aa2euler(torch.as_tensor(pose66).view(1, -1), torch.as_tensor(T0).view(1, -1))
    full = torch.zeros((1, 72), dtype=torch.float32); full[:, constants.G2Hpose_idx] = pw; rot, _ = batch_roteulerSMPL(full)
    return (core.smplh.forward(betas=core.betas, rotmat=rot).vertices[0] + tw[0]).numpy()


LAB = np.load(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy"); ARMV = np.nonzero(np.isin(LAB, [14, 16, 18]))[0]
FACES = np.asarray(core.smplh.faces, np.int64); ARMF = FACES[np.isin(FACES, ARMV).all(1)]; remap = -np.ones(6890, int); remap[ARMV] = np.arange(len(ARMV)); ARMF_L = remap[ARMF]
st_ = json.load(open(f"{O}/b2a2_side_check.json")) if False else None
ref = np.load(f"{D_}/human_b2a_reach_v2.npz")
check = json.load(open(f"{O}/r1_motion_check.json")) if os.path.exists(f"{O}/r1_motion_check.json") else {"speeds": {}}
check.update({"D_retract_m": D, "reach_dir_node": u.tolist()})
_arg = lambda k, d: next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith(f"--{k}=")), d)   # --speeds=1.0 --rest=2.3 --len=120 (env vars do not pass g1_run_human.sh)
SPEEDS = [float(x) for x in _arg("speeds", "0.25,0.5,1.0").split(",")]; REST = float(_arg("rest", "1.5")); LEN = float(_arg("len", "60"))
NF = int(LEN * RATE) + 2
for v in SPEEDS:
    Tr = 1.875 * D / v
    for it in range(2):
        s, state, Pc = schedule(Tr, NF, REST); tips = np.array([fk(x_of(sv))["tip"] for sv in s[: int((REST + Tr + 0.2) * RATE)]]); vt = np.linalg.norm(np.diff(tips, axis=0), axis=1) * RATE
        if it == 0:
            Tr *= vt.max() / v
    s, state, Pc = schedule(Tr, NF, REST); R_ = {k: [] for k in ("com", "vel", "rot", "inv", "jprox", "jdist", "p0", "p1", "vdt", "vvalid", "compute_ms")}; prev = None; poses = []; VF = []; VA = []; tips = []
    for k in range(NF):
        p = pose_of(x_of(s[k])); poses.append(p.reshape(-1)); tk = time.perf_counter()
        o = core.compute(p.reshape(-1), T0, prev=prev, dt=(1 / RATE if prev is not None else None)); sh = o["shapes"]
        R_["com"].append(f32(o["com"][0])); R_["vel"].append(f32(o["vel"][0])); R_["rot"].append(f32(o["world_rot"][0])); R_["inv"].append(f32(o["inv_inertia"][0]))
        R_["jprox"].append(f32(o["joints"][0])); R_["jdist"].append(np.asarray(o["distal"], np.float64)); R_["p0"].append(sh["p0"]); R_["p1"].append(sh["p1"])
        R_["vdt"].append(1 / RATE if (prev is not None and o["vel_valid"]) else 0.0); R_["vvalid"].append(bool(o["vel_valid"])); R_["compute_ms"].append((time.perf_counter() - tk) * 1e3)
        tips.append(np.asarray(o["distal"], float)[18]); Vk = verts(p.reshape(-1)); VF.append(Vk.astype(np.float16)); VA.append(Vk[ARMV].astype(np.float32)); prev = o
    tips = np.array(tips); vt = np.linalg.norm(np.diff(tips, axis=0), axis=1) * RATE; dev = np.linalg.norm(np.cross(tips - tip0, u), axis=1)
    hold = np.nonzero(state == "hold")[0]; c_hold = (np.array(R_["p0"])[hold] + np.array(R_["p1"])[hold]) / 2; c_ref = (ref["p0"][0] + ref["p1"][0]) / 2
    act = ref["shape_radius"] > 0; hold_diff = float(np.abs(c_hold[:, act] - c_ref[None, act]).max())
    rest_gap = hand_path_gap(core.compute(pose_of(x_of(D)).reshape(-1), T0, prev=None, dt=None))
    tag = {0.25: "v025", 0.5: "v050", 1.0: "v100"}[v]; out = f"{D_}/human_r1_reach_{tag}.npz"
    sh0 = core.compute(pose_of(x0).reshape(-1), T0)["shapes"]
    np.savez(out, input_time=np.arange(NF) / RATE, frame=np.arange(NF), mass=ref["mass"], shape_type=ref["shape_type"], shape_segment=ref["shape_segment"], shape_radius=ref["shape_radius"],
             segment_names=np.array(SEGMENT_NAMES), rate=RATE, betas=np.zeros(10, np.float32), source_npz="r1_motion.py", vdt_from="", shapes_version="v2", shape_active=ref["shape_active"],
             pose66=np.array(poses, np.float32), phase=np.array(state, dtype="U10"), s_retract=s.astype(np.float32), tip=tips.astype(np.float32),
             **{k: (np.asarray(vv, np.float32) if k not in ("vvalid", "compute_ms") else np.asarray(vv)) for k, vv in R_.items()})
    link = f"{D_}/human_r1_reach_{tag}_v2.npz"
    if not os.path.exists(link):
        os.symlink(os.path.basename(out), link)
    np.savez(f"{D_}/human_r1_reach_{tag}_verts.npz", verts=np.array(VF), faces=FACES, source=np.array("r1_motion.py node body model, float16"))
    np.savez(f"{D_}/human_r1_reach_{tag}_armverts.npz", verts=np.array(VA), vid=ARMV, faces_local=ARMF_L)
    check["speeds"][tag] = dict(rest_s=REST, length_s=LEN, reach_time_s=Tr, period_s=Pc, tip_peak_speed_measured=float(vt.max()), tip_line_dev_max_m=float(dev.max()), retracted_hand_path_gap_m=rest_gap,
                                hold_shape_center_max_diff_m=hold_diff, frames=NF, compute_ms_mean=float(np.mean(R_["compute_ms"])), file=out)
    print(tag, json.dumps({k: (round(x_, 4) if isinstance(x_, float) else x_) for k, x_ in check["speeds"][tag].items()}), flush=True)
json.dump(check, open(f"{O}/r1_motion_check.json", "w"), indent=1)
