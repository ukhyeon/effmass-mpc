"""lv_mesh_verts.py <tag> - overnight2: display-only SMPL-H body mesh vertices for the real-time replay file of a live run
(compare/human_rt_v2.npz, 30 Hz), with the body model, camera pose, betas and pelvis correction of the effmass_human node path
(HumanInertiaCore.aa2euler -> smplh.forward, as b2a_mesh.py). Message j of the replay file = sensing pose row k0 + j (pr_live_prep.py).
Writes compare/human_rt_verts.npz (verts float32 [F, 6890, 3], faces) next to the replay file (picked up by step2b --human-viz mesh)
and, for each compare_tip/human_<motion>_v2.npz slice, human_<motion>_verts.npz. Alignment: per segment p95 of the vertex excess over
its own capsules on 3 frames (b2a_mesh.align). Not committed (vertex data)."""
import glob, json, os, re, sys
import numpy as np, torch
torch.set_num_threads(2)
from effmass_human.human_inertia_core import HumanInertiaCore
from effmass_human import constants
from effmass_human.utils import batch_roteulerSMPL
H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; GG = f"{H}/effmass_mpc/goal1"


def main(tag):
    C = f"{GG}/live/{tag}/compare"; slog = open(f"{O}/{tag}_sensing.log", errors="replace").read()
    m = re.findall(r"\[camera pose\] roll ([-\d.]+) deg, pitch ([-\d.]+) deg.*?yaw ([-\d.]+) deg, t \[([^\]]+)\]", slog)[-1]
    cam = dict(roll_deg=float(m[0]), pitch_deg=float(m[1]), yaw_deg=float(m[2]), t=[float(v) for v in m[3].split(",")])
    bm = re.findall(r"cliff beta mean : \[([^\]]+)\]", slog, re.S); betas = np.array([float(v) for v in bm[-1].split()], np.float32)[:10]
    pz = np.load(sorted(glob.glob(f"{O}/{tag}_sensing/pose_npz/*.npz"))[-1]); PO, TR = pz["poses"], pz["trans"]; N = len(PO)
    upd = np.load(f"{O}/{tag}_human_inertia_rec.npz")["upd"]; ks = sorted(int(c) - 1 for c in upd[:, 3] if 0 <= int(c) - 1 < N); k0 = ks[0]
    z = np.load(f"{C}/human_rt_v2.npz"); j = z["frame"].astype(int); F = len(j)
    core = HumanInertiaCore("cpu", camera_pose=cam, pelvis_correction=True); core.set_betas(betas)
    cache = {}
    with torch.inference_mode():
        def verts(k):
            if k not in cache:
                pose_cam = torch.as_tensor(np.asarray(PO[k], np.float32)).view(1, -1); trans_cam = torch.as_tensor(np.asarray(TR[k], np.float32)).view(1, -1)
                pose_world, trans_world = core.aa2euler(pose_cam, trans_cam, betas=betas)
                full = torch.zeros((1, 72), dtype=torch.float32); full[:, constants.G2Hpose_idx] = pose_world
                rot, _ = batch_roteulerSMPL(full); out = core.smplh.forward(betas=core.betas, rotmat=rot)
                cache[k] = (out.vertices[0] + trans_world[0]).numpy().astype(np.float32)
            return cache[k]
        V = np.stack([verts(k0 + int(jj)) for jj in j])
    faces = np.asarray(core.smplh.faces, np.int64)
    if not os.path.exists(f"{C}/human_rt_verts.npz"):
        np.savez(f"{C}/human_rt_verts.npz", verts=V, faces=faces, source=np.array(f"{tag}: SMPL-H vertices per replay frame (display only)"))
    sys.path.insert(0, f"{GG}/scripts")
    from b2a_mesh import LAB, segdist
    def excess(f, sg):   # p95 of (vertex distance to its own segment's capsules - radius), m
        P = V[f][LAB == sg].astype(np.float64); p0 = z["p0"][f].reshape(-1, 3); p1 = z["p1"][f].reshape(-1, 3); rad = z["shape_radius"]; ks_ = np.nonzero((z["shape_segment"] == sg) & (rad > 0))[0]
        return round(float(np.percentile(np.min(np.stack([segdist(P, p0[k], p1[k]) - rad[k] for k in ks_]), 0), 95)), 3) if len(ks_) and len(P) else None
    rep = {int(f): {sg: excess(f, sg) for sg in (0, 3, 12, 15, 16, 17, 18)} for f in (F // 4, F // 2, 3 * F // 4)}
    T = f"{GG}/live/{tag}/compare_tip"; sl = []
    if os.path.exists(f"{T}/scenes.json"):
        for mm in json.load(open(f"{T}/scenes.json"))["motions"]:
            if mm.get("status") == "ok":
                nm = mm["name"].replace(" ", ""); f0 = mm["frame0"]; n = mm["frames"]
                np.savez(f"{T}/human_{nm}_verts.npz", verts=V[f0:f0 + n], faces=faces); sl.append(nm)
    print(json.dumps(dict(frames=F, unique_rows=len(cache), slices=sl, align_excess_p95_m=rep)))


if __name__ == "__main__":
    main(sys.argv[1])
