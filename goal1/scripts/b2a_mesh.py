"""b2a_mesh.py - display-only mesh vertices with the same body model as the effmass_human node (run: g1_run_human.sh py b2a_mesh.py).
Vertices = HumanInertiaCore.smplh.forward(betas, rotmat_world).vertices + trans_world, where rotmat_world comes from the node path
(aa2euler -> G2Hpose_idx -> batch_roteulerSMPL), same betas (zeros), pose, global orientation, translation and camera-to-world transform.
Sensing smpl_vertices are not used. Outputs are separate from the cost path:
  goal1/data/human_b2a_reach_verts.npz            (static pose, 1 frame)
  goal1/data/human_precomputed_1781524848_verts.npz (replay, 1262 frames)
Alignment check: per segment p95 of distance from each vertex to the nearest capsule surface of its own segment (positive = outside),
plus centroid offset (mean vertex - mean capsule axis point) to detect a global shift."""
import json, os, sys
import numpy as np, torch
torch.set_num_threads(1)
from effmass_human.human_inertia_core import HumanInertiaCore
from effmass_human import constants
from effmass_human.utils import batch_roteulerSMPL
from effmass_human.human_shapes import MERGE
H = os.path.expanduser("~"); D = f"{H}/effmass_mpc/goal1/data"
core = HumanInertiaCore("cpu"); core.set_betas(np.zeros(10, np.float32))
faces = np.asarray(core.smplh.faces, np.int64)


@torch.inference_mode()
def verts(pose66, trans3):
    pose_cam = torch.as_tensor(np.asarray(pose66, np.float32)).view(1, -1); trans_cam = torch.as_tensor(np.asarray(trans3, np.float32)).view(1, -1)
    pose_world, trans_world = core.aa2euler(pose_cam, trans_cam)
    full = torch.zeros((1, 72), dtype=torch.float32); full[:, constants.G2Hpose_idx] = pose_world
    rotmat_world, _ = batch_roteulerSMPL(full)
    out = core.smplh.forward(betas=core.betas, rotmat=rotmat_world)
    return (out.vertices[0] + trans_world[0]).numpy().astype(np.float64)


def seg_of_vertex():
    lab = np.full(6890, -1)
    for s, parts in MERGE.items():
        for p in parts:
            ids = np.asarray(core.physics.bodypart_vertexid[p]); ids = ids[ids < 6890]; lab[ids] = s
    return lab


LAB = seg_of_vertex()


def segdist(P, a, b):
    ab = b - a; L2 = float(ab @ ab); t = np.clip(((P - a) @ ab) / max(L2, 1e-12), 0, 1)
    return np.linalg.norm(P - (a + t[:, None] * ab), axis=1)


def align(V, p0, p1, rad, sseg):
    rep = {}
    for s in range(19):
        P = V[LAB == s]; ks = np.nonzero(sseg == s)[0]
        ex = np.min(np.stack([segdist(P, p0[k], p1[k]) - rad[k] for k in ks]), 0)
        axc = np.mean([0.5 * (p0[k] + p1[k]) for k in ks], 0)
        rep[int(s)] = dict(n=int(len(P)), excess_p95=float(np.percentile(ex, 95)), excess_median=float(np.median(ex)), outside_frac=float((ex > 0).mean()),
                           abs_dist_p95=float(np.percentile(np.abs(ex), 95)), centroid_minus_axis_mid=(P.mean(0) - axc).round(4).tolist())
    return rep


if __name__ == "__main__":
    S = np.load(f"{D}/human_b2a_reach.npz"); pose = S["pose66"][0]; tr = np.load(f"{H}/hrc_ws/pose_npz/human_state_1781524848.npz")["trans"][0]
    Vs = verts(pose, tr)[None].astype(np.float32)
    np.savez(f"{D}/human_b2a_reach_verts.npz", verts=Vs, faces=faces, source="b2a static reach pose66 + frame0 trans, betas 0, node SMPLH")
    rs = align(Vs[0].astype(float), S["p0"][0], S["p1"][0], S["shape_radius"], S["shape_segment"])
    z = np.load(f"{H}/hrc_ws/pose_npz/human_state_1781524848.npz"); P, T = z["poses"].astype(np.float32), z["trans"].astype(np.float32)
    Vr = np.stack([verts(P[k], T[k]) for k in range(len(P))]).astype(np.float32)
    np.savez(f"{D}/human_precomputed_1781524848_verts.npz", verts=Vr, faces=faces, source="replay human_state_1781524848.npz, betas 0, node SMPLH")
    R = np.load(f"{D}/human_precomputed_1781524848.npz")
    rr = {f: align(Vr[f].astype(float), R["p0"][f], R["p1"][f], R["shape_radius"], R["shape_segment"]) for f in (0, 300, 600, 900, 1261)}
    json.dump(dict(static=rs, replay=rr, faces=int(len(faces)), seg_unassigned=int((LAB < 0).sum())), open(f"{H}/effmass_mpc/goal1/b2a/mesh_align.json", "w"), indent=1)
    names = list(R["segment_names"])
    print("faces", faces.shape, "unassigned vertices", int((LAB < 0).sum()), "replay verts", Vr.shape)
    for s in range(19):
        a = rs[s]; print(f"{names[s]:16s} n {a['n']:4d}  excess p95 {a['excess_p95']*100:6.2f} cm  median {a['excess_median']*100:6.2f}  outside {a['outside_frac']*100:5.1f}%  centroid-axis {a['centroid_minus_axis_mid']}")
    print("replay excess p95 max over segments per frame:", {f: round(max(v['excess_p95'] for v in r.values()) * 100, 2) for f, r in rr.items()})
    np.save(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy", LAB)
