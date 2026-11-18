"""lv_surface_meff.py <tag> - overnight2 bundle 5: zone thickness from the effective mass at the segment COM (current method) vs at the actual
surface point (calculation only; the MPC code and the human dynamics node are not changed; ~/hrc_ws is not used).
Posture: the representative held frame of each motion (as lv_zone_figs.py), recomputed with HumanInertiaCore (camera pose, betas,
pelvis correction, shapes v2 of the live run). For the arm of the motion (upper arm, forearm, hand capsules): surface points at 5 positions
along each capsule axis x 8 directions around it + the two end caps; approach direction u = -outward normal.
  COM: Lambda^-1 = inv_inertia[segment] (J_p of the segment COM, M^-1 with the node's contact / lock projection)
  surface: Lambda_p^-1 = J_p(point) M^-1 J_p(point)^T, J_p(point) = physics.compute_J_p_19 with the segment's COM replaced by the point
  distal end cap of each capsule (the fingertip front for the hand) reported separately;
  m_h = 1 / (u^T Lambda^-1 u); mu = 1 / (1/m_h + 1/4.6 kg); eta = clip(0.065 + 0.10 (sqrt(mu / mu_iso(seg)) - 1)+, 0.065, 0.25) (per-part rule)
Writes goal1/overnight2/surface_meff.json (per motion and part: delta eta median / p95 / max, signed median, share |delta| > 1 cm)."""
import glob, json, os, re, sys
import numpy as np, torch
torch.set_num_threads(2)
from effmass_human.human_inertia_core import HumanInertiaCore
from effmass_human.human_shapes import ShapeSet
H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; GG = f"{H}/effmass_mpc/goal1"
ISO_NM = ["pelvis", "left_thigh", "right_thigh", "abdomen", "left_calf", "right_calf", "left_calf", "right_calf", "chest", "neck", "left_shoulder",
          "right_shoulder", "head", "left_upper_arm", "right_upper_arm", "left_lower_arm", "right_lower_arm", "left_hand", "right_hand"]
M_ISO = np.array([json.load(open(f"{GG}/data/iso_15066.json"))[n]["m_h_iso"] for n in ISO_NM], float); MR = 4.6


def eta(mh, seg):
    mu = 1.0 / (1.0 / mh + 1.0 / MR); return np.clip(0.065 + 0.10 * np.maximum(np.sqrt(mu * (1.0 / M_ISO[seg] + 1.0 / MR)) - 1, 0), 0.065, 0.25)


def main(tag):
    C = f"{GG}/live/{tag}/compare"; P = json.load(open(f"{GG}/live/{tag}/compare_tip/paths.json"))
    slog = open(f"{O}/{tag}_sensing.log", errors="replace").read()
    m = re.findall(r"\[camera pose\] roll ([-\d.]+) deg, pitch ([-\d.]+) deg.*?yaw ([-\d.]+) deg, t \[([^\]]+)\]", slog)[-1]
    cam = dict(roll_deg=float(m[0]), pitch_deg=float(m[1]), yaw_deg=float(m[2]), t=[float(v) for v in m[3].split(",")])
    bm = re.findall(r"cliff beta mean : \[([^\]]+)\]", slog, re.S); betas = np.array([float(v) for v in bm[-1].split()], np.float32)[:10]
    pz = np.load(sorted(glob.glob(f"{O}/{tag}_sensing/pose_npz/*.npz"))[-1]); PO, TR = pz["poses"], pz["trans"]; N = len(PO)
    upd = np.load(f"{O}/{tag}_human_inertia_rec.npz")["upd"]; k0 = sorted(int(c) - 1 for c in upd[:, 3] if 0 <= int(c) - 1 < N)[0]
    z = np.load(f"{C}/human_rt_v2.npz"); P0a, P1a = z["p0"].reshape(len(z["p0"]), -1, 3), z["p1"].reshape(len(z["p1"]), -1, 3)
    core = HumanInertiaCore("cpu", camera_pose=cam, pelvis_correction=True); core.set_betas(betas)
    V2 = json.load(open(f"{GG}/data/shapes_v2.json")); E = V2["shapes"] + [V2["inactive"]] * (24 - len(V2["shapes"]))
    core.shapes = ShapeSet([1] * 24, [e["segment"] for e in E], [e["p0_local"] for e in E], [e["p1_local"] for e in E], [e["radius"] for e in E], core.shapes.distal_local)
    out = {}
    for mm in P["motions"]:
        hf = np.array(mm["hold_frames"]); med0 = np.median(P0a[hf], 0); med1 = np.median(P1a[hf], 0)
        f = int(hf[np.argmin([np.abs(P0a[x] - med0).sum() + np.abs(P1a[x] - med1).sum() for x in hf])]); k = k0 + int(z["frame"][f])
        with torch.inference_mode():
            o = core.compute(PO[k], TR[k], betas=betas)
        Minv = o["Minv_contact"][0]; Jr = o["Jr"]; joints = o["joints"]; com = o["com"]; inv = o["inv_inertia"][0].numpy(); sh = o["shapes"]
        p0, p1, rad, seg = np.asarray(sh["p0"]), np.asarray(sh["p1"]), np.asarray(sh["radius"]), np.asarray(sh["segment"])
        segs = (13, 15, 17) if mm["side"] == "L" else (14, 16, 18); res = {}
        for s, part in zip(segs, ("upper_arm", "forearm", "hand")):
            pts, nrm, cap = [], [], []
            for kk in [q for q in range(len(rad)) if seg[q] == s and rad[q] > 0]:
                a, b = p0[kk], p1[kk]; ax = b - a; L = np.linalg.norm(ax); ax = ax / L if L > 1e-9 else np.array([1.0, 0, 0])
                e1 = np.cross(ax, [0, 0, 1.0]); e1 = e1 / np.linalg.norm(e1) if np.linalg.norm(e1) > 1e-6 else np.array([1.0, 0, 0]); e2 = np.cross(ax, e1)
                for tt in (0.1, 0.3, 0.5, 0.7, 0.9):
                    for ang in np.arange(8) * np.pi / 4:
                        n = np.cos(ang) * e1 + np.sin(ang) * e2; pts.append(a + tt * (b - a) + rad[kk] * n); nrm.append(n); cap.append(0)
                for e, sg in ((a, -1), (b, 1)):
                    pts.append(e + sg * rad[kk] * ax); nrm.append(sg * ax); cap.append(1 if sg > 0 else 0)
            pts = np.array(pts); nrm = np.array(nrm); n_ = len(pts)
            cm = com.expand(n_, -1, -1).clone(); cm[:, s] = torch.as_tensor(pts, dtype=cm.dtype)
            with torch.inference_mode():
                Jp = core.physics.compute_J_p_19(cm, joints.expand(n_, -1, -1), Jr.expand(n_, -1, -1, -1))[:, s]   # (n, 3, 60)
                Lp = torch.einsum("nid,de,nje->nij", Jp, Minv, Jp).numpy()
            Lp = 0.5 * (Lp + np.transpose(Lp, (0, 2, 1))); Lc = inv[s]
            mh_c = 1.0 / np.einsum("ni,ij,nj->n", nrm, Lc, nrm); mh_s = 1.0 / np.einsum("ni,nij,nj->n", nrm, Lp, nrm)
            ec, es = eta(mh_c, s), eta(mh_s, s); d = (es - ec) * 100; cap = np.array(cap, bool)
            # distal end caps (e.g. the fingertip front for the hand): eta COM / surface
            capd = dict(eta_com_cm=[round(float(x * 100), 2) for x in ec[cap]], eta_surface_cm=[round(float(x * 100), 2) for x in es[cap]])
            res[part] = dict(points=int(n_), distal_caps=capd, delta_eta_cm=dict(median_abs=float(np.median(np.abs(d))), p95_abs=float(np.percentile(np.abs(d), 95)), max_abs=float(np.max(np.abs(d))),
                             signed_median=float(np.median(d)), share_over_1cm=float((np.abs(d) > 1).mean())),
                             eta_com_cm=dict(median=float(np.median(ec) * 100), max=float(np.max(ec) * 100)), eta_surface_cm=dict(median=float(np.median(es) * 100), max=float(np.max(es) * 100)),
                             mh_ratio_surface_over_com=dict(median=float(np.median(mh_s / mh_c)), min=float(np.min(mh_s / mh_c)), max=float(np.max(mh_s / mh_c))))
        out[mm["name"]] = dict(frame=f, pose_row=k, parts=res); print(mm["name"], json.dumps({p: {kk: (round(v, 2) if isinstance(v, float) else v) for kk, v in r["delta_eta_cm"].items()} for p, r in res.items()}), flush=True)
    json.dump(out, open(f"{GG}/overnight2/surface_meff.json", "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1])
