"""g1_viz.py - display-only helpers for step2b_mpc_isaac.py (demo). Nothing here feeds the MPC: it reads the current robot spheres and the
human inputs after the solver step.
  eta shells (--viz-eta-shell on): one translucent capsule per v2 hand shape, radius = shape radius + eta of the robot sphere closest to that shape,
      eta by the run's cost mode (mass: set A of mu; iso: 0.6 kg hand; uniform: eta0; off: hidden). Colour by mu (blue 0.5 kg -> red 3.0 kg).
  trail (--viz-trail on): TCP path of the last --viz-trail-sec seconds (TCP = hand link - 0.1034 m in z, as in the analyses), condition colour.
  ghost (--viz-ghost cond,cond --viz-ghost-tag <tag with {cond}>): saved TCP paths of other runs (B1 rows) as grey curves.
  record (--viz-record DIR): cameras (--viz-cam eye,target[;eye,target]) frames every step through replicator rgb annotators (DIR/cam<j>/), DIR_cam<j>.mp4 at the end.
Timing of the per-step update is kept (ms) and written to <DIR or log dir>/<tag>_viz.json."""
import json, os, time, subprocess
from collections import deque
import numpy as np

COND_RGB = {0: (0.55, 0.55, 0.55), 1: (0.93, 0.63, 0.0), 2: (0.92, 0.41, 0.20), 3: (0.16, 0.47, 0.84)}   # off, uniform, mass, iso


def mu_rgb(mu, lo=0.5, hi=3.0):
    t = float(np.clip((mu - lo) / (hi - lo), 0, 1)); return (0.16 + 0.76 * t, 0.47 - 0.22 * t, 0.84 - 0.70 * t)


class Viz:
    def __init__(self, stage, args, meff, human, tag, log, n_spheres):
        from pxr import Gf, UsdGeom, UsdShade, Sdf, Vt
        self.Gf, self.UsdGeom, self.UsdShade, self.Sdf, self.Vt = Gf, UsdGeom, UsdShade, Sdf, Vt
        self.stage, self.args, self.meff, self.human, self.tag, self.log, self.S = stage, args, meff, human, tag, log, n_spheres
        self.t_upd = []; self.t_cap = []; self.shells = []; self.trail = None; self.rec = None
        mode = int(meff.mode[0]) if meff is not None else 0; self.mode = mode
        from pxr import UsdLux
        if not stage.GetPrimAtPath("/World/viz/dome").IsValid():   # display light (the run scene has only the default light)
            UsdLux.DomeLight.Define(stage, "/World/viz/dome").CreateIntensityAttr(700)
        if getattr(args, "viz_zone", ""):   # (V1) static zone surface
            _bm = "uniform" not in os.path.basename(args.viz_zone)
            if getattr(args, "viz_zone_style", "cage") == "cage":
                log(f"(viz) zone {add_zone_cage(stage, os.path.expanduser(args.viz_zone), by_mu=_bm)}")
            else:
                log(f"(viz) zone {add_zone(stage, os.path.expanduser(args.viz_zone), opacity=getattr(args, 'viz_zone_opacity', 0.35), by_mu=_bm)}")
        self.zseq = None
        if getattr(args, "viz_zone_seq", ""):   # (R1) zone cages along the retract motion; the one closest to the current frame's retract distance is shown
            zs = json.load(open(os.path.expanduser(args.viz_zone_seq))); cond = args.viz_zone_seq_cond; hz = np.load(zs["human"] if "human" in zs else self.human.pre_path)["s_retract"]; prims = []   # retract distance per frame of the human file in use
            for i, fn in enumerate(zs["files"][cond]):
                add_zone_cage(stage, fn, path=f"/World/viz/zoneseq/c{i:02d}", by_mu=(cond != "uniform")); pr = stage.GetPrimAtPath(f"/World/viz/zoneseq/c{i:02d}")
                UsdGeom.Imageable(pr).MakeInvisible(); prims.append(pr)
            self.zseq = dict(S=np.array(zs["s"]), prims=prims, hs=hz, cur=-1); log(f"(viz) zone sequence {cond}: {len(prims)} cages")
        if getattr(args, "viz_hide_com", "off") == "on":   # hide the human segment COM markers (PointInstancer of the human link)
            pr = stage.GetPrimAtPath("/World/human/instancer")
            if pr.IsValid():
                UsdGeom.Imageable(pr).MakeInvisible()
        if args.viz_eta_shell == "on" and meff is not None:
            h = human.last_h; self.hand = [k for k in range(len(h["rad"])) if int(h["seg"][k]) == 18 and float(h["rad"][k]) > 0]
            for k in self.hand:
                g = UsdGeom.Capsule.Define(stage, f"/World/viz/eta_shell_{k:02d}"); g.CreateAxisAttr("Z"); x = UsdGeom.Xformable(g)
                m = UsdShade.Material.Define(stage, f"/World/viz/Looks/shell_{k:02d}"); sh = UsdShade.Shader.Define(stage, f"/World/viz/Looks/shell_{k:02d}/s"); sh.CreateIdAttr("UsdPreviewSurface")
                col = sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f); col.Set(Gf.Vec3f(0.5, 0.5, 0.5)); sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.22)
                sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3); m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
                UsdGeom.PrimvarsAPI(g.GetPrim()).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
                UsdShade.MaterialBindingAPI(g.GetPrim()).Bind(m); g.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.5, 0.5, 0.5)])); g.CreateDisplayOpacityAttr(Vt.FloatArray([0.28]))
                self.shells.append((k, g, x.AddTranslateOp(), x.AddOrientOp(), col, g.GetDisplayColorAttr()))
            if mode == 0:
                for _, g, *_r in self.shells:
                    UsdGeom.Imageable(g).MakeInvisible()
        if args.viz_trail == "on":
            c = UsdGeom.BasisCurves.Define(stage, "/World/viz/trail"); c.CreateTypeAttr("linear"); c.CreateWidthsAttr(Vt.FloatArray([0.006])); c.SetWidthsInterpolation("constant")
            c.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*COND_RGB.get(mode, (0.5, 0.5, 0.5)))]))
            self.trail = c; self.buf = deque(maxlen=max(2, int(args.viz_trail_sec * 60)))
            UsdShade.MaterialBindingAPI(c.GetPrim()).Bind(self._mat("trail", COND_RGB.get(mode, (0.5, 0.5, 0.5))))
        if args.viz_ghost:
            O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac")
            for j, cond in enumerate([x for x in args.viz_ghost.split(",") if x]):
                f = f"{O}/{args.viz_ghost_tag.format(cond=cond)}_b1.npz"
                if not os.path.exists(f):
                    log(f"(viz) ghost {cond}: {f} not found"); continue
                P = np.load(f)["rows"][:, 8:11] - np.array([0, 0, 0.1034])
                g = UsdGeom.BasisCurves.Define(stage, f"/World/viz/ghost_{cond}"); g.CreateTypeAttr("linear"); g.CreateCurveVertexCountsAttr(Vt.IntArray([len(P)]))
                g.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(P, np.float32))); g.CreateWidthsAttr(Vt.FloatArray([0.004])); g.SetWidthsInterpolation("constant")
                g.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*([0.45, 0.45, 0.45] if j == 0 else [0.65, 0.65, 0.65]))]))
                UsdShade.MaterialBindingAPI(g.GetPrim()).Bind(self._mat(f"ghost_{cond}", (0.45, 0.45, 0.45) if j == 0 else (0.65, 0.65, 0.65)))
                log(f"(viz) ghost {cond}: {len(P)} points from {os.path.basename(f)}")
        if args.viz_record:
            import omni.replicator.core as rep
            W, H = [int(x) for x in args.viz_record_res.split("x")]; self.rec = []; self.rec_dir = os.path.expanduser(args.viz_record)
            for j, cs in enumerate([c for c in args.viz_cam.split(";") if c]):   # several cameras: "eye,target;eye,target"
                v = [float(x) for x in cs.split(",")]; cp = f"/World/viz/rec_cam{j}"
                cam = UsdGeom.Camera.Define(stage, cp); cam.CreateFocalLengthAttr(24.0); cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
                UsdGeom.Xformable(cam).AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*v[:3]), Gf.Vec3d(*v[3:]), Gf.Vec3d(0, 0, 1)).GetInverse())
                rp = rep.create.render_product(cp, (W, H)); a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach([rp])
                self.rec.append(a); os.makedirs(f"{self.rec_dir}/cam{j}", exist_ok=True)
        log(f"(viz) eta shell {args.viz_eta_shell} ({len(self.shells)} hand shapes, cost mode {mode}), trail {args.viz_trail} {args.viz_trail_sec} s, "
            f"ghost '{args.viz_ghost}', record '{args.viz_record}'")

    def _mat(self, name, rgb):
        UsdShade, Sdf, Gf = self.UsdShade, self.Sdf, self.Gf
        m = UsdShade.Material.Define(self.stage, f"/World/viz/Looks/{name}"); sh = UsdShade.Shader.Define(self.stage, f"/World/viz/Looks/{name}/s"); sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb)); sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[0.5 * x for x in rgb]))
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface"); return m

    def _eta(self, mu, mr):
        p = self.meff.par.cpu().numpy() if not hasattr(self, "_par") else self._par; self._par = p
        e0, mref, pw, emin, emax = [float(x) for x in p[1:6]]
        if self.mode == 1:
            return e0
        if self.mode == 3:
            mu = 1.0 / (float(self.meff.inv_iso[18]) + (1.0 / mr if np.isfinite(mr) and mr > 0 else 0.0))
        return float(np.clip(e0 * (mu / mref) ** pw, emin, emax))

    def update(self, i, sph, ee):
        t0 = time.perf_counter(); Gf = self.Gf
        if self.zseq is not None and self.human.cur_frame is not None:
            z = self.zseq; j = int(np.argmin(np.abs(z["S"] - z["hs"][min(self.human.cur_frame, len(z["hs"]) - 1)])))
            if j != z["cur"]:
                if z["cur"] >= 0:
                    self.UsdGeom.Imageable(z["prims"][z["cur"]]).MakeInvisible()
                self.UsdGeom.Imageable(z["prims"][j]).MakeVisible(); z["cur"] = j
        if self.shells and self.mode != 0:
            qa = self.meff.query_all(sph); d = qa["dist"].cpu().numpy(); mu = qa["mu"].cpu().numpy(); mr = qa["mr"].cpu().numpy(); h = self.human.last_h
            for k, g, tr, ori, col, dcol in self.shells:
                s = int(np.argmin(d[:, k])); e = self._eta(float(mu[s, k]), float(mr[s, k])); a, b = h["p0"][k], h["p1"][k]; v = b - a; n = float(np.linalg.norm(v))
                g.GetRadiusAttr().Set(float(h["rad"][k]) + e); g.GetHeightAttr().Set(n); tr.Set(Gf.Vec3d(*((a + b) / 2).tolist()))
                z = np.array([0, 0, 1.0]); vv = v / max(n, 1e-12); ax = np.cross(z, vv); sn = np.linalg.norm(ax)
                ori.Set(Gf.Quatf(1, 0, 0, 0) if sn < 1e-9 else Gf.Quatf(float(np.cos(np.arctan2(sn, z @ vv) / 2)), *[float(x) for x in ax / sn * np.sin(np.arctan2(sn, z @ vv) / 2)]))
                rgb = mu_rgb(float(mu[s, k])); col.Set(Gf.Vec3f(*rgb)); dcol.Set(self.Vt.Vec3fArray([Gf.Vec3f(*rgb)]))
        if self.trail is not None and i % 2 == 0:
            self.buf.append(ee.detach().cpu().numpy().reshape(3) - np.array([0, 0, 0.1034]))
            if len(self.buf) >= 2:
                P = np.ascontiguousarray(np.array(self.buf), np.float32); self.trail.GetPointsAttr().Set(self.Vt.Vec3fArray.FromNumpy(P))
                self.trail.GetCurveVertexCountsAttr().Set(self.Vt.IntArray([len(P)]))
        self.t_upd.append((time.perf_counter() - t0) * 1000)

    def capture(self, i):
        if self.rec is None:
            return
        t0 = time.perf_counter(); from PIL import Image
        for j, a in enumerate(self.rec):
            dat = np.asarray(a.get_data())
            if dat.size:
                Image.fromarray(dat[:, :, :3]).save(f"{self.rec_dir}/cam{j}/f{i:05d}.png")
        self.t_cap.append((time.perf_counter() - t0) * 1000)

    def finish(self, out_dir):
        st = dict(update_ms_mean=float(np.mean(self.t_upd)) if self.t_upd else None, update_ms_p95=float(np.percentile(self.t_upd, 95)) if self.t_upd else None,
                  capture_ms_mean=float(np.mean(self.t_cap)) if self.t_cap else None, n_updates=len(self.t_upd))
        if self.rec is not None:
            st["mp4"] = []
            for j in range(len(self.rec)):
                mp4 = f"{self.rec_dir.rstrip('/')}_cam{j}.mp4"
                r = subprocess.run(["/usr/bin/ffmpeg", "-loglevel", "error", "-y", "-framerate", "30", "-i", f"{self.rec_dir}/cam{j}/f%05d.png", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", mp4])
                st["mp4"].append(mp4 if r.returncode == 0 else None)
            st["mp4_note"] = "raw camera clips, 30 fps for 60 Hz steps (0.5x)"
        json.dump(st, open(os.path.join(out_dir, f"{self.tag}_viz.json"), "w"), indent=1); self.log(f"(viz) {st}")
        return st


def add_zone(stage, npz, path="/World/viz/zone", opacity=0.35, by_mu=True, rgb=(0.62, 0.72, 0.85), roughness=0.9):
    """(V1) static translucent avoidance-zone surface from v1_zone.py (zone_<cond>_mesh.npz, world frame); vertex colour by mu (blue 0.5 -> red 3.0 kg)"""
    from pxr import Gf, UsdGeom, UsdShade, Sdf, Vt
    z = np.load(npz); P = np.ascontiguousarray(z["verts"], np.float32); Fc = np.asarray(z["faces"], np.int32)
    m = UsdGeom.Mesh.Define(stage, path); m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P)); m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*P.min(0).tolist()), Gf.Vec3f(*P.max(0).tolist())]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(Fc))); m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(Fc.reshape(-1))); m.CreateSubdivisionSchemeAttr("none"); m.CreateDoubleSidedAttr(True)
    C = np.array([mu_rgb(float(x)) for x in z["mu"]], np.float32) if by_mu else np.tile(np.array(rgb, np.float32), (len(P), 1))
    pv = UsdGeom.PrimvarsAPI(m.GetPrim()).CreatePrimvar("displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex); pv.Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(C)))
    UsdGeom.PrimvarsAPI(m.GetPrim()).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
    # translucent colour by mu: faces binned into 8 mu bins (0.5 - 3.0 kg), one UsdPreviewSurface material per bin (colour + opacity) bound to a face subset
    # (a single material reading the displayColor primvar rendered white in RTX; displayColor alone ignores displayOpacity)
    fm = np.asarray(z["mu"], float)[Fc].mean(1) if by_mu else np.zeros(len(Fc)); edges = np.linspace(0.5, 3.0, 9); bins = np.clip(np.digitize(fm, edges) - 1, 0, 7)
    for bi in (range(8) if by_mu else [0]):
        idx = np.nonzero(bins == bi)[0] if by_mu else np.arange(len(Fc))
        if not len(idx):
            continue
        col = mu_rgb(0.5 * (edges[bi] + edges[bi + 1])) if by_mu else rgb
        sub = UsdGeom.Subset.CreateGeomSubset(UsdGeom.Imageable(m), f"mu{bi}", "face", Vt.IntArray.FromNumpy(idx.astype(np.int32)), "materialBind")
        mat = UsdShade.Material.Define(stage, f"{path}_mat{bi}"); sh = UsdShade.Shader.Define(stage, f"{path}_mat{bi}/s"); sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*col)); sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness)); sh.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
        sh.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.02, 0.02, 0.02)); mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat)
    return dict(verts=len(P), faces=len(Fc), file=npz)


def add_zone_cage(stage, npz, path="/World/viz/zone_cage", dz=0.02, dx=0.03, width=0.0035, by_mu=True, rgb=(0.35, 0.45, 0.65)):
    """(V2) zone boundary drawn as contour rings (see-through): sections of the v1 zone surface by horizontal planes every dz and by planes normal to x
    every dx; segments coloured by mu (8 bins, blue 0.5 -> red 3.0 kg; one emissive material per bin)"""
    from pxr import Gf, UsdGeom, UsdShade, Sdf, Vt
    z = np.load(npz); V = np.asarray(z["verts"], float); Fc = np.asarray(z["faces"]); MU = np.asarray(z["mu"], float); T = V[Fc]; TM = MU[Fc]; segs, mus = [], []
    for axis, step in ((2, dz), (0, dx)):
        lo, hi = V[:, axis].min(), V[:, axis].max()
        for val in np.arange(np.ceil(lo / step) * step, hi, step):
            sv = T[:, :, axis] - val; cross = ~((sv > 0).all(1) | (sv < 0).all(1))
            for tri, sv_, tm in zip(T[cross], sv[cross], TM[cross]):
                pts, mm = [], []
                for i, j in ((0, 1), (1, 2), (2, 0)):
                    if (sv_[i] > 0) != (sv_[j] > 0):
                        w = sv_[i] / (sv_[i] - sv_[j]); pts.append(tri[i] + w * (tri[j] - tri[i])); mm.append(tm[i] + w * (tm[j] - tm[i]))
                if len(pts) == 2:
                    segs.append(pts); mus.append(0.5 * (mm[0] + mm[1]))
    segs = np.asarray(segs, np.float32); mus = np.asarray(mus); edges = np.linspace(0.5, 3.0, 9); bins = np.clip(np.digitize(mus, edges) - 1, 0, 7) if by_mu else np.zeros(len(mus), int)
    for bi in sorted(set(bins.tolist())):
        sel = segs[bins == bi]; col = mu_rgb(0.5 * (edges[bi] + edges[bi + 1])) if by_mu else rgb
        c = UsdGeom.BasisCurves.Define(stage, f"{path}/b{bi}"); c.CreateTypeAttr("linear"); c.CreateCurveVertexCountsAttr(Vt.IntArray([2] * len(sel)))
        c.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(sel.reshape(-1, 3)))); c.CreateWidthsAttr(Vt.FloatArray([width])); c.SetWidthsInterpolation("constant")
        c.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*col)])); UsdGeom.PrimvarsAPI(c.GetPrim()).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
        m = UsdShade.Material.Define(stage, f"{path}/m{bi}"); sh = UsdShade.Shader.Define(stage, f"{path}/m{bi}/s"); sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*col)); sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[0.6 * x for x in col]))
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface"); UsdShade.MaterialBindingAPI(c.GetPrim()).Bind(m)
    return dict(segments=int(len(segs)), file=npz, style="cage")
