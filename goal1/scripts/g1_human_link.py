"""g1_human_link.py — Isaac 쪽 HumanInertiaState 수신·표시·GPU 입력 공급 (1-2d). rclpy·effmass_msgs 가 import 가능해야 함 (EFFMASS_WS=1).

  HumanInertiaLink(node, transform_json, inputs, stage=None, display=("instancer",), shapes_viz=False)
    - 구독 /effmass/human_inertia_state (사본 노드와 같은 QoS: BEST_EFFORT, depth 5)
      콜백(수신 스레드): 배열을 numpy 로 복사해 '최신 슬롯' 하나를 잠금 안에서 통째로 바꿈 (분절·형상·Λ 가 한 메시지에서 옴)
    - pump() (메인 스레드, 매 스텝): 새 메시지가 있을 때만 to_isaac 변환 → MeffInputs.set_human (pinned → 고정 GPU 텐서 copy_),
      COM 19개 표시(PointInstancer, 4단계와 같음), 선택: 형상 반투명 표시. 매 스텝 age = 지금 − 입력 HumanPose stamp 를 갱신
      반환 dict 키는 step4_human.HumanLink.pump 와 같음 (step2b 기록 열 그대로)
  set_lines(...) : 로봇 구 ↔ 가장 가까운 형상 선 (UsdGeom.BasisCurves) — 표시 선택 옵션
  기록: recv (순번, 입력 stamp, 수신 시각, 노드 compute_ms), upd (처리 시각, 변환·텐서·표시 ms) — save(prefix)
"""
import json
import threading
import time

import numpy as np

import g1_meff_gpu as G

CAPSULE_RGB = (0.30, 0.60, 0.95); CAPSULE_OPACITY = 0.30          # (B2a) display only
SKIN_RGB = (0.87, 0.70, 0.58)
RAD = [0.12, 0.08, 0.08, 0.11, 0.06, 0.06, 0.05, 0.05, 0.12, 0.06, 0.06, 0.06, 0.10, 0.05, 0.05, 0.045, 0.045, 0.045, 0.045]   # step4_human 과 같음


def load_frame(path):
    """(live) node world -> Isaac transform (R, t), p_isaac = R p + t.
    .json: {"R", "t"} (previous transforms). .yml/.yaml: live_frame.yml (rotation_z_deg, base_position_world):
    R = Rz(rotation_z_deg)^T, t = -R base_position_world."""
    if str(path).endswith((".yml", ".yaml")):
        import yaml
        c = yaml.safe_load(open(path))
        a = np.radians(float(c["rotation_z_deg"])); ca, sa = np.cos(a), np.sin(a)
        Rz = np.array([[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]])
        R = Rz.T; t = -R @ np.asarray(c["base_position_world"], np.float64)
        return R, t
    tf = json.load(open(path))
    return np.asarray(tf["R"], np.float64), np.asarray(tf["t"], np.float64)


class HumanInertiaLink:
    def __init__(self, node, transform_json, inputs, stage=None, display=("instancer",), shapes_viz=False, root="/World/human", vel_alpha=0.0,
                 loss_hold_s=0.0, loss_detect_s=0.1, gui=False):
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from effmass_msgs.msg import HumanInertiaState
        self.R, self.t = load_frame(transform_json)
        # (live) ROS path: EMA of the segment velocity the node computed from input stamps, per new message (0 = off, previous behaviour).
        # Same rule as PrecomputedHumanLink (which sets its own vel_alpha after this __init__).
        self.vel_alpha = float(vel_alpha); self.vf = None
        # (live) tracking loss (ROS path, virtual robot stage): no new message for loss_detect_s -> "lost" (last shapes kept),
        # for loss_hold_s -> "off" (human cost off: shapes and COMs moved 1 km away, velocity 0), next message -> "ok" at once.
        # 0 = off (previous behaviour). On the real robot a lost human should stop the robot instead (see live_t1.yml).
        self.loss_hold_s = float(loss_hold_s); self.loss_detect_s = float(loss_detect_s)
        self.loss_state = "wait"; self.last_msg_ns = None; self.loss_events = []; self.loss_since_ns = None
        self.status_label = None
        if self.loss_hold_s > 0 and gui:
            try:
                import omni.ui as ui
                self._status_win = ui.Window("human status", width=260, height=60)
                with self._status_win.frame:
                    self.status_label = ui.Label("human: waiting", style={"font_size": 22})
            except Exception as e:      # display only
                print(f"[human-loss] status window not created: {e}", flush=True)
        self.inp = inputs
        self.lock = threading.Lock()
        self.latest = None
        self.n_dyn = 0; self.last_pumped = 0; self.last_stamp_ns = None
        self.recv, self.upd = [], []
        self.display = tuple(display or ())
        self.last_h = None; self.last_raw = None
        self.pose_rec = []
        if node is not None:      # (1-3) node=None: 미리 계산한 프레임 (PrecomputedHumanLink), 구독 없음
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
            node.create_subscription(HumanInertiaState, "/effmass/human_inertia_state", self._on_msg, qos)
            # (F) 설정과 같게 HumanPose 도 구독해 수신만 기록 (4단계 HumanLink 와 같은 부하, 재생은 구독자 2 를 기다림)
            import step3_msgs as M
            from hrc_interfaces.msg import HumanPose
            self.M = M
            node.create_subscription(HumanPose, "/hrc/human_pose", self._on_pose, M.best_effort_qos())
        self.stage = stage
        if stage is not None and "instancer" in self.display:
            from pxr import Gf, UsdGeom, Vt
            self.Gf, self.Vt = Gf, Vt
            inst = UsdGeom.PointInstancer.Define(stage, f"{root}/instancer")
            proto = UsdGeom.Sphere.Define(stage, f"{root}/instancer/proto")
            proto.CreateRadiusAttr(1.0); proto.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.95, 0.75, 0.25)]))
            inst.CreatePrototypesRel().SetTargets([proto.GetPath()]); inst.CreateProtoIndicesAttr(Vt.IntArray([0] * 19))
            inst.CreateScalesAttr(Vt.Vec3fArray([Gf.Vec3f(r, r, r) for r in RAD]))
            self.inst_pos = inst.CreatePositionsAttr(Vt.Vec3fArray([Gf.Vec3f(10, 10, -10)] * 19))
        self.shapes_viz = bool(shapes_viz and stage is not None)
        self.shape_ops = None
        self.lines = None
        self.root = root
        self.mesh_link = None      # 선택: step4_human.HumanLink (메시만, GUI)

    # ---------- 수신 스레드 ----------
    def _on_pose(self, msg):
        kind, seq = self.M.parse_frame_id(msg.header.frame_id)
        self.pose_rec.append((seq, self.M.stamp_ns(msg.header), time.time_ns(), len(msg.smpl_vertices)))

    def _on_msg(self, m):
        t = time.time_ns()
        st = m.input_stamp.sec * 1_000_000_000 + m.input_stamp.nanosec
        d = dict(com=np.array(m.com_position, np.float32), vel=np.array(m.com_velocity, np.float32), inv=np.array(m.inv_inertia, np.float32),
                 p0=np.array(m.shape_p0, np.float32), p1=np.array(m.shape_p1, np.float32), rad=np.array(m.shape_radius, np.float32),
                 seg=np.array(m.shape_segment, np.int64), type=np.array(m.shape_type, np.uint8))
        with self.lock:
            self.n_dyn += 1
            self.latest = (self.n_dyn, st, t, int(m.input_count), d)
        self.recv.append((self.n_dyn, st, t, int(m.input_count), float(m.compute_ms), len(m.shape_radius)))

    # ---------- 메인 스레드 ----------
    def _make_shapes(self, h, types):
        from pxr import Gf, UsdGeom, Vt, Sdf
        self.shape_ops = []
        self.shape_idx = [i for i in range(len(h["rad"])) if float(h["rad"][i]) > 0]     # (v2) inactive padding shapes (radius 0) are not drawn
        for i in self.shape_idx:
            path = f"{self.root}/shapes/s{i:02d}"
            if int(types[i]) == 1:
                g = UsdGeom.Capsule.Define(self.stage, path); g.CreateAxisAttr("Z"); g.CreateRadiusAttr(float(h["rad"][i]))
                g.CreateHeightAttr(float(np.linalg.norm(h["p1"][i] - h["p0"][i])))
            else:
                g = UsdGeom.Sphere.Define(self.stage, path); g.CreateRadiusAttr(float(h["rad"][i]))
            g.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*CAPSULE_RGB)]))      # (B2a) one colour for all capsules
            g.CreateDisplayOpacityAttr(Vt.FloatArray([CAPSULE_OPACITY]))
            x = UsdGeom.Xformable(g); self.shape_ops.append((x.AddTranslateOp(), x.AddOrientOp()))

    def _make_mesh(self, faces, pts_isaac):
        # (B2a) display-only body mesh, opaque skin colour. Vertices are set by _set_mesh (Isaac world).
        from pxr import Gf, UsdGeom, Vt
        m = UsdGeom.Mesh.Define(self.stage, f"{self.root}/mesh")
        f = np.asarray(faces, np.int32); P = np.ascontiguousarray(pts_isaac, np.float32)
        m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P))                      # points, topology and extent set together at creation
        m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*P.min(0).tolist()), Gf.Vec3f(*P.max(0).tolist())]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(f))); m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(f.reshape(-1)))
        m.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*SKIN_RGB)])); m.CreateDisplayOpacityAttr(Vt.FloatArray([1.0])); m.CreateDoubleSidedAttr(True)
        m.CreateSubdivisionSchemeAttr("none")
        self.mesh_pts = m.GetPointsAttr(); self.mesh_ext = m.GetExtentAttr(); self.mesh_vt = Vt; self.mesh_gf = Gf

    def _set_mesh(self, pts_isaac):
        P = np.ascontiguousarray(pts_isaac, np.float32)
        self.mesh_pts.Set(self.mesh_vt.Vec3fArray.FromNumpy(P))
        self.mesh_ext.Set(self.mesh_vt.Vec3fArray([self.mesh_gf.Vec3f(*P.min(0).tolist()), self.mesh_gf.Vec3f(*P.max(0).tolist())]))

    def _update_shapes(self, h):
        Gf = self.Gf
        for i, (tr, ori) in zip(self.shape_idx, self.shape_ops):
            a, b = h["p0"][i], h["p1"][i]; c = 0.5 * (a + b); v = b - a; n = np.linalg.norm(v)
            z = np.array([0, 0, 1.0]); q = Gf.Quatf(1, 0, 0, 0)
            if n > 1e-9:
                v = v / n; ax = np.cross(z, v); s = np.linalg.norm(ax); cth = float(np.dot(z, v))
                if s > 1e-9:
                    ang = np.arctan2(s, cth); ax = ax / s; q = Gf.Quatf(float(np.cos(ang / 2)), *[float(x) for x in ax * np.sin(ang / 2)])
                elif cth < 0:
                    q = Gf.Quatf(0, 1, 0, 0)
            tr.Set(Gf.Vec3d(*[float(x) for x in c])); ori.Set(q)

    def pump(self):
        pc = time.perf_counter
        T = dict(pickup=0.0, transform=0.0, disp_instancer=0.0, disp_prims=0.0, tensor=0.0, verify=0.0, new=0, mesh_new=0, mesh_xf=0.0, mesh_usd=0.0, mesh_rt=0.0,
                 mesh_verify=0.0, shapes=0.0)
        a = pc()
        with self.lock:
            L = self.latest
        if L is not None and self.last_stamp_ns is not None:
            self.inp.set_age((time.time_ns() - self.last_stamp_ns) / 1e9)
        if L is None or L[0] == self.last_pumped:
            if self.loss_hold_s > 0:
                self._loss_tick()
            T["pickup"] = (pc() - a) * 1000
            return T
        n, st, trecv, cnt, d = L
        if self.loss_hold_s > 0:
            self._loss_new(trecv)
        self.last_pumped = n; self.last_stamp_ns = st
        if self.vel_alpha:
            vel = d["vel"].astype(np.float64)
            self.vf = vel.copy() if self.vf is None else self.vel_alpha * vel + (1.0 - self.vel_alpha) * self.vf
            d = dict(d); d["vel"] = self.vf
        b = pc()
        h = G.to_isaac(d, self.R, self.t)
        c = pc()
        self.inp.set_human(h); self.inp.set_age((time.time_ns() - st) / 1e9)
        e = pc()
        if "instancer" in self.display and hasattr(self, "inst_pos"):
            self.inst_pos.Set(self.Vt.Vec3fArray.FromNumpy(h["com"].astype(np.float32)))
        f = pc()
        if self.shapes_viz:
            if self.shape_ops is None:
                self._make_shapes(h, d["type"])
            self._update_shapes(h)
        g = pc()
        self.last_h = h; self.last_raw = d       # 실제로 GPU 에 넣은 메시지 (덤프·검증용)
        T.update(new=1, pickup=(b - a) * 1000, transform=(c - b) * 1000, tensor=(e - c) * 1000, disp_instancer=(f - e) * 1000, shapes=(g - f) * 1000)
        self.upd.append((n, st, trecv, cnt, time.time_ns(), T["transform"], T["tensor"], T["disp_instancer"], T["shapes"]))
        return T

    # ---------- (live) tracking loss ----------
    def _loss_event(self, state, now_ns, gap_s):
        self.loss_events.append((now_ns, {"ok": 0, "lost": 1, "off": 2, "back": 3}[state], gap_s))
        msg = {"lost": f"human lost: no message for {gap_s:.2f} s, last shapes kept (hold {self.loss_hold_s:.2f} s)",
               "off": f"human cost OFF: no message for {gap_s:.2f} s",
               "back": f"human back after {gap_s:.2f} s: human cost ON"}[state]
        print(f"[human-loss] {msg}", flush=True)
        if self.status_label is not None:
            self.status_label.text = {"lost": "human: LOST (hold)", "off": "human: OFF", "back": "human: OK"}[state]
        if self.shapes_viz and self.shape_ops is not None:
            from pxr import Gf, UsdGeom, Vt
            rgb = {"lost": (1.0, 0.55, 0.0), "back": CAPSULE_RGB}.get(state)
            if rgb is not None:
                for i in self.shape_idx:
                    UsdGeom.Gprim(self.stage.GetPrimAtPath(f"{self.root}/shapes/s{i:02d}")).GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*rgb)]))

    def _loss_new(self, trecv_ns):
        now = time.time_ns()
        if self.loss_state in ("lost", "off") and self.last_msg_ns is not None:
            self._loss_event("back", now, (trecv_ns - self.last_msg_ns) / 1e9)
        self.loss_state = "ok"; self.last_msg_ns = trecv_ns

    def _loss_tick(self):
        if self.loss_state in ("wait", "off") or self.last_msg_ns is None:
            return
        now = time.time_ns(); gap = (now - self.last_msg_ns) / 1e9
        if self.loss_state == "ok" and gap > self.loss_detect_s:
            self.loss_state = "lost"; self._loss_event("lost", now, gap)
        if self.loss_state == "lost" and gap > self.loss_hold_s and self.last_h is not None:
            far = dict(self.last_h); off = np.array([0.0, 0.0, 1000.0])
            far.update(com=self.last_h["com"] + off, p0=self.last_h["p0"] + off, p1=self.last_h["p1"] + off, vel=np.zeros_like(self.last_h["vel"]))
            self.inp.set_human(far)
            if "instancer" in self.display and hasattr(self, "inst_pos"):
                self.inst_pos.Set(self.Vt.Vec3fArray.FromNumpy(far["com"].astype(np.float32)))
            if self.shapes_viz and self.shape_ops is not None:
                self._update_shapes(far)
            self.loss_state = "off"; self._loss_event("off", now, gap)

    def set_lines(self, starts, ends, colors=None):
        """로봇 구 중심 → 가장 가까운 형상 축 위 점. starts/ends (N,3)"""
        from pxr import Gf, UsdGeom, Vt
        N = len(starts)
        if self.lines is None or self.lines[1] != N:
            cur = UsdGeom.BasisCurves.Define(self.stage, f"{self.root}/pair_lines"); cur.CreateTypeAttr("linear")
            cur.CreateCurveVertexCountsAttr(Vt.IntArray([2] * N)); cur.CreateWidthsAttr(Vt.FloatArray([0.006] * (2 * N)))
            cur.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.1, 0.1)]))
            self.lines = (cur.CreatePointsAttr(), N)
        P = np.empty((2 * N, 3), np.float32); P[0::2] = starts; P[1::2] = ends
        self.lines[0].Set(Vt.Vec3fArray.FromNumpy(P))

    @property
    def dyn_rec(self):          # step2b 종료 기록과 같은 이름 (HumanInertiaState 수신)
        return self.recv

    @property
    def upd_rec(self):
        return self.upd

    @property
    def mesh_live_info(self):
        return getattr(self.mesh_link, "mesh_live_info", None) if self.mesh_link is not None else None

    def mesh_final_check(self):
        return self.mesh_link.mesh_final_check() if self.mesh_link is not None else None

    def final_check(self):
        return dict(n_dyn=self.n_dyn, last_pumped=self.last_pumped, inputs_ptr_same=self.inp.pointers() == self.inp.addr0, n_human_set=self.inp.n_human,
                    n_robot_set=self.inp.n_robot)

    def save(self, prefix):
        if self.mesh_link is not None:
            self.mesh_link.save(prefix)
        np.savez(prefix + "_human_inertia_rec.npz", pose=np.array(self.pose_rec, dtype=np.int64).reshape(-1, 4), recv=np.array(self.recv, dtype=np.float64).reshape(-1, 6), upd=np.array(self.upd, dtype=np.float64).reshape(-1, 9))
        if self.loss_hold_s > 0:
            np.save(prefix + "_human_loss.npy", np.array(self.loss_events, dtype=np.float64).reshape(-1, 3))


class PrecomputedHumanLink(HumanInertiaLink):
    """(1-3, 1안) 결정적 재생: g13_human_precompute.py 로 미리 계산한 HumanInertiaState 내용을 시뮬레이션 시각으로 읽는다. ROS·노드 없음.
    pump(i): 스텝 i 의 시각 t_sim = (i+1)·dt_phys (그 스텝 world.step 뒤 측정 상태의 시각, 실시간 배율과 무관),
             프레임 = clip(floor(t_sim·rate + 1e-6) + delay_frames, 0, F−1). 새 프레임일 때만 to_isaac → set_human (ROS 경로와 같은 함수).
             age = t_sim − 입력 시각(프레임/rate) 를 매 스텝 갱신 (ROS 경로: 지금 − 입력 stamp). 등속 예측·τ_h 정의는 그대로.
    pump(-1) = t_sim 0 (예열 전 첫 프레임)."""

    def __init__(self, pre_npz, physics_dt, transform_json, inputs, delay_frames=0, stage=None, display=("instancer",), shapes_viz=False, root="/World/human",
                 mesh_npz=None, lead_s=0.0, lag_s=0.0, jitter_m=0.0, drop_frac=0.0, noise_seed=0, vel_alpha=0.0):
        super().__init__(None, transform_json, inputs, stage=stage, display=display, shapes_viz=shapes_viz, root=root)
        # (B2a) display-only mesh: vertices precomputed with the node body model (b2a_mesh.py), separate file, not used by the cost path
        self.mesh_V = None; self.mesh_ms = []
        if mesh_npz and stage is not None:
            zm = np.load(mesh_npz); self.mesh_V = zm["verts"]; self.mesh_path = mesh_npz
            self._make_mesh(zm["faces"], zm["verts"][0].astype(np.float64) @ self.R.T + self.t)
        z = np.load(pre_npz)
        self.pre = {k: z[k] for k in ("com", "vel", "inv", "p0", "p1", "input_time")}
        self.pre_static = dict(rad=z["shape_radius"].astype(np.float32), seg=z["shape_segment"].astype(np.int64), type=z["shape_type"].astype(np.uint8))
        self.rate = float(z["rate"]); self.F = len(self.pre["com"]); self.dt = float(physics_dt); self.delay = int(delay_frames)
        if self.mesh_V is not None:
            assert len(self.mesh_V) == self.F, ("mesh frames", len(self.mesh_V), self.F)
        self.pre_path = pre_npz; self.step_frames = []; self.n_clamped = 0; self.cur_frame = None
        # (overnight) optional perception perturbations of what the controller sees; all 0 = previous behaviour (same code path, bit-equal).
        # lag_s: frame chosen at t_sim - lag_s. drop_frac: that fraction of frames is missing (seeded), the last delivered frame is kept.
        # jitter_m: per frame, per segment Gaussian offset (std jitter_m, seeded) added to the segment COM and its shapes; vel gets the matching difference.
        # lead_s: shapes and COM moved ahead by vel[segment] * lead_s (file velocity = backward difference, causal).
        self.lead_s, self.lag_s, self.jitter_m, self.drop_frac = float(lead_s), float(lag_s), float(jitter_m), float(drop_frac)
        # vel_alpha: exponential moving average of the delivered segment velocity (v_f = a v + (1 - a) v_f_prev, per new frame); 0 = off.
        self.vel_alpha = float(vel_alpha); self.vf = None
        self.pert = bool(self.lead_s or self.lag_s or self.jitter_m or self.drop_frac or self.vel_alpha)
        if self.pert:
            rng = np.random.default_rng(int(noise_seed))
            keep = np.ones(self.F, bool)
            if self.drop_frac > 0:
                keep = rng.random(self.F) >= self.drop_frac; keep[0] = True
            self.last_kept = np.maximum.accumulate(np.where(keep, np.arange(self.F), 0))
            nseg = self.pre["com"].shape[1]
            self.jit = rng.normal(0.0, self.jitter_m, (self.F, nseg, 3)) if self.jitter_m > 0 else None
            self.shape_seg = self.pre_static["seg"]

    def _perturb(self, d, fc):
        com = d["com"].reshape(-1, 3).astype(np.float64); vel = d["vel"].reshape(-1, 3).astype(np.float64)
        p0 = d["p0"].reshape(-1, 3).astype(np.float64); p1 = d["p1"].reshape(-1, 3).astype(np.float64); sg = self.shape_seg
        if self.jit is not None:
            o = self.jit[fc]; com = com + o; p0 = p0 + o[sg]; p1 = p1 + o[sg]
            vel = vel + (o - self.jit[max(fc - 1, 0)]) * self.rate
        if self.vel_alpha:
            self.vf = vel.copy() if self.vf is None else self.vel_alpha * vel + (1.0 - self.vel_alpha) * self.vf
            vel = self.vf
        if self.lead_s:
            com = com + vel * self.lead_s; p0 = p0 + vel[sg] * self.lead_s; p1 = p1 + vel[sg] * self.lead_s
        d = dict(d); d.update(com=com.reshape(-1), vel=vel.reshape(-1), p0=p0.reshape(-1), p1=p1.reshape(-1)); return d

    def frame_at(self, i):
        t = (i + 1) * self.dt
        if self.lag_s:
            f = int(np.floor((t - self.lag_s) * self.rate + 1e-6)) + self.delay
        else:
            f = int(np.floor(t * self.rate + 1e-6)) + self.delay
        fc = min(max(f, 0), self.F - 1)
        if self.pert and self.drop_frac > 0:
            fc = int(self.last_kept[fc])
        return t, f, fc

    def pump(self, i=-1):
        pc = time.perf_counter
        T = dict(pickup=0.0, transform=0.0, disp_instancer=0.0, disp_prims=0.0, tensor=0.0, verify=0.0, new=0, mesh_new=0, mesh_xf=0.0, mesh_usd=0.0, mesh_rt=0.0,
                 mesh_verify=0.0, shapes=0.0)
        a = pc()
        t, f, fc = self.frame_at(i)
        self.n_clamped += int(f != fc)
        age = t - float(self.pre["input_time"][fc])
        self.step_frames.append((i, fc, age))
        if fc == self.cur_frame:
            self.inp.set_age(age)
            T["pickup"] = (pc() - a) * 1000
            return T
        self.cur_frame = fc
        d = dict(com=self.pre["com"][fc].reshape(-1), vel=self.pre["vel"][fc].reshape(-1), inv=self.pre["inv"][fc].reshape(-1), p0=self.pre["p0"][fc].reshape(-1),
                 p1=self.pre["p1"][fc].reshape(-1), **self.pre_static)
        if self.pert:
            d = self._perturb(d, fc)
        self.n_dyn += 1; n = self.n_dyn; self.last_pumped = n; self.last_stamp_ns = None
        self.recv.append((n, fc, i, fc, 0.0, len(d["rad"])))
        b = pc()
        h = G.to_isaac(d, self.R, self.t)
        c = pc()
        self.inp.set_human(h); self.inp.set_age(age)
        e = pc()
        if "instancer" in self.display and hasattr(self, "inst_pos"):
            self.inst_pos.Set(self.Vt.Vec3fArray.FromNumpy(h["com"].astype(np.float32)))
        f_ = pc()
        if self.shapes_viz:
            if self.shape_ops is None:
                self._make_shapes(h, d["type"])
            self._update_shapes(h)
        g = pc()
        if self.mesh_V is not None:
            self._set_mesh(self.mesh_V[fc].astype(np.float64) @ self.R.T + self.t)
            T["mesh_usd"] = (pc() - g) * 1000; self.mesh_ms.append(T["mesh_usd"])
        self.last_h = h; self.last_raw = d
        T.update(new=1, pickup=(b - a) * 1000, transform=(c - b) * 1000, tensor=(e - c) * 1000, disp_instancer=(f_ - e) * 1000, shapes=(g - f_) * 1000)
        self.upd.append((n, fc, i, fc, time.time_ns(), T["transform"], T["tensor"], T["disp_instancer"], T["shapes"]))
        return T

    def final_check(self):
        r = super().final_check()
        sf = np.array(self.step_frames, np.float64).reshape(-1, 3)
        r.update(precomputed=self.pre_path, delay_frames=self.delay, frames_total=self.F, n_clamped=self.n_clamped,
                 frame_first_last=[int(sf[0, 1]), int(sf[-1, 1])] if len(sf) else None, age_min_max=[float(sf[:, 2].min()), float(sf[:, 2].max())] if len(sf) else None)
        return r

    def save(self, prefix):
        super().save(prefix)
        np.save(prefix + "_human_step_frames.npy", np.array(self.step_frames, np.float64).reshape(-1, 3))
