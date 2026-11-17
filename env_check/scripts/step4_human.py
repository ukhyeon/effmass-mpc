"""step4_human.py — Isaac 쪽 사람 데이터 처리 (4-1 ~ 4-3 공통). rclpy·hrc_interfaces 는 호출 전에 import 가능해야 함.

  HumanLink(node, transform_json, stage=None, display=("instancer",), tensor=True, verify_every=1)
    - 구독: /hrc/human_pose (HumanPose), /hrc/human_dynamics_state (HumanDynamicsState), QoS 는 실제 노드와 같음
      콜백(수신 스레드): 수신 시각·stamp·순번 기록, HumanDynamicsState 는 위치·속도를 numpy 로 복사해 '최신 슬롯'에 넣음
    - pump() (메인 스레드): 새 HumanDynamicsState 가 있을 때만
        1) 임시 변환: p = R p_w + t, v = R v_w  (step4/temp_transform.json)
        2) 표시: "instancer" = UsdGeom.PointInstancer positions 한 번 쓰기 / "prims" = 구 prim 19개 각각 translate Set
        3) 고정 GPU 텐서: 미리 만든 torch CUDA 텐서 pos(19,3), vel(19,3), stamp(1) 에 pinned CPU 스테이징 → copy_(non_blocking)
        4) verify_every 마다: 텐서 주소 불변, 값 == 변환된 메시지 값 (동기화 후 비교, 시간은 따로 기록)
      반환: 항목별 시간(ms) dict
  (4-5) mesh=("usd",) / ("rt",) / ("usd","rt"): SMPL 메시 (HumanPose.smpl_vertices, 카메라 좌표 6890×3)
      수신 콜백: 정점을 numpy 로 복사해 '최신 메시 슬롯'에. pump(): 새 HumanPose 가 있을 때만
        변환 p = v @ A + b (step4/mesh_transform.json, mesh_offset "node" | "rigid") 한 번 → 방법별 한 번에 쓰기
        "usd": UsdGeom.Mesh points 속성 Set(Vt.Vec3fArray.FromNumpy) / "rt": USDRT(Fabric) 같은 prim 의 points 속성 Set(usdrt.Vt.Vec3fArray)
        방법마다 따로 prim (/World/human/mesh_usd, mesh_rt). mesh_verify_every 마다 다시 읽어 값 비교 (시간 따로)
  기록: self.pose_rec, self.dyn_rec, self.cb_ms, self.upd_rec — save(prefix) 로 저장
"""
import json
import os
import threading
import time

import numpy as np

RAD = [0.12, 0.08, 0.08, 0.11, 0.06, 0.06, 0.05, 0.05, 0.12, 0.06, 0.06, 0.06, 0.10, 0.05, 0.05, 0.045, 0.045, 0.045, 0.045]


class HumanLink:
    def __init__(self, node, transform_json, stage=None, display=("instancer",), tensor=True, verify_every=1,
                 root="/World/human", device="cuda:0", mesh=(), mesh_transform_json=None, mesh_offset="node",
                 mesh_faces=None, mesh_verify_every=0):
        import step3_msgs as M
        from hrc_interfaces.msg import HumanDynamicsState, HumanPose

        tf = json.load(open(transform_json))
        self.R = np.asarray(tf["R"], np.float64)
        self.t = np.asarray(tf["t"], np.float64)
        self.M = M
        self.lock = threading.Lock()
        self.latest = None          # (seq_no, stamp_ns, t_recv_ns, pos(19,3) f32, vel(19,3) f32)
        self.n_dyn = 0
        self.last_pumped = 0
        self.pose_rec, self.dyn_rec, self.cb_ms, self.upd_rec = [], [], {"pose": [], "dyn": []}, []
        self.display = tuple(display or ())
        self.verify_every = verify_every
        self.verify_fail = 0
        self.verify_n = 0
        # (4-5) 메시
        self.mesh = tuple(mesh or ())
        self.latest_mesh = None      # (n, seq, stamp_ns, t_recv_ns, verts (6890,3) f32 카메라 좌표)
        self.n_mesh = 0
        self.last_mesh = 0
        self.mesh_rec = []
        self.mesh_verify_every = mesh_verify_every
        self.mesh_verify_n = 0
        self.mesh_verify_fail = 0
        self.rt_attr = None
        if self.mesh:
            mt = json.load(open(mesh_transform_json))
            self.mA = np.asarray(mt["A"], np.float64)
            if mesh_offset == "node_live":
                # (5단계) 첫 메시지의 betas·smpl_joint[0]·global_trans 로 노드 관례 오프셋 계산 (step5_make_transform.py)
                self.mb = None
                self.m_b_rigid = np.asarray(mt["b_rigid"], np.float64); self.m_camR = np.asarray(mt["cam_R"], np.float64)
                self.m_R = np.asarray(mt["R"], np.float64); self.m_j0a = np.asarray(mt["j0_a"], np.float64); self.m_j0B = np.asarray(mt["j0_B"], np.float64)
                self.mesh_live_info = None
            else:
                self.mb = np.asarray(mt["b_node" if mesh_offset == "node" else "b_rigid"], np.float64)
            self.mesh_offset = mesh_offset
            from pxr import Gf, UsdGeom, Vt
            self.Vt = Vt
            faces = np.load(mesh_faces).astype(np.int32)
            self.mesh_paths = {}
            for k, meth in enumerate(self.mesh):
                path = f"{root}/mesh_{meth}"
                mm = UsdGeom.Mesh.Define(stage, path)
                mm.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
                mm.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.reshape(-1)))
                mm.CreateSubdivisionSchemeAttr("none")
                mm.CreateDoubleSidedAttr(True)
                col = [Gf.Vec3f(0.85, 0.65, 0.55), Gf.Vec3f(0.55, 0.75, 0.85)][k % 2]
                mm.CreateDisplayColorAttr(Vt.Vec3fArray([col]))
                self.mesh_paths[meth] = path
                if meth == "usd":
                    self.usd_mesh_attr = mm.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.full((6890, 3), 10.0, np.float32)))
                else:
                    mm.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.full((6890, 3), 10.0, np.float32)))
        node.create_subscription(HumanPose, "/hrc/human_pose", self._on_pose, M.best_effort_qos())
        node.create_subscription(HumanDynamicsState, "/hrc/human_dynamics_state", self._on_dyn, M.best_effort_qos())
        # 표시
        if stage is not None and self.display:
            from pxr import Gf, UsdGeom, Vt
            self.Gf, self.Vt = Gf, Vt
            if "instancer" in self.display:
                inst = UsdGeom.PointInstancer.Define(stage, f"{root}/instancer")
                proto = UsdGeom.Sphere.Define(stage, f"{root}/instancer/proto")
                proto.CreateRadiusAttr(1.0)
                proto.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.95, 0.75, 0.25)]))
                inst.CreatePrototypesRel().SetTargets([proto.GetPath()])
                inst.CreateProtoIndicesAttr(Vt.IntArray([0] * 19))
                inst.CreateScalesAttr(Vt.Vec3fArray([Gf.Vec3f(r, r, r) for r in RAD]))
                self.inst_pos = inst.CreatePositionsAttr(Vt.Vec3fArray([Gf.Vec3f(10, 10, -10)] * 19))
            if "prims" in self.display:
                self.prim_attr = []
                for k in range(19):
                    s = UsdGeom.Sphere.Define(stage, f"{root}/prims/seg_{k:02d}")
                    s.CreateRadiusAttr(RAD[k])
                    s.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.3, 0.6, 0.95)]))
                    op = UsdGeom.Xformable(s).AddTranslateOp()
                    op.Set(Gf.Vec3d(10, 10, -10))
                    self.prim_attr.append(op.GetAttr())
        # 고정 GPU 텐서 (MPC 비용이 읽을 형태, HumanDynamicsState 가 주는 것: 분절 위치·속도 + stamp)
        self.tensor = tensor
        if tensor:
            import torch
            self.torch = torch
            self.pos_t = torch.zeros((19, 3), dtype=torch.float32, device=device)
            self.vel_t = torch.zeros((19, 3), dtype=torch.float32, device=device)
            self.stamp_t = torch.zeros((1,), dtype=torch.int64, device=device)
            self.pos_c = torch.zeros((19, 3), dtype=torch.float32).pin_memory()
            self.vel_c = torch.zeros((19, 3), dtype=torch.float32).pin_memory()
            self.stamp_c = torch.zeros((1,), dtype=torch.int64).pin_memory()
            self.ptr0 = (self.pos_t.data_ptr(), self.vel_t.data_ptr(), self.stamp_t.data_ptr())

    # ---------------- 수신 스레드 ----------------
    def _on_pose(self, msg):
        t = time.time_ns(); c0 = time.perf_counter()
        kind, seq = self.M.parse_frame_id(msg.header.frame_id)
        nv = len(msg.smpl_vertices)
        self.pose_rec.append((seq, self.M.stamp_ns(msg.header), t, nv))
        if self.mesh and nv == 6890 * 3:
            V = np.frombuffer(msg.smpl_vertices, dtype=np.float32).reshape(6890, 3).copy()
            if self.mesh_offset == "node_live" and self.mb is None and len(msg.smpl_joint) >= 3:
                be = np.zeros(10); be[:min(10, len(msg.smpl_betas))] = np.asarray(msg.smpl_betas, np.float64)[:10]
                j0m = np.asarray(msg.smpl_joint[:3], np.float64) - np.asarray(msg.global_trans, np.float64)
                off = (self.m_j0a + self.m_j0B @ be) - self.m_camR.T @ j0m
                self.mb = self.m_b_rigid + off @ self.m_R.T
                self.mesh_live_info = dict(betas=be.tolist(), j0_mesh_cam=j0m.tolist(), offset_nodeworld=off.tolist(), b=self.mb.tolist())
            with self.lock:
                self.n_mesh += 1
                self.latest_mesh = (self.n_mesh, seq, self.M.stamp_ns(msg.header), t, V)
        self.cb_ms["pose"].append((time.perf_counter() - c0) * 1000)

    def _on_dyn(self, msg):
        t = time.time_ns(); c0 = time.perf_counter()
        pos = np.frombuffer(msg.body_part_positions, dtype=np.float32).reshape(19, 3).copy()
        vel = np.frombuffer(msg.body_part_velocities, dtype=np.float32).reshape(19, 3).copy()
        st = self.M.stamp_ns(msg.header)
        with self.lock:
            self.n_dyn += 1
            self.latest = (self.n_dyn, st, t, pos, vel)
        self.dyn_rec.append((st, t, pos, vel))
        self.cb_ms["dyn"].append((time.perf_counter() - c0) * 1000)

    # ---------------- 메인 스레드 ----------------
    def _rt_attach(self):
        import omni.usd
        from usdrt import Usd as RtUsd, Vt as RtVt
        self.RtVt = RtVt
        rs = RtUsd.Stage.Attach(omni.usd.get_context().get_stage_id())
        self.rt_attr = rs.GetPrimAtPath(self.mesh_paths["rt"]).GetAttribute("points")

    def _pump_mesh(self, T):
        pc = time.perf_counter
        with self.lock:
            L = self.latest_mesh
        if L is None or L[0] == self.last_mesh or self.mb is None:
            return
        n, seq, st, trecv, V = L
        self.last_mesh = n
        a = pc()
        P = (V.astype(np.float64) @ self.mA + self.mb).astype(np.float32)
        b = pc()
        if "usd" in self.mesh:
            self.usd_mesh_attr.Set(self.Vt.Vec3fArray.FromNumpy(P))
        c = pc()
        if "rt" in self.mesh:
            if self.rt_attr is None:
                self._rt_attach()
            self.rt_attr.Set(self.RtVt.Vec3fArray(P))
        d = pc()
        ok = None
        if self.mesh_verify_every and n % self.mesh_verify_every == 0:
            ok = True
            if "usd" in self.mesh:
                ok &= bool(np.array_equal(np.asarray(self.usd_mesh_attr.Get(), np.float32), P))
            if "rt" in self.mesh:
                ok &= bool(np.array_equal(np.asarray(self.rt_attr.Get(), np.float32), P))
            self.mesh_verify_n += 1
            self.mesh_verify_fail += int(not ok)
        e = pc()
        T.update(mesh_new=1, mesh_xf=(b - a) * 1000, mesh_usd=(c - b) * 1000, mesh_rt=(d - c) * 1000, mesh_verify=(e - d) * 1000)
        self.mesh_rec.append((n, seq, st, trecv, time.time_ns(), T["mesh_xf"], T["mesh_usd"], T["mesh_rt"], T["mesh_verify"],
                              -1 if ok is None else int(ok)))
        self._last_mesh_P = P

    def pump(self):
        pc = time.perf_counter
        T = dict(pickup=0.0, transform=0.0, disp_instancer=0.0, disp_prims=0.0, tensor=0.0, verify=0.0, new=0,
                 mesh_new=0, mesh_xf=0.0, mesh_usd=0.0, mesh_rt=0.0, mesh_verify=0.0)
        if self.mesh:
            self._pump_mesh(T)
        a = pc()
        with self.lock:
            L = self.latest
        if L is None or L[0] == self.last_pumped:
            T["pickup"] = (pc() - a) * 1000
            return T
        n, st, trecv, pos_w, vel_w = L
        self.last_pumped = n
        b = pc()
        P = (pos_w.astype(np.float64) @ self.R.T + self.t).astype(np.float32)
        V = (vel_w.astype(np.float64) @ self.R.T).astype(np.float32)
        c = pc()
        if "instancer" in self.display and hasattr(self, "inst_pos"):
            self.inst_pos.Set(self.Vt.Vec3fArray.FromNumpy(P))
        d = pc()
        if "prims" in self.display and hasattr(self, "prim_attr"):
            Gf = self.Gf
            for k in range(19):
                self.prim_attr[k].Set(Gf.Vec3d(float(P[k, 0]), float(P[k, 1]), float(P[k, 2])))
        e = pc()
        if self.tensor:
            self.pos_c.numpy()[:] = P
            self.vel_c.numpy()[:] = V
            self.stamp_c[0] = st
            self.pos_t.copy_(self.pos_c, non_blocking=True)
            self.vel_t.copy_(self.vel_c, non_blocking=True)
            self.stamp_t.copy_(self.stamp_c, non_blocking=True)
        f = pc()
        ok = None
        if self.tensor and self.verify_every and (n % self.verify_every == 0):
            self.torch.cuda.current_stream().synchronize()
            same_ptr = (self.pos_t.data_ptr(), self.vel_t.data_ptr(), self.stamp_t.data_ptr()) == self.ptr0
            ok = bool(same_ptr and np.array_equal(self.pos_t.cpu().numpy(), P) and np.array_equal(self.vel_t.cpu().numpy(), V)
                      and int(self.stamp_t.cpu()[0]) == st)
            self.verify_n += 1
            self.verify_fail += int(not ok)
        g = pc()
        T.update(new=1, pickup=(b - a) * 1000, transform=(c - b) * 1000, disp_instancer=(d - c) * 1000,
                 disp_prims=(e - d) * 1000, tensor=(f - e) * 1000, verify=(g - f) * 1000)
        self.upd_rec.append((n, st, trecv, time.time_ns(), T["transform"], T["disp_instancer"], T["disp_prims"], T["tensor"],
                             T["verify"], -1 if ok is None else int(ok)))
        return T

    def mesh_final_check(self):
        """마지막으로 쓴 메시 값이 prim 에 그대로 있는지 (usd / rt)"""
        if not self.mesh or not hasattr(self, "_last_mesh_P"):
            return None
        r = dict(last_mesh=self.last_mesh, n_mesh=self.n_mesh, verify_n=self.mesh_verify_n, verify_fail=self.mesh_verify_fail)
        if "usd" in self.mesh:
            r["usd_equal"] = bool(np.array_equal(np.asarray(self.usd_mesh_attr.Get(), np.float32), self._last_mesh_P))
        if "rt" in self.mesh:
            r["rt_equal"] = bool(np.array_equal(np.asarray(self.rt_attr.Get(), np.float32), self._last_mesh_P))
        return r

    def final_check(self):
        """마지막 메시지 기준 텐서 주소·값 확인 (동기화 포함)"""
        if not self.tensor or self.latest is None:
            return None
        self.torch.cuda.synchronize()
        n, st, trecv, pos_w, vel_w = self.latest
        P = (pos_w.astype(np.float64) @ self.R.T + self.t).astype(np.float32)
        same_ptr = (self.pos_t.data_ptr(), self.vel_t.data_ptr(), self.stamp_t.data_ptr()) == self.ptr0
        val = np.array_equal(self.pos_t.cpu().numpy(), P) if n == self.last_pumped else None
        return dict(same_ptr=same_ptr, value_equal_last=val, last_pumped=self.last_pumped, n_dyn=n)

    def save(self, prefix):
        np.savez(prefix + "_human_rec.npz",
                 pose=np.array([r[:4] for r in self.pose_rec], dtype=np.int64).reshape(-1, 4),
                 dyn_stamp=np.array([r[0] for r in self.dyn_rec], dtype=np.int64),
                 dyn_recv=np.array([r[1] for r in self.dyn_rec], dtype=np.int64),
                 dyn_pos=np.array([r[2] for r in self.dyn_rec], dtype=np.float32).reshape(-1, 19, 3),
                 dyn_vel=np.array([r[3] for r in self.dyn_rec], dtype=np.float32).reshape(-1, 19, 3),
                 cb_pose_ms=np.array(self.cb_ms["pose"]), cb_dyn_ms=np.array(self.cb_ms["dyn"]),
                 upd=np.array(self.upd_rec, dtype=np.float64).reshape(-1, 10),
                 mesh_upd=np.array(self.mesh_rec, dtype=np.float64).reshape(-1, 10))
