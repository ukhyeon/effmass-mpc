"""g1_meff_gpu.py — 1-2d: MPC rollout 의 각 (후보 b, 시점 h, 로봇 구 s) 에서 사람 형상까지 거리·형상·분절·u·m_h·m_r·μ 를 GPU 로 계산.

  MeffInputs   : 고정 GPU 입력 텐서 (사람 분절 COM·속도·Λ_h⁻¹, 형상 24개, 로봇 구별 Λ_r⁻¹, 시간) + pinned 스테이징 → copy_(non_blocking)
                 주소는 만든 뒤 바뀌지 않음 (pointers() 로 확인)
  MeffChannel  : 프로토타입 TargetCostChannel(ref/franka-depth-mppi experiment/base_target_cost.py) 방식.
                 각 ParallelMPPI optimizer 의 rollout_fn.cost_fn 을 CUDA graph 캡처 전에 감싼다.
                 가중치 0 = 비용에 더하지 않음: 원래 비용(base)을 그대로 반환하고, 계산 결과는 고정 디버그 텐서에 copy_ 만 한다.
  계산 정의 (1-2d 지시):
    - 거리 = |로봇 구 중심 − 형상 축(선분) 위 가장 가까운 점| − 형상 반지름 − 로봇 구 반지름 (구 형상은 p0 = p1). 음수 = 겹침
    - 가장 가까운 형상 k = argmin 거리 (같으면 앞 번호), 분절 = shape_seg[k]
    - u: "com" (기본, Indy7 정의) = 로봇 구 중심 → 소속 분절 COM, "closest" = 로봇 구 중심 → 형상 위 가장 가까운 점. 단위 벡터 (|·| + 1e-9 로 나눔)
    - a_h = uᵀ Λ_h⁻¹[seg] u, a_r = uᵀ Λ_r⁻¹[s] u ((−u) 와 같음), m_h = 1/a_h, m_r = 1/a_r (a_r = 0 → ∞), μ = 1/(a_h + a_r) (a_r = 0 → μ = m_h)
    - 사람 위치 시간 처리: "static" (기본, 현재 위치) / "cv" (분절 등속: 형상·COM 을 소속 분절 COM 속도로 τ_h = age + Σ_{k=1..h} traj_dt_k 만큼 평행 이동)
  구현 단계 (시간 줄이기 순서, 앞 단계를 포함해 누적):
    "pairs"  : 모든 (구, 형상) 쌍에서 거리·u·a_h·a_r·μ 계산 후 가장 가까운 형상 값 선택 (기본 구현)
    "skip0"  : + 역관성 0 인 구(link0·link1 축 위)는 a_r 계산 생략 (a_r = 0 고정, m_h·μ = m_h 는 계산)
    "nearest": + 가장 가까운 형상 하나에 대해서만 u·a_h·a_r·μ 계산
    "warp"   : + warp 커널 하나 (구마다 스레드 하나, 형상 24개 순회, 중간 텐서 없음)
    hfront>0 : + horizon 앞 hfront 스텝만 계산 (마지막 수단)
  CUDA graph 조건: 캡처 안 함수에서 호스트 데이터로 텐서를 만들지 않음(상수·인덱스는 미리 GPU 에), .item()/.cpu()/boolean mask/nonzero 없음,
    모든 크기 고정. torch 구현의 중간 결과는 그래프 메모리 풀에 잡힌다 (warp 구현은 중간 텐서 없음).
"""
import types

import numpy as np
import torch

N_SEG = 19
N_SHAPES = 24
IMPLS = ("pairs", "skip0", "nearest", "warp")

ISO_JSON = "/home/robotics/effmass_mpc/goal1/data/iso_15066.json"   # (B1) indy_ws ISO15066/iso_15066.json 사본
ISO_SEGMENTS = ["pelvis", "left_thigh", "right_thigh", "abdomen", "left_calf", "right_calf", "left_foot", "right_foot", "chest", "neck",
                "left_shoulder", "right_shoulder", "head", "left_upper_arm", "right_upper_arm", "left_lower_arm", "right_lower_arm", "left_hand", "right_hand"]
ISO_ASSUMED = {"left_foot": "left_calf", "right_foot": "right_calf"}   # JSON 에 발 없음 → 종아리 값 (가정)


def iso_mass_19(path=ISO_JSON):
    """분절 19개 ISO 15066 m_h_iso [kg] (collision_assessment_node.cpp initHumanBodyIndexMap 순서 = effmass_human 순서)"""
    import json
    j = json.load(open(path))
    return np.array([float(j[ISO_ASSUMED.get(n, n)]["m_h_iso"]) for n in ISO_SEGMENTS], np.float64)



class MeffInputs:
    def __init__(self, n_spheres, traj_dt, device="cuda:0"):
        self.S, self.device = int(n_spheres), device
        z = lambda *s, dt=torch.float32: torch.zeros(s, dtype=dt, device=device)
        self.com, self.vel, self.inv_h = z(N_SEG, 3), z(N_SEG, 3), z(N_SEG, 3, 3)
        self.p0, self.p1, self.rad, self.seg = z(N_SHAPES, 3), z(N_SHAPES, 3), z(N_SHAPES), z(N_SHAPES, dt=torch.long)
        self.dvec, self.invl2, self.seg32 = z(N_SHAPES, 3), z(N_SHAPES), z(N_SHAPES, dt=torch.int32)   # warp 용: p1−p0, 1/max(|p1−p0|²,1e-12), 분절 int32
        self.inv_r = z(self.S, 3, 3)
        self.age = z(1)
        td = torch.as_tensor(np.asarray(traj_dt, np.float32).reshape(-1), device=device)
        # 상태 h 의 시각 = Σ_{k=1..h} traj_dt_k (상태 0 = 현재 시작 상태, 0 s). cuRobo curobolib/cpp/tensor_step_kernel.cu
        # acceleration_loop(_idx)(_rk2)_kernel: out_pos[0] = start_position, h ≥ 1 에서 dt[h] 로 적분 [확인, 1-2d]
        self.tau = torch.cumsum(td, 0) - td[0]          # (H,) 상수
        self.names = ["com", "vel", "inv_h", "p0", "p1", "rad", "seg", "dvec", "invl2", "seg32", "inv_r", "age", "tau"]
        self.pin = {k: torch.zeros_like(getattr(self, k), device="cpu").pin_memory() for k in self.names if k != "tau"}
        self.addr0 = self.pointers()
        self.n_human = 0; self.n_robot = 0; self.age_val = 0.0

    def pointers(self):
        return [getattr(self, k).data_ptr() for k in self.names]

    def _put(self, k, arr):
        p = self.pin[k]; p.numpy()[...] = arr
        getattr(self, k).copy_(p, non_blocking=True)

    def set_human(self, h):
        """h: Isaac 좌표로 바꾼 dict (com (19,3), vel (19,3), inv_h (19,3,3), p0, p1 (24,3), rad (24), seg (24))"""
        for k in ("com", "vel", "inv_h", "p0", "p1", "rad", "seg"):
            self._put(k, h[k])
        dv = (np.asarray(h["p1"], np.float32) - np.asarray(h["p0"], np.float32))     # GPU torch 경로 (p1−p0, float32) 와 같은 값
        self._put("dvec", dv); self._put("invl2", 1.0 / np.maximum((dv * dv).sum(1), np.float32(1e-12))); self._put("seg32", h["seg"])
        self.n_human += 1

    def set_age(self, age_s):
        self._put("age", np.float32(age_s)); self.age_val = float(np.float32(age_s))

    def set_robot(self, inv_r):
        self._put("inv_r", inv_r)
        self.n_robot += 1


def to_isaac(msg, R, t):
    """HumanInertiaState 배열 (노드 world) → Isaac world: p' = R p + t, v' = R v, Λ' = R Λ Rᵀ. float64 로 계산 후 float32"""
    R = np.asarray(R, np.float64); t = np.asarray(t, np.float64)
    com = np.asarray(msg["com"], np.float64).reshape(N_SEG, 3); vel = np.asarray(msg["vel"], np.float64).reshape(N_SEG, 3)
    inv = np.asarray(msg["inv"], np.float64).reshape(N_SEG, 3, 3)
    p0 = np.asarray(msg["p0"], np.float64).reshape(-1, 3); p1 = np.asarray(msg["p1"], np.float64).reshape(-1, 3)
    assert len(p0) == N_SHAPES, len(p0)
    return dict(com=com @ R.T + t, vel=vel @ R.T, inv_h=np.einsum("ij,sjk,lk->sil", R, inv, R), p0=p0 @ R.T + t, p1=p1 @ R.T + t,
                rad=np.asarray(msg["rad"], np.float64).reshape(-1), seg=np.asarray(msg["seg"], np.int64).reshape(-1))


# ---------------- warp 커널 ----------------
_WP = {}


def _warp():
    if "k" in _WP:
        return _WP["wp"], _WP["k"]
    import warp as wp

    @wp.kernel
    def meff_kernel(sph: wp.array(dtype=wp.vec4), n_sall: int, n_s: int, H: int, Hf: int,
                    p0: wp.array(dtype=wp.vec3), dvec: wp.array(dtype=wp.vec3), invl2: wp.array(dtype=float), rad: wp.array(dtype=float), seg: wp.array(dtype=wp.int32),
                    com: wp.array(dtype=wp.vec3), vel: wp.array(dtype=wp.vec3), inv_h: wp.array(dtype=wp.mat33), inv_r: wp.array(dtype=wp.mat33),
                    mass_on: wp.array(dtype=wp.int32), tau: wp.array(dtype=float), age: wp.array(dtype=float), cv: int, use_com: int, n_shapes: int, noop: int,
                    o_dist: wp.array(dtype=float), o_shape: wp.array(dtype=wp.int32), o_seg: wp.array(dtype=wp.int32), o_u: wp.array(dtype=wp.vec3),
                    o_mh: wp.array(dtype=float), o_mr: wp.array(dtype=float), o_mu: wp.array(dtype=float),
                    par: wp.array(dtype=float), mode: wp.array(dtype=wp.int32), agg: wp.array(dtype=wp.int32), inv_iso: wp.array(dtype=float),
                    o_cost: wp.array(dtype=float), o_eta: wp.array(dtype=float), o_kc: wp.array(dtype=wp.int32), o_fb: wp.array(dtype=wp.int32)):
        tid = wp.tid()
        if noop == 1:
            return
        s = tid % n_s
        r = tid // n_s
        h = r % Hf
        b = r // Hf
        x = sph[(b * H + h) * n_sall + s]
        c = wp.vec3(x[0], x[1], x[2])
        tt = float(0.0)
        if cv == 1:
            tt = age[0] + tau[h]
        best = float(1.0e10)
        bk = int(0)
        bq = wp.vec3(c[0], c[1], c[2])
        for k in range(n_shapes):
            a = p0[k]
            if cv == 1:
                a = a + vel[seg[k]] * tt
            d = dvec[k]
            t = wp.clamp(wp.dot(c - a, d) * invl2[k], 0.0, 1.0)
            q = a + d * t
            dd = wp.length(q - c) - rad[k] - x[3]
            if dd < best:
                best = dd
                bk = k
                bq = q
        sg = seg[bk]
        tgt = bq
        nfb = int(0)
        if use_com == 1:
            tgt = com[sg]
            if cv == 1:
                tgt = tgt + vel[sg] * tt
        else:
            # nearest-point u; fall back to the segment COM when the sphere centre is within 1 mm of the shape axis
            if wp.length(bq - c) < 1.0e-3:
                tgt = com[sg]
                if cv == 1:
                    tgt = tgt + vel[sg] * tt
                nfb = nfb + 1
        u = tgt - c
        u = u / (wp.length(u) + 1.0e-9)
        ah = wp.dot(u, inv_h[sg] * u)
        ar = float(0.0)
        if mass_on[s] == 1:
            ar = wp.dot(u, inv_r[s] * u)
        o = (b * H + h) * n_s + s
        o_dist[o] = best
        o_shape[o] = bk
        o_seg[o] = sg
        o_u[o] = u
        o_mh[o] = 1.0 / ah
        o_mr[o] = 1.0 / ar
        o_mu[o] = 1.0 / (ah + ar)
        # ---- (1-3) 조합 1 비용: 구별 비용 = w · max_k c(d_k, η_k) (agg 0 = max_all) 또는 c(d_k*, η_k*) (agg 1 = nearest)
        md = mode[0]
        cbest = float(0.0)
        ebest = float(0.0)
        kbest = int(-1)
        if md > 0:
            w = par[0]
            eta0 = par[1]
            muref = par[2]
            pw = par[3]
            emin = par[4]
            emax = par[5]
            ecap = emax
            if md == 1:
                ecap = eta0
            k0 = int(0)
            k1 = n_shapes
            if agg[0] == 1:
                k0 = bk
                k1 = bk + 1
            for kk in range(k0, k1):
                a2 = p0[kk]
                sk2 = seg[kk]
                if cv == 1:
                    a2 = a2 + vel[sk2] * tt
                d2 = dvec[kk]
                t2 = wp.clamp(wp.dot(c - a2, d2) * invl2[kk], 0.0, 1.0)
                q2 = a2 + d2 * t2
                dk = wp.length(q2 - c) - rad[kk] - x[3]
                if dk < ecap:
                    eta = eta0
                    if md >= 2:
                        tg = q2
                        if use_com == 1:
                            tg = com[sk2]
                            if cv == 1:
                                tg = tg + vel[sk2] * tt
                        else:
                            if wp.length(q2 - c) < 1.0e-3:
                                tg = com[sk2]
                                if cv == 1:
                                    tg = tg + vel[sk2] * tt
                                nfb = nfb + 1
                        uk = tg - c
                        uk = uk / (wp.length(uk) + 1.0e-9)
                        ahk = wp.dot(uk, inv_h[sk2] * uk)
                        if md == 3:
                            ahk = inv_iso[sk2]      # (B1) iso: 1/m_h_iso(segment), direction independent
                        ark = float(0.0)
                        if mass_on[s] == 1:
                            ark = wp.dot(uk, inv_r[s] * uk)
                        muk = 1.0 / (ahk + ark)
                        if md == 5:
                            # (part) gain with a per-segment reference: mu_iso = 1 / (1/m_iso(segment) + 1/m_r), same robot term as mu
                            eta = wp.clamp(eta0 + pw * wp.max(wp.sqrt(muk * (inv_iso[sk2] + ark)) - 1.0, 0.0), emin, emax)
                        elif md == 4:
                            # (V1) gain: eta = clamp(d_base + k_gain * max(sqrt(mu / mu_ref) - 1, 0), d_base, eta_max); par: eta0 = d_base, p = k_gain, eta_min = d_base
                            eta = wp.clamp(eta0 + pw * wp.max(wp.sqrt(muk / muref) - 1.0, 0.0), emin, emax)
                        else:
                            eta = wp.clamp(eta0 * wp.pow(muk / muref, pw), emin, emax)
                    ck = float(0.0)
                    if dk < 0.0:
                        ck = 0.5 * eta - dk
                    elif dk < eta:
                        ck = (eta - dk) * (eta - dk) * (0.5 / eta)
                    if ck > cbest:
                        cbest = ck
                        ebest = eta
                        kbest = kk
            cbest = w * cbest
        o_cost[o] = cbest
        o_eta[o] = ebest
        o_kc[o] = kbest
        o_fb[o] = nfb

    _WP["wp"], _WP["k"] = wp, meff_kernel
    return wp, meff_kernel


class MeffChannel:
    def __init__(self, mpc, inputs, impl="warp", direction="com", motion="static", hfront=0, sphere_idx=None, mass_on=None, record_q=True, speed_cap=False):
        assert impl in IMPLS and direction in ("com", "closest") and motion in ("static", "cv")
        self.inp, self.impl, self.dir, self.cv, self.record_q = inputs, impl, direction, motion == "cv", record_q
        # (T1) speed cap cost: built into the captured graph only when speed_cap=True (False = no extra op, same graph as before)
        self.speed_cap = bool(speed_cap)
        self.sc_par = torch.zeros(3, dtype=torch.float32, device=inputs.device)          # [v_cap m/s, dist m, w]
        self._sc_c = torch.zeros(3, dtype=torch.float32).pin_memory()
        self.inv_dt = (1.0 / (inputs.tau[1:] - inputs.tau[:-1])).contiguous()             # (H-1,) 1 / traj_dt_h for h >= 1
        import os as _os
        self.noop = _os.environ.get("G1_MEFF_NOOP", "0") == "1"      # 시간 분해 시험용: warp 커널을 띄우되 바로 반환
        self.S = inputs.S
        dev = inputs.device
        self.sphere_idx = sphere_idx
        mo = np.ones(self.S, np.int32) if mass_on is None else np.asarray(mass_on, np.int32)
        if impl == "pairs":
            mo[:] = 1                                   # 기본 구현: 모든 구에서 로봇 항 계산 (역관성 0 이면 a_r = 0 으로 나옴)
        self.mass_on = torch.as_tensor(mo, device=dev)
        self.mass_idx = torch.as_tensor(np.nonzero(mo)[0], device=dev, dtype=torch.long)
        # (1-3) 비용 파라미터 (GPU 텐서, 값만 copy_ → 재캡처 없음): par = [w, η0, μref, p, η_min, η_max], mode 0 off / 1 uniform / 2 mass, agg 0 max_all / 1 nearest
        self.par = torch.zeros(6, dtype=torch.float32, device=dev); self.mode = torch.zeros(1, dtype=torch.int32, device=dev); self.agg = torch.zeros(1, dtype=torch.int32, device=dev)
        self.inv_iso = torch.as_tensor(1.0 / iso_mass_19(), dtype=torch.float32, device=dev)   # (B1) mode 3
        self._par_c = torch.zeros(6, dtype=torch.float32).pin_memory(); self._mode_c = torch.zeros(1, dtype=torch.int32).pin_memory(); self._agg_c = torch.zeros(1, dtype=torch.int32).pin_memory()
        self.optimizers, self.info = [], []
        for i, opt in enumerate(mpc.solver.optimizers):
            assert not opt.cu_opt_init, "hook must precede optimizer capture"
            B, H = int(opt.total_num_particles), int(opt.horizon)
            Hf = H if not hfront else int(hfront)
            z = lambda *s, dt=torch.float32: torch.zeros(s, dtype=dt, device=dev)
            d = dict(dist=z(B, H, self.S), shape=z(B, H, self.S, dt=torch.int32), seg=z(B, H, self.S, dt=torch.int32), u=z(B, H, self.S, 3),
                     mh=z(B, H, self.S), mr=z(B, H, self.S), mu=z(B, H, self.S), calls=torch.zeros((), dtype=torch.int64, device=dev),
                     cost=z(B, H, self.S), eta=z(B, H, self.S), kc=z(B, H, self.S, dt=torch.int32), cost_bh=z(B, H), fb=z(B, H, self.S, dt=torch.int32), vcost=z(B, H))
            if record_q:
                d["q"] = z(B, H, 7); d["sph"] = z(B, H, self.S, 4)
            ctx = dict(B=B, H=H, Hf=Hf, h_idx=torch.arange(Hf, device=dev), ar_buf=z(B, Hf, self.S, N_SHAPES), ar_buf1=z(B, Hf, self.S))
            original = opt.rollout_fn.cost_fn

            def hooked(rollout, state, action_batch=None, _old=original, _d=d, _c=ctx):
                base = _old(state, action_batch)
                self.compute(state.robot_spheres, _d, _c)
                _d["sph_ref"] = state.robot_spheres          # (B2a-2) python reference only (no GPU op); read after the step for the plate rollout count
                torch.sum(_d["cost"], dim=-1, out=_d["cost_bh"])     # (1-3) 구별 비용 합 (B,H). off 면 0
                if self.record_q:
                    _d["q"].copy_(state.state_seq.position)
                    _d["sph"].copy_(state.robot_spheres[:, :, : self.S])
                _d["calls"].add_(1)
                if self.speed_cap:                       # (T1) speed cap: w * max over spheres within dist of the human of max(0, v - v_cap)^2, per (b, h >= 1)
                    self.speed_cost(state.robot_spheres, _d, _c)
                    return base + _d["cost_bh"] + _d["vcost"]
                return base + _d["cost_bh"]              # (1-3) off 면 + 0 (1-2d 와 같은 값)

            opt.rollout_fn.cost_fn = types.MethodType(hooked, opt.rollout_fn)
            self.optimizers.append((opt, d, ctx, original))
            self.info.append(dict(index=i, optimizer_type=type(opt).__name__, not_aux_rollout=opt.rollout_fn is not mpc.rollout_fn, pre_capture=not opt.cu_opt_init,
                                  shape=[B, H, self.S], hfront=Hf))
        assert self.optimizers and all(x["optimizer_type"] == "ParallelMPPI" and x["not_aux_rollout"] for x in self.info)
        if impl == "warp":
            wp, _ = _warp()
            wp.init()
        self.addr0 = self.pointers()

    def speed_cost(self, sph_all, d, c):
        """(T1) v = |c_h - c_(h-1)| / traj_dt_h per sphere (h >= 1); penalty only where the sphere-human shape gap (kernel dist, MPC radius) <= dist"""
        assert c["Hf"] == c["H"], "speed cap needs the human distance on the whole horizon (hfront 0)"
        x = sph_all[:, :, : self.S, :3]
        v = torch.linalg.vector_norm(x[:, 1:] - x[:, :-1], dim=-1) * self.inv_dt.view(1, -1, 1)
        ex = torch.clamp(v - self.sc_par[0], min=0.0)
        pen = torch.where(d["dist"][:, 1:] <= self.sc_par[1], ex * ex, torch.zeros_like(ex)).amax(-1)
        d["vcost"][:, 1:].copy_(pen * self.sc_par[2])

    def set_speed_cap(self, v=0.5, dist=0.4, w=0.0):
        self._sc_c.copy_(torch.tensor([v, dist, w], dtype=torch.float32)); self.sc_par.copy_(self._sc_c, non_blocking=True)

    def set_cost(self, mode=None, agg=None, w=None, eta0=None, muref=None, p=None, eta_min=None, eta_max=None):
        """비용 파라미터 갱신: pinned → 고정 GPU 텐서 copy_ (주소 불변, 재캡처 없음)"""
        modes = {"off": 0, "uniform": 1, "mass": 2, "iso": 3, "gain": 4, "gain_part": 5}; aggs = {"max_all": 0, "nearest": 1}
        cur = self._par_c.numpy()
        for i, v in enumerate((w, eta0, muref, p, eta_min, eta_max)):
            if v is not None:
                cur[i] = v
        self.par.copy_(self._par_c, non_blocking=True)
        if mode is not None:
            self._mode_c[0] = modes[mode]; self.mode.copy_(self._mode_c, non_blocking=True)
        if agg is not None:
            self._agg_c[0] = aggs[agg]; self.agg.copy_(self._agg_c, non_blocking=True)

    def cost_state(self):
        torch.cuda.synchronize()
        inv = {0: "off", 1: "uniform", 2: "mass", 3: "iso", 4: "gain", 5: "gain_part"}
        return dict(mode=inv[int(self.mode[0])], agg={0: "max_all", 1: "nearest"}[int(self.agg[0])],
                    **dict(zip(["w", "eta0", "muref", "p", "eta_min", "eta_max"], [float(x) for x in self.par.cpu()])))

    def pointers(self):
        p = list(self.inp.pointers()) + [self.par.data_ptr(), self.mode.data_ptr(), self.agg.data_ptr(), self.inv_iso.data_ptr()]
        for opt, d, c, o in self.optimizers:
            p += [v.data_ptr() for v in d.values()] + [c["ar_buf"].data_ptr()]
        return p

    # -------- 계산 (캡처 안에서 호출됨) --------
    def compute(self, sph_all, d, c):
        if self.impl == "warp":
            return self._compute_warp(sph_all, d, c)
        I, S, Hf = self.inp, self.S, c["Hf"]
        x = sph_all[:, :Hf, :S, :]
        cpos = x[..., :3]; rs = x[..., 3]
        if self.cv:
            tt = I.age + I.tau[:Hf]                                           # (Hf,)
            off = I.vel[I.seg].unsqueeze(0) * tt.view(Hf, 1, 1)               # (Hf,NS,3)
            P0 = (I.p0.unsqueeze(0) + off).unsqueeze(0).unsqueeze(2)          # (1,Hf,1,NS,3)
            D = (I.p1 - I.p0).view(1, 1, 1, N_SHAPES, 3)
            COMh = I.com.unsqueeze(0) + I.vel.unsqueeze(0) * tt.view(Hf, 1, 1)   # (Hf,19,3)
            COMk = COMh[:, I.seg].unsqueeze(0).unsqueeze(2)                   # (1,Hf,1,NS,3)
        else:
            P0 = I.p0.view(1, 1, 1, N_SHAPES, 3); D = (I.p1 - I.p0).view(1, 1, 1, N_SHAPES, 3)
            COMh = None; COMk = I.com[I.seg].view(1, 1, 1, N_SHAPES, 3)
        L2 = (D * D).sum(-1).clamp_min(1e-12)
        cc = cpos.unsqueeze(-2)                                                # (B,Hf,S,1,3)
        t = (((cc - P0) * D).sum(-1) / L2).clamp(0.0, 1.0)
        Q = P0 + t.unsqueeze(-1) * D                                           # (B,Hf,S,NS,3)
        V = Q - cc
        sd = V.norm(dim=-1) - I.rad - rs.unsqueeze(-1)
        k = sd.argmin(-1)                                                      # (B,Hf,S)
        dist = sd.gather(-1, k.unsqueeze(-1)).squeeze(-1)
        seg = I.seg[k]
        if self.impl in ("pairs", "skip0"):
            U = (COMk - cc) if self.dir == "com" else V
            U = U / (U.norm(dim=-1, keepdim=True) + 1e-9)
            ah_all = torch.einsum("bhsni,nij,bhsnj->bhsn", U, I.inv_h[I.seg], U)
            if self.impl == "pairs":
                ar_all = torch.einsum("bhsni,sij,bhsnj->bhsn", U, I.inv_r, U)
            else:
                Um = U.index_select(2, self.mass_idx)
                c["ar_buf"].index_copy_(2, self.mass_idx, torch.einsum("bhsni,sij,bhsnj->bhsn", Um, I.inv_r.index_select(0, self.mass_idx), Um))
                ar_all = c["ar_buf"]
            kk = k.unsqueeze(-1)
            u = U.gather(-2, kk.unsqueeze(-1).expand(*k.shape, 1, 3)).squeeze(-2)
            ah = ah_all.gather(-1, kk).squeeze(-1); ar = ar_all.gather(-1, kk).squeeze(-1)
        else:   # nearest
            if self.dir == "com":
                tgt = I.com[seg] if COMh is None else COMh[c["h_idx"].view(1, Hf, 1), seg]
            else:
                tgt = Q.gather(-2, k.unsqueeze(-1).unsqueeze(-1).expand(*k.shape, 1, 3)).squeeze(-2)
            u = tgt - cpos
            u = u / (u.norm(dim=-1, keepdim=True) + 1e-9)
            ah = torch.einsum("bhsi,bhsij,bhsj->bhs", u, I.inv_h[seg], u)
            um = u.index_select(2, self.mass_idx)
            c["ar_buf1"].index_copy_(2, self.mass_idx, torch.einsum("bhsi,sij,bhsj->bhs", um, I.inv_r.index_select(0, self.mass_idx), um))
            ar = c["ar_buf1"]
        d["dist"][:, :Hf].copy_(dist); d["shape"][:, :Hf].copy_(k); d["seg"][:, :Hf].copy_(seg); d["u"][:, :Hf].copy_(u)
        d["mh"][:, :Hf].copy_(1.0 / ah); d["mr"][:, :Hf].copy_(1.0 / ar); d["mu"][:, :Hf].copy_(1.0 / (ah + ar))

    def _compute_warp(self, sph_all, d, c):
        wp, kern = _warp()
        assert sph_all.is_contiguous()
        I = self.inp; B, H, Hf = c["B"], c["H"], c["Hf"]
        n_sall = sph_all.shape[2]
        wp.launch(kernel=kern, dim=B * Hf * self.S,
                  inputs=[wp.from_torch(sph_all.detach().reshape(-1, 4), dtype=wp.vec4), n_sall, self.S, H, Hf,
                          wp.from_torch(I.p0, dtype=wp.vec3), wp.from_torch(I.dvec, dtype=wp.vec3), wp.from_torch(I.invl2), wp.from_torch(I.rad), wp.from_torch(I.seg32, dtype=wp.int32),
                          wp.from_torch(I.com, dtype=wp.vec3), wp.from_torch(I.vel, dtype=wp.vec3), wp.from_torch(I.inv_h, dtype=wp.mat33),
                          wp.from_torch(I.inv_r, dtype=wp.mat33), wp.from_torch(self.mass_on, dtype=wp.int32), wp.from_torch(I.tau), wp.from_torch(I.age),
                          int(self.cv), int(self.dir == "com"), N_SHAPES, int(self.noop),
                          wp.from_torch(d["dist"].view(-1)), wp.from_torch(d["shape"].view(-1), dtype=wp.int32), wp.from_torch(d["seg"].view(-1), dtype=wp.int32),
                          wp.from_torch(d["u"].view(-1, 3), dtype=wp.vec3), wp.from_torch(d["mh"].view(-1)), wp.from_torch(d["mr"].view(-1)),
                          wp.from_torch(d["mu"].view(-1)),
                          wp.from_torch(self.par), wp.from_torch(self.mode, dtype=wp.int32), wp.from_torch(self.agg, dtype=wp.int32), wp.from_torch(self.inv_iso),
                          wp.from_torch(d["cost"].view(-1)), wp.from_torch(d["eta"].view(-1)), wp.from_torch(d["kc"].view(-1), dtype=wp.int32), wp.from_torch(d["fb"].view(-1), dtype=wp.int32)],
                  stream=wp.stream_from_torch(sph_all.device))

    # -------- 캡처 밖: 현재 상태 하나 (표시용) --------
    @torch.no_grad()
    def query(self, sph):
        """sph (N,4) 현재 로봇 구 → dict(dist, shape, seg, closest (N,3), u, mh, mr, mu). 정지 위치 기준, 이 채널과 같은 정의"""
        I = self.inp; c = sph[:, None, :3]; rs = sph[:, 3]
        P0 = I.p0[None]; D = (I.p1 - I.p0)[None]; L2 = (D * D).sum(-1).clamp_min(1e-12)
        t = (((c - P0) * D).sum(-1) / L2).clamp(0, 1); Q = P0 + t[..., None] * D
        sd = (Q - c).norm(dim=-1) - I.rad - rs[:, None]; k = sd.argmin(-1); ar = torch.arange(len(k), device=k.device)
        q = Q[ar, k]; seg = I.seg[k]
        fb = (q - sph[:, :3]).norm(dim=-1) < 1e-3                       # nearest-point u falls back to the COM within 1 mm of the axis
        u = (I.com[seg] if self.dir == "com" else torch.where(fb[:, None], I.com[seg], q)) - sph[:, :3]; u = u / (u.norm(dim=-1, keepdim=True) + 1e-9)
        ah = torch.einsum("ni,nij,nj->n", u, I.inv_h[seg], u); a_r = torch.einsum("ni,nij,nj->n", u, I.inv_r, u) * self.mass_on.float()
        return dict(dist=sd[ar, k], shape=k, seg=seg, closest=q, u=u, mh=1 / ah, mr=1 / a_r, mu=1 / (ah + a_r), fb=(fb & (self.dir != "com")).sum())

    @torch.no_grad()
    def query_all(self, sph):
        """sph (N,4) 현재 로봇 구 → 모든 (구, 형상) 쌍의 거리·분절·m_h·m_r·μ (정지 위치, u 는 이 채널 방향 정의). (N,24) 텐서들"""
        I = self.inp; c = sph[:, None, :3]; rs = sph[:, 3]
        P0 = I.p0[None]; D = (I.p1 - I.p0)[None]; L2 = (D * D).sum(-1).clamp_min(1e-12)
        t = (((c - P0) * D).sum(-1) / L2).clamp(0, 1); Q = P0 + t[..., None] * D
        sd = (Q - c).norm(dim=-1) - I.rad - rs[:, None]
        COMk = I.com[I.seg][None].expand_as(Q); fb = (Q - c).norm(dim=-1) < 1e-3
        tgt = COMk if self.dir == "com" else torch.where(fb[..., None], COMk, Q)
        u = tgt - c; u = u / (u.norm(dim=-1, keepdim=True) + 1e-9)
        ah = torch.einsum("nki,kij,nkj->nk", u, I.inv_h[I.seg], u); a_r = torch.einsum("nki,nij,nkj->nk", u, I.inv_r, u) * self.mass_on.float()[:, None]
        return dict(dist=sd, seg=I.seg[None].expand_as(sd), mh=1 / ah, mr=1 / a_r, mu=1 / (ah + a_r), u=u, fb=(fb & (self.dir != "com")).sum())

    def snapshot(self, opt_index=0):
        torch.cuda.synchronize()
        opt, d, c, o = self.optimizers[opt_index]
        return {k: v.detach().cpu().numpy().copy() for k, v in d.items()}
