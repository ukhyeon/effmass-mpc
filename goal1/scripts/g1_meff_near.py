"""g1_meff_near.py — 가까운·겹친 경우 검증용 합성 입력 (Isaac Python). 실제 사람 프레임 형상 근처(표면 −0.10 ~ +0.15 m)에 로봇 구를 무작위로 두고
warp 채널 계산(eager, MPC 없음) 결과를 bench 덤프와 같은 형식으로 저장 → g1_meff_cpu_check.py 로 비교 (q 없으면 덤프의 구·Λ_r 사용)"""
import json, os, sys
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/goal1/scripts"))
import numpy as np, torch
import g1_meff_gpu as G
H_ = os.path.expanduser("~"); dev = "cuda:0"; rng = np.random.default_rng(int(sys.argv[2]) if len(sys.argv) > 2 else 0)
direction, motion = sys.argv[1].split("_")
CM = sys.argv[3] if len(sys.argv) > 3 else "off"; AG = sys.argv[4] if len(sys.argv) > 4 else "max_all"
B, H, SA, S = 50, 30, 65, 61
seq = np.load(f"{H_}/effmass_mpc/goal1/logs/g1_human_seq.npz"); tf = json.load(open(f"{H_}/effmass_mpc/env_check/step4/temp_transform.json"))
traj_dt = [1 / 60] * 16 + [0.016667 + 0.0016667 * k for k in range(1, 15)]
inp = G.MeffInputs(S, traj_dt, dev)
for rep in range(4):
    f = int(rng.integers(0, len(seq["com"])))
    hm = G.to_isaac(dict(com=seq["com"][f], vel=seq["vel"][f], inv=seq["inv"][f], p0=seq["p0"][f], p1=seq["p1"][f], rad=seq["rad"], seg=seq["seg"]), tf["R"], tf["t"])
    inp.set_human(hm); inp.set_age(0.03)
    A = rng.normal(size=(S, 3, 3)) * 0.3; inv_r = A @ A.transpose(0, 2, 1) + 0.01 * np.eye(3); inv_r[[0, 1, 4, 5]] = 0; inp.set_robot(inv_r)
    mo = np.ones(S, np.int32); mo[[0, 1, 4, 5]] = 0
    sph = np.zeros((B, H, SA, 4), np.float32)
    k = rng.integers(0, G.N_SHAPES, (B, H, S)); tt = rng.uniform(size=(B, H, S, 1))
    axis = hm["p0"][k] + tt * (hm["p1"][k] - hm["p0"][k]); dv = rng.normal(size=(B, H, S, 3)); dv /= np.linalg.norm(dv, axis=-1, keepdims=True)
    rr = rng.uniform(0.03, 0.10, (B, H, S)); off = hm["rad"][k] + rr + rng.uniform(-0.10, 0.15, (B, H, S))
    sph[:, :, :S, :3] = axis + dv * off[..., None]; sph[:, :, :S, 3] = rr; sph[:, :, S:, 3] = -100.0
    if motion == "cv":   # 등속일 때도 같은 τ 로 앞당긴 위치 근처가 되게 (검증 범위만 넓힘)
        pass
    sph_t = torch.as_tensor(sph, device=dev)
    ch = G.MeffChannel.__new__(G.MeffChannel)
    ch.inp, ch.impl, ch.dir, ch.cv, ch.record_q, ch.S, ch.noop = inp, "warp", direction, motion == "cv", False, S, False
    ch.mass_on = torch.as_tensor(mo, device=dev)
    ch.par = torch.tensor([1.0e5, 0.10, 3.0, 0.5, 0.05, 0.20], device=dev); ch.inv_iso = torch.as_tensor(1.0 / G.iso_mass_19(), dtype=torch.float32, device=dev)
    ch.mode = torch.tensor([{"off": 0, "uniform": 1, "mass": 2, "iso": 3}[CM]], dtype=torch.int32, device=dev)
    ch.agg = torch.tensor([{"max_all": 0, "nearest": 1}[AG]], dtype=torch.int32, device=dev)
    z = lambda *s, dt=torch.float32: torch.zeros(s, dtype=dt, device=dev)
    d = dict(dist=z(B, H, S), shape=z(B, H, S, dt=torch.int32), seg=z(B, H, S, dt=torch.int32), u=z(B, H, S, 3), mh=z(B, H, S), mr=z(B, H, S), mu=z(B, H, S),
             cost=z(B, H, S), eta=z(B, H, S), kc=z(B, H, S, dt=torch.int32), fb=z(B, H, S, dt=torch.int32))
    G._warp()[0].init(); ch.compute(sph_t, d, dict(B=B, H=H, Hf=H)); torch.cuda.synchronize()
    sn = {kk: v.cpu().numpy() for kk, v in d.items()}
    bb = np.repeat(np.arange(B), H); hh = np.tile(np.arange(H), B)
    np.savez_compressed(f"{H_}/effmass_mpc/goal1/logs/meff_bench/near_{direction}_{motion}_{CM}_{AG}_dump_{rep}.npz", cost_par=ch.par.cpu().numpy(), cost_mode=int(ch.mode[0]), cost_agg=int(ch.agg[0]), b=bb, h=hh, frame=f, tau=inp.tau.cpu().numpy(), traj_dt=np.asarray(traj_dt),
                        age=0.03, inv_r=inv_r, mass_on=mo, sph=sph[bb, hh, :S], **{kk: sn[kk][bb, hh] for kk in ["dist", "shape", "seg", "u", "mh", "mr", "mu", "cost", "eta", "kc"]})
print("[near] saved", direction, motion)
