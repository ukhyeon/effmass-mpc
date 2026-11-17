"""g1_meff_micro.py — 채널 계산만 따로 시간 (MPC 없이, 고정 입력, CUDA graph 캡처 후 재생). Isaac Python."""
import os, sys, json, time, types
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/goal1/scripts"))
import numpy as np, torch
import g1_meff_gpu as G
dev = "cuda:0"; B, H, SA, S = 400, 30, 65, 61
seq = np.load(os.path.expanduser("~/effmass_mpc/goal1/logs/g1_human_seq.npz"))
tf = json.load(open(os.path.expanduser("~/effmass_mpc/env_check/step4/temp_transform.json")))
inp = G.MeffInputs(S, [1 / 60] * H, dev)
inp.set_human(G.to_isaac(dict(com=seq["com"][300], vel=seq["vel"][300], inv=seq["inv"][300], p0=seq["p0"][300], p1=seq["p1"][300], rad=seq["rad"], seg=seq["seg"]), tf["R"], tf["t"]))
inp.set_robot(np.tile(np.eye(3) * 0.2, (S, 1, 1)))
g = torch.Generator(device=dev).manual_seed(0)
sph = torch.rand((B, H, SA, 4), device=dev, generator=g) * torch.tensor([1.0, 1.0, 1.0, 0.1], device=dev) - torch.tensor([0.2, 0.5, 0.0, 0.0], device=dev)
class FakeMpc: pass
res = {}
for impl in sys.argv[1:]:
    ch = G.MeffChannel.__new__(G.MeffChannel)
    ch.inp, ch.impl, ch.dir, ch.cv, ch.record_q, ch.S, ch.noop = inp, impl, "com", False, False, S, False
    mo = np.ones(S, np.int32); mo[[0, 1, 4, 5]] = 0
    if impl == "pairs": mo[:] = 1
    ch.mass_on = torch.as_tensor(mo, device=dev); ch.mass_idx = torch.as_tensor(np.nonzero(mo)[0], device=dev)
    z = lambda *s, dt=torch.float32: torch.zeros(s, dtype=dt, device=dev)
    d = dict(dist=z(B, H, S), shape=z(B, H, S, dt=torch.int32), seg=z(B, H, S, dt=torch.int32), u=z(B, H, S, 3), mh=z(B, H, S), mr=z(B, H, S), mu=z(B, H, S))
    c = dict(B=B, H=H, Hf=H, h_idx=torch.arange(H, device=dev), ar_buf=z(B, H, S, G.N_SHAPES), ar_buf1=z(B, H, S))
    if impl == "warp":
        G._warp()[0].init()
    st = torch.cuda.Stream(); st.wait_stream(torch.cuda.current_stream())
    with torch.cuda.stream(st):
        for _ in range(3): ch.compute(sph, d, c)
    torch.cuda.current_stream().wait_stream(st); torch.cuda.synchronize()
    gr = torch.cuda.CUDAGraph()
    with torch.cuda.graph(gr, stream=st):
        ch.compute(sph, d, c)
    torch.cuda.synchronize()
    e0, e1 = torch.cuda.Event(True), torch.cuda.Event(True); ts = []
    for _ in range(50):
        e0.record(); gr.replay(); e1.record(); torch.cuda.synchronize(); ts.append(e0.elapsed_time(e1))
    res[impl] = round(float(np.median(ts)), 4)
    print(f"[micro] {impl}: graph replay per call median {res[impl]} ms", flush=True)
