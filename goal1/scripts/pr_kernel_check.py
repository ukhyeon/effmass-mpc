"""pr_kernel_check.py - (part) check of the warp cost kernel for gain (mode 4, global mu_ref) and gain_part (mode 5, per-part mu_iso).

Run with the Isaac python (warp + torch): isaacsim/python.sh goal1/scripts/pr_kernel_check.py
  1) random spheres / shapes / inverse inertias / robot terms: kernel o_cost, o_eta vs a float64 CPU reference of the same rule
  2) m_h = m_iso (isotropic inv_h = 1/m_iso per segment), random m_r: mode 5 eta must equal d_base wherever a shape is in range
Writes goal1/part_regen/kernel_check.json.
"""
import json
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g1_meff_gpu as G

OUT = os.path.expanduser("~/effmass_mpc/goal1/part_regen"); os.makedirs(OUT, exist_ok=True)
rng = np.random.default_rng(0)
B, H, S, K, NSEG = 6, 5, 20, 24, 19
dev = "cuda:0"
wp, kern = G._warp()
wp.init()
inv_iso = (1.0 / G.iso_mass_19()).astype(np.float32)


def spd(n, lo, hi):
    A = rng.normal(size=(n, 3, 3)); M = A @ A.transpose(0, 2, 1); w, V = np.linalg.eigh(M)
    w = rng.uniform(lo, hi, size=(n, 3))
    return np.einsum("nij,nj,nkj->nik", V, w, V).astype(np.float32)


def run(mode, inv_h, inv_r, mass_on, par):
    sph = np.zeros((B, H, S, 4), np.float32)
    sph[..., :3] = rng.uniform(-0.3, 0.3, size=(B, H, S, 3)); sph[..., 3] = rng.uniform(0.02, 0.08, size=(B, H, S))
    p0 = rng.uniform(-0.3, 0.3, size=(K, 3)).astype(np.float32); p1 = p0 + rng.uniform(-0.15, 0.15, size=(K, 3)).astype(np.float32)
    rad = rng.uniform(0.03, 0.08, size=K).astype(np.float32); seg = rng.integers(0, NSEG, size=K).astype(np.int32)
    dv = (p1 - p0).astype(np.float32); invl2 = (1.0 / np.maximum((dv * dv).sum(1), 1e-12)).astype(np.float32)
    com = rng.uniform(-0.3, 0.3, size=(NSEG, 3)).astype(np.float32)
    T = lambda a, dt=torch.float32: torch.as_tensor(a, dtype=dt, device=dev).contiguous()
    o = {k: torch.zeros(B * H * S, dtype=torch.float32, device=dev) for k in ("dist", "mh", "mr", "mu", "cost", "eta")}
    oi = {k: torch.zeros(B * H * S, dtype=torch.int32, device=dev) for k in ("shape", "seg", "kc", "fb")}
    ou = torch.zeros(B * H * S, 3, dtype=torch.float32, device=dev)
    t_sph = T(sph.reshape(-1, 4)); t_par = T(np.asarray(par, np.float32)); t_mode = T([mode], torch.int32); t_agg = T([0], torch.int32)
    args = [wp.from_torch(t_sph, dtype=wp.vec4), S, S, H, H, wp.from_torch(T(p0), dtype=wp.vec3), wp.from_torch(T(dv), dtype=wp.vec3), wp.from_torch(T(invl2)),
            wp.from_torch(T(rad)), wp.from_torch(T(seg, torch.int32), dtype=wp.int32), wp.from_torch(T(com), dtype=wp.vec3),
            wp.from_torch(T(np.zeros((NSEG, 3))), dtype=wp.vec3), wp.from_torch(T(inv_h), dtype=wp.mat33), wp.from_torch(T(inv_r), dtype=wp.mat33),
            wp.from_torch(T(mass_on, torch.int32), dtype=wp.int32), wp.from_torch(T(np.zeros(H))), wp.from_torch(T([0.0])), 0, 0, K, 0,
            wp.from_torch(o["dist"]), wp.from_torch(oi["shape"], dtype=wp.int32), wp.from_torch(oi["seg"], dtype=wp.int32), wp.from_torch(ou, dtype=wp.vec3),
            wp.from_torch(o["mh"]), wp.from_torch(o["mr"]), wp.from_torch(o["mu"]), wp.from_torch(t_par), wp.from_torch(t_mode, dtype=wp.int32),
            wp.from_torch(t_agg, dtype=wp.int32), wp.from_torch(T(inv_iso)), wp.from_torch(o["cost"]), wp.from_torch(o["eta"]),
            wp.from_torch(oi["kc"], dtype=wp.int32), wp.from_torch(oi["fb"], dtype=wp.int32)]
    wp.launch(kern, dim=B * H * S, inputs=args); torch.cuda.synchronize()
    # CPU reference (float64), same rule
    w, eta0, muref, pw, emin, emax = [float(v) for v in par]
    ref_c = np.zeros(B * H * S); ref_e = np.zeros(B * H * S); ref_k = np.full(B * H * S, -1)
    X = sph.reshape(-1, 4).astype(np.float64)
    for i in range(B * H * S):
        s = i % S; c = X[i, :3]; r = X[i, 3]; cb = 0.0; eb = 0.0; kb = -1
        for k in range(K):
            a = p0[k].astype(np.float64); d = dv[k].astype(np.float64)
            tt = np.clip(np.dot(c - a, d) * float(invl2[k]), 0, 1); q = a + d * tt
            dk = np.linalg.norm(q - c) - rad[k] - r
            if dk >= emax:
                continue
            tg = q if np.linalg.norm(q - c) >= 1e-3 else com[seg[k]].astype(np.float64)
            u = (tg - c) / (np.linalg.norm(tg - c) + 1e-9)
            ah = u @ inv_h[seg[k]].astype(np.float64) @ u; ar = u @ inv_r[s].astype(np.float64) @ u if mass_on[s] else 0.0
            mu = 1 / (ah + ar)
            if mode == 5:
                eta = np.clip(eta0 + pw * max(np.sqrt(mu * (inv_iso[seg[k]] + ar)) - 1, 0), emin, emax)
            else:
                eta = np.clip(eta0 + pw * max(np.sqrt(mu / muref) - 1, 0), emin, emax)
            ck = 0.5 * eta - dk if dk < 0 else ((eta - dk) ** 2 * 0.5 / eta if dk < eta else 0.0)
            if ck > cb:
                cb, eb, kb = ck, eta, k
        ref_c[i], ref_e[i], ref_k[i] = w * cb, eb, kb
    gc, ge, gk = o["cost"].cpu().numpy(), o["eta"].cpu().numpy(), oi["kc"].cpu().numpy()
    m = ref_k >= 0
    return dict(pairs_in_range=int(m.sum()), same_shape=float((gk[m] == ref_k[m]).mean()),
                cost_rel_err_max=float(np.max(np.abs(gc[m] - ref_c[m]) / np.maximum(np.abs(ref_c[m]), 1e-9))),
                eta_abs_err_max=float(np.max(np.abs(ge[m] - ref_e[m]))), eta_min=float(ge[m].min()), eta_max=float(ge[m].max()),
                eta_above_base_frac=float((ge[m] > par[1] + 1e-7).mean())), ge[m]


par = [1e5, 0.065, 0.53, 0.10, 0.065, 0.25]
res = {}
inv_h = spd(NSEG, 1 / 8.0, 1 / 0.2); inv_r = spd(S, 1 / 10.0, 1 / 0.5); mass_on = (rng.random(S) > 0.2).astype(np.int32)
for mode in (4, 5):
    r, _ = run(mode, inv_h, inv_r, mass_on, par); res[f"random_mode{mode}"] = r
iso_h = np.stack([np.eye(3, dtype=np.float32) * inv_iso[g] for g in range(NSEG)])
r, ge = run(5, iso_h, spd(S, 1 / 50.0, 1 / 0.3), mass_on, par)
r["all_eta_equal_d_base"] = bool(np.all(np.abs(ge - 0.065) < 1e-6)); res["m_h_equals_m_iso_mode5"] = r
json.dump(res, open(f"{OUT}/kernel_check.json", "w"), indent=1)
print(json.dumps(res, indent=1))
