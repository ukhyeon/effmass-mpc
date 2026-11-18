"""lv_bridge_sim.py <out_json> <run_tag[:a:b]> [...] - overnight2 bundle 3(b): 60 Hz -> 1 kHz bridging, offline simulation.

Input: the executed 60 Hz joint targets q_cmd of replay runs (b1 log rows: step, q_cmd ..., Isaac physics dt 1/60 s).
Transport (per condition): packet k sent at t_k = k / 60 s arrives at t_k + delay + jitter_k (jitter_k ~ N(0, sd), clipped >= -delay + 0.5 ms),
dropped with probability p (independent, seed 0); a packet older than the newest received one is ignored.
Receiver at 1 kHz (t = n ms) builds the joint reference q_ref(t) with one of:
  hold     : newest received target
  linear   : straight line from the previous received target to the newest one over one nominal period (1/60 s) starting at its arrival
  lowpass  : first-order low-pass of hold, time constant 20 ms
  smooth3  : third-order critically damped filter of hold (three equal poles, time constant 8 ms): continuous acceleration, finite jerk
  smooth3_16: same with time constant 16 ms
  cubic    : cubic Hermite through received targets placed at their arrival time + one period (finite-difference velocities)
  spline3  : cubic Hermite with knots at the nominal times t_k + Db (the packet number gives t_k), backward-difference velocities,
             Db = delay + 2 sd + 1 period; dropped or late packets skipped (continuous velocity)
  spline5  : quintic Hermite, central-difference velocity and acceleration, Db = delay + 2 sd + 2 periods (continuous acceleration)
Measured on q_ref at 1 kHz (finite differences), interior of the window only (t > 0.3 s and before the last command's send time): max |velocity|, |acceleration|, |jerk| per joint relative to the FR3 limits
(goal1/overnight2/fr3_limits.json, official page), number of 1 kHz samples above each limit (any joint), and the added delay: the lag
(1 ms grid, joint with the largest motion, every 5 ms) that minimises the RMS between q_ref(t) and the commands linearly interpolated
in send time, minus the transport delay.
Torque rate (rough model): each joint follows q_ref with tau = K (q_ref - q) - D dq, I ddq = tau (decoupled, I assumed), K / D = libfranka
joint impedance example values as remembered (not verified), I = [0.5, 0.5, 0.3, 0.3, 0.05, 0.05, 0.02] kg m^2 (assumed);
max |d tau / dt| vs 1000 Nm/s and count of samples above it.
"""
import json
import os
import re
import sys

import numpy as np

GG = os.path.expanduser("~/effmass_mpc/goal1"); O = f"{GG}/logs/meff_isaac"
LIM = json.load(open(f"{GG}/overnight2/fr3_limits.json"))
DQ, DDQ, DDDQ, DTAU = (np.array(LIM[k], float) for k in ("dq_max", "ddq_max", "dddq_max", "dtau_max"))
KG = np.array([600, 600, 600, 600, 250, 150, 50.0]); DG = np.array([50, 50, 50, 50, 30, 25, 15.0]); IN = np.array([0.5, 0.5, 0.3, 0.3, 0.05, 0.05, 0.02])
P = 1 / 60.0
METHODS = ("hold", "linear", "lowpass", "smooth3", "smooth3_16", "cubic", "spline3", "spline5")


def load(tag, a=0, b=None):   # (window) steps a..b of the run
    z = np.load(f"{O}/{tag}_b1.npz"); r = z["rows"]; return r[a:b, 1:8].astype(float)


def transport(n, delay, sd, drop, rng):
    t = np.arange(n) * P; j = np.clip(rng.normal(0, sd, n), -delay + 0.0005, None) if sd > 0 else np.zeros(n)
    keep = rng.random(n) >= drop; keep[0] = True
    return t + delay + j, keep


def reference(Q, arr, keep, method):
    n = len(Q); T_end = arr[keep].max() + 0.1; tt = np.arange(0, T_end, 0.001); out = np.zeros((len(tt), 7))
    order = np.argsort(arr); rec = []   # (arrival, k) accepted in arrival order, newer k only
    newest = -1
    for k in order:
        if keep[k] and k > newest:
            rec.append((arr[k], k)); newest = k
    ra = np.array([a for a, _ in rec]); rk = np.array([k for _, k in rec]); m = 0; cur = Q[0].copy(); prev = Q[0].copy(); t_new = 0.0
    lp = Q[0].copy(); s1 = Q[0].copy(); s2 = Q[0].copy(); s3 = Q[0].copy()
    for i, t in enumerate(tt):
        while m < len(ra) and ra[m] <= t:
            prev = cur.copy() if method != "linear" else (prev_lin(prev, cur, t_new, ra[m]))
            cur = Q[rk[m]].copy(); t_new = ra[m]; m += 1
        if method == "hold":
            out[i] = cur
        elif method == "linear":
            a = min((t - t_new) / P, 1.0); out[i] = prev + a * (cur - prev)
        elif method == "lowpass":
            lp += (0.001 / 0.020) * (cur - lp); out[i] = lp
        elif method == "smooth3":
            c = 0.001 / 0.008; s1 += c * (cur - s1); s2 += c * (s1 - s2); s3 += c * (s2 - s3); out[i] = s3
    if method in ("spline3", "spline5"):   # knots at nominal times t_k + Db (packet number known), late (after use) or dropped packets skipped
        return spline(Q, arr, keep, method, tt)
    if method == "smooth3_16":
        out = np.zeros((len(tt), 7)); s1 = Q[0].copy(); s2 = Q[0].copy(); s3 = Q[0].copy(); m = 0; cur = Q[0].copy(); c = 0.001 / 0.016
        for i, t in enumerate(tt):
            while m < len(ra) and ra[m] <= t:
                cur = Q[rk[m]].copy(); m += 1
            s1 += c * (cur - s1); s2 += c * (s1 - s2); s3 += c * (s2 - s3); out[i] = s3
        return tt, out
    if method == "cubic":   # Hermite through the received targets, time axis = arrival time + one period (buffer)
        tk = ra + P; qk = Q[rk]; vk = np.gradient(qk, tk, axis=0) if len(tk) > 2 else np.zeros_like(qk)
        idx = np.clip(np.searchsorted(tk, tt) - 1, 0, len(tk) - 2); h = tk[idx + 1] - tk[idx]; s = np.clip((tt - tk[idx]) / np.maximum(h, 1e-6), 0, 1)[:, None]
        h00 = 2 * s ** 3 - 3 * s ** 2 + 1; h10 = s ** 3 - 2 * s ** 2 + s; h01 = -2 * s ** 3 + 3 * s ** 2; h11 = s ** 3 - s ** 2
        out = h00 * qk[idx] + h10 * h[:, None] * vk[idx] + h01 * qk[idx + 1] + h11 * h[:, None] * vk[idx + 1]; out[tt < tk[0]] = qk[0]
    return tt, out


SD = [0.0]; DL = [0.0]


def spline(Q, arr, keep, method, tt):
    """spline3: cubic Hermite, velocities by backward differences of the usable knots, buffer Db = delay + 2 sd + 1 period.
    spline5: quintic Hermite, velocity / acceleration by central differences (needs the next knot), Db = delay + 2 sd + 2 periods.
    A packet is usable if it is not dropped and arrives before its knot is first needed (t_k + Db - P for spline3, - 2P for spline5)."""
    n = len(Q); tk = np.arange(n) * P; extra = P if method == "spline3" else 2 * P; Db = DL[0] + 2 * SD[0] + extra
    # 1 us tolerance: float rounding must not turn an on-time packet into a late one
    use = keep & (arr <= tk + Db - extra + 1e-6); use[0] = True; k = np.nonzero(use)[0]; T = tk[k] + Db; X = Q[k]
    if method == "spline3":
        V = np.vstack([np.zeros((1, 7)), np.diff(X, axis=0) / np.diff(T)[:, None]])
        A = None
    else:
        V = np.gradient(X, T, axis=0); A = np.gradient(V, T, axis=0)
    i = np.clip(np.searchsorted(T, tt) - 1, 0, len(T) - 2); h = (T[i + 1] - T[i])[:, None]; s = np.clip((tt - T[i])[:, None] / h, 0, 1)
    if A is None:
        h00 = 2 * s ** 3 - 3 * s ** 2 + 1; h10 = s ** 3 - 2 * s ** 2 + s; h01 = -2 * s ** 3 + 3 * s ** 2; h11 = s ** 3 - s ** 2
        out = h00 * X[i] + h10 * h * V[i] + h01 * X[i + 1] + h11 * h * V[i + 1]
    else:
        H0 = 1 - 10 * s ** 3 + 15 * s ** 4 - 6 * s ** 5; H1 = s - 6 * s ** 3 + 8 * s ** 4 - 3 * s ** 5; H2 = 0.5 * s ** 2 - 1.5 * s ** 3 + 1.5 * s ** 4 - 0.5 * s ** 5
        G0 = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5; G1 = -4 * s ** 3 + 7 * s ** 4 - 3 * s ** 5; G2 = 0.5 * s ** 3 - s ** 4 + 0.5 * s ** 5
        out = H0 * X[i] + H1 * h * V[i] + H2 * h ** 2 * A[i] + G0 * X[i + 1] + G1 * h * V[i + 1] + G2 * h ** 2 * A[i + 1]
    out[tt < T[0]] = X[0]; out[tt > T[-1]] = X[-1]
    return tt, out


def prev_lin(prev, cur, t_new, t_now):   # linear: the new segment starts where the current one is now
    a = min((t_now - t_new) / P, 1.0); return prev + a * (cur - prev)


def follow(tt, R):
    q = R[0].copy(); dq = np.zeros(7); tau_prev = np.zeros(7); dtau = np.zeros(len(tt))
    for i in range(len(tt)):
        tau = KG * (R[i] - q) - DG * dq; ddq = tau / IN; dq += ddq * 0.001; q += dq * 0.001
        if i:
            dtau[i] = np.max(np.abs(tau - tau_prev) / 0.001 / DTAU)
        tau_prev = tau
    return dtau


def metrics(Q, tt, R, delay):
    ins = (tt > 0.3) & (tt < (len(Q) - 1) * P)   # interior: the window start (robot at rest, first packets) and end (last command) left out
    tt_all, R_all = tt, R; tt, R = tt[ins], R[ins]
    v = np.diff(R, axis=0) / 0.001; a = np.diff(v, axis=0) / 0.001; j = np.diff(a, axis=0) / 0.001
    ts = np.arange(len(Q)) * P; best = None; c = int(np.argmax(np.ptp(Q, 0))); sub = slice(None, None, 5)   # joint with the largest motion, every 5 ms
    for lag in np.arange(0, 0.08, 0.001):
        ref = np.interp(tt[sub] - lag, ts, Q[:, c]); msk = (tt[sub] - lag > 0.05) & (tt[sub] - lag < ts[-1] - 0.05)
        e = float(np.sqrt(np.mean((R[sub][msk, c] - ref[msk]) ** 2)))
        if best is None or e < best[0]:
            best = (e, lag)
    dtau = follow(tt_all, R_all)[ins]
    return dict(v_max_ratio=float(np.max(np.abs(v) / DQ)), a_max_ratio=float(np.max(np.abs(a) / DDQ)), j_max_ratio=float(np.max(np.abs(j) / DDDQ)),
                n_v_over=int(np.any(np.abs(v) > DQ, 1).sum()), n_a_over=int(np.any(np.abs(a) > DDQ, 1).sum()), n_j_over=int(np.any(np.abs(j) > DDDQ, 1).sum()),
                dtau_max_ratio=float(dtau.max()), n_dtau_over=int((dtau > 1).sum()), samples=len(tt), total_lag_ms=round(best[1] * 1000, 1),
                added_delay_ms=round((best[1] - delay) * 1000, 1), rms_at_lag_rad=best[0])


def main(out, tags):
    res = []
    for spec in tags:   # "tag" or "tag:a:b" (step window)
        f = spec.split(":"); tag = f[0]; Q = load(tag, int(f[1]) if len(f) > 1 else 0, int(f[2]) if len(f) > 2 else None)
        for delay in (0.005, 0.010, 0.020):
            for sd in (0.0, 0.005):
                for drop in (0.0, 0.05, 0.20):
                    rng = np.random.default_rng(0); arr, keep = transport(len(Q), delay, sd, drop, rng); SD[0] = sd; DL[0] = delay
                    for meth in METHODS:
                        tt, R = reference(Q, arr, keep, meth); m = metrics(Q, tt, R, delay)
                        m.update(run=tag, delay_ms=delay * 1000, jitter_sd_ms=sd * 1000, drop=drop, method=meth); res.append(m)
                        print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items()}), flush=True)
    json.dump(res, open(out, "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
