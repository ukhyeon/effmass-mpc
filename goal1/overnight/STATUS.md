# STATUS

Started 2026-11-17 23:18 KST. Disk free at start: 1.4 TB (/dev/sda6, 19% used).

| WP | Budget | State | Start | End | Runs |
|---|---|---|---|---|---|
| 0 prep | 0.5 h | done | 23:18 | 23:25 | 3 |
| 1 fast hand cause | 1.5 h | done | 23:25 | 00:20 | 28 |
| 2 posture zone maps | 0.75 h | done | 00:16 | 00:32 | 0 (no MPC) |
| 3 static scene matrix | 1.75 h | done | 00:30 | 01:10 | 65 |
| 4 moving collab scenes | 1.5 h | done | 01:10 | 01:52 | 28 |
| 5 perception imperfection | 1.0 h | done | 01:52 | 02:26 | 26 |
| 6 tuning | 1.0 h | done | 02:25 | 02:38 | 24 |
| 7 wrap-up | 1.25 h | done (videos 03:39-04:48, docs and report 05:15-05:28) | 03:39 | 05:28 | 8 display runs |

## Run time calibration (WP0, measured start to end of the wrapper)
- single pass (V2 front, 304 steps): 27.4 s (both runs). Timeout 3x = 82 s, used 90 s.
- 60 s round trip (R1 v050, 3600 steps): 75.6 s. Timeout 3x = 227 s, used 230 s. 120 s round trip: estimate 140 s, timeout 420 s.

## Planned run counts (recomputed from the times above)
| WP | Plan | Runs | Est. run time |
|---|---|---|---|
| 1 | variants L0.1, L0.2, P(cv), V0.75, V1.0 x speeds 0.5, 1.0 x uniform/ours, seed 0; then best-two combination | 20 + 4 | ~45 min |
| 3 | 4 postures x 3 directions x 2 conds x 2 seeds + calibration (off, uniform, <= 3 tries) | 48 + <= 72 | ~55 min |
| 4 | M1, M2 x 0.25, 0.5 m/s x 2 conds x 2 seeds (+ M1 0.5 best variant x 2 conds x 2 seeds if WP1 gives one) | 16 (+4) | ~25 min |
| 5 | lag 50/100/150 ms, jitter 1/2 cm, drop 30 % x 2 conds seed 0; worst two x seed 1; lag 100 + lead 0.1 if L helps | 12 + 4 + 2 | ~23 min |
| 6 | <= 25 single-pass runs | <= 25 | ~12 min |
Seeds are not reduced: everything fits the budgets.

## Reserve work
| item | state | start | end | runs |
|---|---|---|---|---|
| R1 seed 2 for WP3 / WP4 | done inside WP3 / WP4 | | | (counted there) |
| R2 gain knob on 2 WP3 scenes | done | 02:38 | 02:44 | 12 |
| R3 M3 bent-arm forearm task | done | 02:44 | 02:55 | 6 |
| R4 ROS replay path RTF / delay | skipped (not possible with existing scripts only) | 02:56 | 02:56 | 0 |

## Self-made follow-ups
| item | state | start | end | runs |
|---|---|---|---|---|
| F1 P (cv) in the crossing task M2 | done | 02:57 | 03:12 | 12 |
| F2 P (cv) under jitter / lag, and R1 1.0 m/s P seed 2 | done | 03:12 | 03:25 | 8 |
| F3 velocity filter for P under jitter | done | 03:26 | 03:38 | 6 + 2 bit-check |
| F4 lag 100 ms + jitter 2 cm: static vs P + filter | done | 04:07 | 04:28 | 12 |
| F5 speed cap under lag + jitter + P + filter at 1.0 m/s | done | 04:29 | 04:45 | 4 |
| F6 seed 1 and 0.5 m/s check of F5 (c2 radius) | done | 04:49 | 05:02 | 6 |
| F7 full Kinect candidate (P + filter + radius 0.5) on clean R1 data | done | 05:03 | 05:14 | 8 |
| F8 full Kinect candidate on M1 / M2 0.5 m/s | done | 05:15 | 05:26 | 8 |

## Totals
268 runs logged in runs.tsv (all wall-time ok, 0 timeouts, 0 retries): WP0 7 (incl. 4 bit-check re-runs), WP1 28, WP3 65, WP4 28, WP5 26, WP6 24,
reserve 2: 12, reserve 3: 6, follow-ups F1-F8: 64, display recordings 8. Isaac wall time about 4.9 h.
