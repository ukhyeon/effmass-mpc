# STATUS

Started 2026-11-17 23:18 KST. Disk free at start: 1.4 TB (/dev/sda6, 19% used).

| WP | Budget | State | Start | End | Runs |
|---|---|---|---|---|---|
| 0 prep | 0.5 h | done | 23:18 | 23:25 | 3 |
| 1 fast hand cause | 1.5 h | done | 23:25 | 00:20 | 28 |
| 2 posture zone maps | 0.75 h | done | 00:16 | 00:32 | 0 (no MPC) |
| 3 static scene matrix | 1.75 h | done | 00:30 | 01:10 | 65 |
| 4 moving collab scenes | 1.5 h | waiting | | | |
| 5 perception imperfection | 1.0 h | waiting | | | |
| 6 tuning | 1.0 h | waiting | | | |
| 7 wrap-up | 1.25 h | waiting | | | |

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
