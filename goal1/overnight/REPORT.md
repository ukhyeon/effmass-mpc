# Overnight report (2026-11-17 to 2026-11-18)

(Front page is written at the end, in WP7.)

## WP0 Preparation (23:18-23:25)
- Deleted the 6 frame / recording folders (r1/rec, r1/frames_r1_main, r1/frames_r1_fast, v2/rec, v2/frames_v2_main, v2/frames_v2_gain_knob, about 5.1 GB) after
  ffprobe (r1_main/r1_fast 645 frames, v2_main 236, v2_gain_knob 172, all 1920x1080 h264) and the existing bit checks (r1_video_bitcheck.json, v2_video_bitcheck.json, all true).
- Local commit 2927641 "r1: moving hand replay" (R1 scripts, R1 summary json, R1 figures, .gitignore widened for R1 code, summary json, figures and overnight notes).
- Regression baseline: tonight's runs reproduce the V2 records bit for bit (uniform s0 and ours s0, 304 steps), and an R1 60 s round trip bit for bit (3600 steps).
  Joint commands do not depend on B1 logging or cost aggregation (V2 records had both on, tonight's runs have both off).
- Wall time: single pass 27.4 s, 60 s round trip 75.6 s. All planned run counts fit; seeds were not reduced (STATUS.md).

## WP1 Why the zone collapses for a fast hand (23:25-00:20, 28 runs, all RTF >= 1.149, recapture 0, infeasible 0)
Question: at 1.0 m/s ours dipped to ~3.5 cm while eta was 16.6 cm. (a) no human motion prediction, (b) the 0.5 m/s near-human speed cap, (c) other.

Method: R1 passes re-grouped by crossing phase = time the TCP passes the fingertip line minus hold onset (D1-1). Critical window = robot crosses
while the hand is still reaching or has just arrived ([-T_reach, +0.25 s]). Deficit = eta - gap at the pass minimum. Variants on the R1 scene, seed 0,
0.5 and 1.0 m/s, uniform and ours: L01 / L02 (shapes moved ahead by file velocity x 0.1 / 0.2 s, replay side), P (existing --meff-motion cv,
constant-velocity shapes over the horizon), V075 / V100 (diagnostic profiles tuned_d1v075 / tuned_d1v100, speed cap 0.75 / 1.0 m/s), P+L02; P repeated with seed 1.

R1 by phase (seed 0-2): the worst gaps sit in the critical window. At 1.0 m/s ours max deficit 11.7-13.8 cm (3 seeds), uniform 4.9-6.8 cm.
In the hold window the gap is steady (uniform 5.7-6.0, ours 10.5-11.6 cm). Seed spread of the critical metrics: 0.1-3.3 cm.

Critical-window results (cm; min gap / max deficit; seed 0 unless noted):
| speed / cond | base (s0-s2) | L01 | L02 | P (s0, s1) | V075 | V100 | P+L02 |
|---|---|---|---|---|---|---|---|
| 0.5 uniform | 1.6-3.7 / 2.8-4.9 | 4.2 / 2.3 | 5.7 / 0.8 | 6.6, 6.5 / -0.1, -0.0 | 3.5 / 3.0 | -0.1 / 6.6 | 8.0 / -1.5 |
| 0.5 ours | 5.4-6.4 / 6.1-7.8 | 8.4 / 4.7 | 8.7 / 3.1 | 9.3, 8.8 / 3.3, 2.9 | 6.0 / 7.2 | 4.2 / 8.0 | 10.5 / 2.2 |
| 1.0 uniform | -0.3-1.6 / 4.9-6.8 | 3.0 / 3.5 | 6.5 / 0.0 | 4.7, 5.6 / 1.8, 0.9 | 0.6 / 5.9 | -0.4 / 6.9 | 6.4 / 0.1 |
| 1.0 ours | 2.0-2.2 / 11.7-13.8 | 4.7 / 9.1 | 7.5 / 3.5 | 6.1, 6.1 / 5.0, 4.7 | 1.9 / 12.3 | 0.9 / 12.3 | 7.8 / 0.8 |
Near-human robot speed p99 (true human): base, L01, P 0.47-0.48 m/s; L02 at 1.0 m/s 0.66 / 0.76; P+L02 at 1.0 m/s 1.04 / 0.95; V075 0.73; V100 0.98-1.00.
Hold-window gap unchanged by P (ours 10.5-10.6, uniform 5.7-5.9). Shape penetration only with V100 (uniform 1 step at 0.5 m/s, 6 steps at 1.0 m/s).

Conclusion:
- (a) Missing prediction is the main cause. P lowers ours max deficit at 1.0 m/s from 11.7-13.8 to 4.7-5.0 cm (2 seeds, both beyond the seed spread)
  and raises the critical min gap from 2.0-2.2 to 6.1 cm, with the speed cap intact. Lead L02 helps as much but breaks the speed cap at 1.0 m/s.
- (b) The speed cap is not the cause: raising it to 0.75 / 1.0 m/s does not reduce the deficit (12.3 cm) and lowers the minimum gaps.
- (c) Remaining deficit with P at 1.0 m/s: ~5 cm for ours, ~1-2 cm for uniform. (interpretation) the residual is the steady-state shortfall seen
  in the hold window (ours gap/eta ~0.85) plus the time the robot needs to move away at <= 0.5 m/s; not checked.
Required extra margin (= critical-window max deficit, cm), base -> P: uniform 0.5 m/s 2.8-4.9 -> ~0, 1.0 m/s 4.9-6.8 -> 0.9-1.8;
ours 0.5 m/s 6.1-7.8 -> 2.9-3.3, 1.0 m/s 11.7-13.8 -> 4.7-5.0. (0.25 m/s base only: uniform 2.5-3.2, ours 5.6-7.0; P not run.)
Candidate for the live Kinect stage: --meff-motion cv (P), lead 0, speed cap 0.5 m/s kept. Lead (L) is not recommended because it lets the robot
exceed the speed cap near the true hand at 1.0 m/s.
Figure: goal1/overnight/wp1_phase_gap.png. Numbers: logs/meff_isaac/on_wp1_summary.json, on_wp1_r1_passes.json, on_wp1_var_passes.json, on_wp1_var_runs.json.

