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
ours 0.5 m/s 6.1-7.8 -> 2.9-3.3, 1.0 m/s 11.7-13.8 -> 4.7-5.0. (0.25 m/s base only: uniform 2.4-3.3, ours 5.1-7.0; P not run.)
Candidate for the live Kinect stage: --meff-motion cv (P), lead 0, speed cap 0.5 m/s kept. Lead (L) is not recommended because it lets the robot
exceed the speed cap near the true hand at 1.0 m/s.
Figure: goal1/overnight/wp1_phase_gap.png. Numbers: logs/meff_isaac/on_wp1_summary.json, on_wp1_r1_passes.json, on_wp1_var_passes.json, on_wp1_var_runs.json.

## WP2 Zone maps by posture and direction (00:16-00:32, no MPC)
Seven right-arm postures from the B2a pose (shoulder 3 + elbow hinge + wrist 3 optimised with the node body model; shapes and inertia recomputed
with g13_human_precompute.py). All pass the checks after the fixes in D2-1. Preview: goal1/overnight/postures_preview.png (first figure to look at).
Ours zone thickness by direction (cm; uniform is 6.5-10.2 cm everywhere, the upper value where shapes overlap); mu ranges (kg, robot 4.6 kg isotropic):
| posture | elbow deg | tip front | hand back | palm | hand inner | hand outer | forearm top | forearm side | elbow | upper-arm side | hand mu | forearm mu | upper-arm mu |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| P1_reach (ref) | 5 | 20.3 | 9.1 | 8.8 | 13.3 | 6.5 | 14.8 | 12.7 | 24.6 | 20.6 | 0.52-2.99 | 1.36-2.87 | 2.90-3.28 |
| P2_bend45 | 42 | 12.4 | 7.3 | 7.2 | 11.3 | 6.5 | 17.2 | 13.4 | 23.4 | 18.2 | 0.51-1.55 | 1.45-2.88 | 2.48-3.30 |
| P3_bend90 | 89 | 9.5 | 7.0 | 8.1 | 6.6 | 10.8 | 37.3 | 13.9 | 21.9 | 19.1 | 0.50-1.08 | 1.46-3.06 | 2.35-3.27 |
| P4_side | 4 | 21.1 | 9.3 | 8.8 | 6.5 | 13.4 | 15.9 | 14.1 | 24.9 | 19.1 | 0.52-3.72 | 1.35-3.30 | 2.70-3.76 |
| P5_up | 5 | 17.5 | 9.5 | 9.3 | 13.6 | 6.5 | 14.7 | 14.5 | 23.9 | 21.8 | 0.52-3.13 | 1.37-3.16 | 2.77-3.53 |
| P6_handover | 48 | 9.1 | 8.9 | 7.6 | 7.2 | 13.8 | 17.8 | 13.9 | 23.2 | 17.9 | 0.51-1.28 | 1.44-3.10 | 2.41-3.39 |
| P7_down | 5 | 13.4 | 7.8 | 8.0 | 11.8 | 6.5 | 13.5 | 13.9 | 24.1 | 21.8 | 0.52-1.71 | 1.39-2.97 | 2.75-3.43 |
- The fingertip-front thickness follows the arm: straight arms 17.5-21.1 cm, bent arms 9.1-12.4 cm (with a bent elbow the hand points along a light
  direction: hand mu max 1.08-1.55 kg vs 2.99-3.72 kg straight).
- Back of hand and palm stay near uniform (7.0-9.5 cm) in every posture. One hand side is always at the uniform value (6.5 cm).
- Elbow and upper arm are thick in every posture (17.9-24.9 cm); no direction reaches eta_max (25 cm) at the surface, the elbow comes closest (24.9).
- forearm_top of P3 (37.3 cm) is the hanging upper arm's zone above the forearm, not the forearm's own.
- Limits: P6_handover palm is 30 deg from up and the elbow 48 deg (D2-1). Single static frame per posture; thickness is along one ray per direction.
- Picked for WP3 (D2-2): P3_bend90, P6_handover, P4_side, P2_bend45.
Files: logs/meff_isaac/on_wp2_postures.json, on_wp2_zone.json; figures goal1/overnight/postures_preview.png, wp2_zone_thickness.png.

## WP3 Static scene matrix (00:30-01:10, 65 runs, all wall-time ok, RTF 1.15-1.25, infeasible 0, recapture 0)
Robot path fixed (front scene, single pass, 420 steps); each WP2 posture placed by a rigid yaw + translation so that the off path passes 2.5 cm from the
target part on the intended side (search: on_wp3_scenes.py). Calibration on off and uniform only; every built scene passed on the first try
(off gap 2.5 cm, uniform arrives, infeasible 0) except P4 forearmside18 (D3-3). Uniform and ours, seeds 0-2 (seed 2 = reserve item 1).
Scene count: original rule (torso >= 0.35 m) built 5 of 12. User instruction (0.25 m) added P3 top, P6 fingertip-front. Added rules (relax first):
0.18 m added P6 top and P6 beside-forearm. Excluded: P2 / P3 beside-forearm (torso <= 0.09 m from the path), P4 beside-forearm (no human-side
placement) = 3 of 12. Purpose check: all 3 approach directions present; bent-arm postures P2 (42 deg), P3 (89 deg), P6 (48 deg).

| scene | torso min (m) | uniform gap mean (range) cm | ours gap mean (range) cm | mesh gap u / o cm | side | u vs front deg | mu kg | ours eta cm | gap/eta u / o | pass time o/u | v99 max m/s | ours torso-cost steps | class |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| P1 reach front (V2) | 0.35 | 6.0 | 12.0 | - | front | - | - | - | - | - | - | - | ours farther |
| P1 reach over hand (V2) | 0.35 | 5.4 | 8.2 | - | up | - | - | - | - | - | - | - | ours farther |
| P2_tipfront | 0.35 | 6.0 (5.3-6.5) | 9.9 (9.3-10.4) | 5.9 / 9.9 | front | 43 | 1.03 | 10.4 | 0.92 / 0.95 | 0.96 | 0.503 | 0 | ours farther |
| P2_top | 0.35 | 6.0 (5.8-6.3) | 7.5 (6.1-8.7) | 5.9 / 7.8 | up | 92 | 0.59 | 7.0 | 0.93 / 1.06 | 0.92 | 0.472 | 0 | no difference (light) |
| P3_tipfront | 0.35 | 5.9 (5.6-6.1) | 8.5 (8.2-8.8) | 6.2 / 8.8 | front | 12 | 0.92 | 9.7 | 0.90 / 0.88 | 1.06 | 0.518 | 0 | ours farther |
| P3_top25 | 0.25 | 5.9 (5.7-6.2) | 6.0 (5.6-6.2) | 7.9 / 8.1 | up | 78 | 0.54 | 6.6 | 0.91 / 0.91 | 1.00 | 0.496 | 0 | no difference (light) |
| P4_tipfront | 0.35 | 5.9 (5.2-6.5) | 11.5 (11.0-11.9) | 5.8 / 12.5 | front | 42 | 1.30 | 12.2 | 0.91 / 0.94 | 1.06 | 0.507 | 0 | ours farther |
| P4_top | 0.35 | 5.9 (5.6-6.1) | 9.5 (8.9-10.0) | 8.6 / 11.7 | up | 73 | 0.85 | 9.2 | 0.90 / 1.04 | 0.98 | 0.493 | 0 | ours farther |
| P6_tipfront25 | 0.25 | 6.0 (5.8-6.1) | 10.2 (9.8-10.6) | 5.8 / 9.7 | front | 15 | 1.08 | 10.8 | 0.92 / 0.94 | 1.00 | 0.546 | 0 | ours farther |
| P6_top18 | 0.18 | 6.2 (5.4-6.6) | 6.6 (6.1-6.9) | 6.0 / 6.4 | up | 66 | 0.57 | 6.8 | 0.95 / 0.96 | 1.05 | 0.503 | 97 | no difference (light), torso cost in ours |
| P6_forearmside18 | 0.18 | 5.6 (5.5-5.8) | 11.2 (11.2-11.2) | 5.9 / 11.1 | front | 20 | 0.65 | 7.6 | 0.87 / 1.48 | 1.09 | 0.498 | 0 | ours farther |
Classification rule written before the runs (on_wp3_summary.py): threshold = max(1 cm, seed range); "light" = ours eta at the minimum < 7.5 cm.
- Ours keeps more distance when the robot crosses in front of the fingertip (all 4 postures, +2.7 to +5.6 cm, 3 seeds each beyond the seed range),
  and over the hand of the side-arm posture P4 (+3.7 cm). Over the hand of bent-arm postures (P2, P3, P6) there is no difference: mu there is
  0.54-0.59 kg (near mu_ref 0.53), so ours eta stays 6.6-7.0 cm. This is the intended direction dependence, not a failure.
- Beside the forearm (P6 only): ours 11.2 vs uniform 5.6 cm although ours eta at the forearm minimum is 7.6 cm (gap/eta 1.48).
  (interpretation) the heavier hand zone keeps the robot out before it reaches the forearm; not checked.
- Torso: torso cost appeared only in P6_top18 for ours (28-35 steps per run, torso gives 86-100 % of the per-step maximum cost on those steps);
  the gap there equals uniform's, so the scene says nothing about the arm. Uniform never had torso cost; no other scene had torso cost in either condition.
- Same passing side in every seed and condition; no path change; all arrived; pass time ours / uniform 0.92-1.09.
- Near-human speed p99 slightly above 0.5 m/s in single passes: max 0.546 (P6_tipfront25), 0.518 (P3_tipfront). Not seen in the R1 round trips.
Figures: goal1/overnight/wp3_effect_table.png, wp3_scene_gap.png. Numbers: logs/meff_isaac/on_wp3_summary.json, on_wp3_runs.json, on_wp3_passes.json,
on_wp3_scenes.json.

## WP4 Moving collaboration scenes (01:10-01:52, 28 runs, all wall-time ok, RTF >= 1.159, infeasible 0, recapture 0)
Motions (on_wp4_motion.py, node code, D4-1, D4-2): M1 hand-over = P6_handover held 3 s, reach / retract 0.38 m along the reach line (min-jerk,
peak 0.253 / 0.500 m/s); M2 crossing = P7_down fingertip swept 0.40 m sideways through the robot TCP line at 40 deg (peak 0.250 / 0.500 m/s),
resting 1.5 s on each side (on the far side the hand sits inside the off-path sweep). Front robot path, round trips, 60 s, uniform and ours,
seeds 0-2 (seed 2 = reserve item 1); M1 0.5 m/s also with P (--meff-motion cv), seeds 0-1. Analysis by crossing phase (WP1 method).
| motion / speed | cond | min gap per seed (cm) | critical-window max deficit per seed (cm) | hold-window mean gap (cm) | penetration (shape / mesh steps) | passes per run | stalls in front of the hand (count, s, 3 runs) |
|---|---|---|---|---|---|---|---|
| M1 0.25 | uniform | 5.3 / 2.8 / 4.1 | 1.2 / 3.7 / 2.4 | 5.8 | 0 / 0 | 27 / 26 / 27 | 0 |
| M1 0.25 | ours | 7.5 / 6.6 / 7.6 | 4.1 / 4.6 / 2.9 | 10.1 | 0 / 0 | 27 / 25 / 27 | 0 |
| M1 0.5 | uniform | 3.0 / 3.5 / 3.9 | 3.5 / 3.0 / 2.6 | 5.9 | 0 / 0 | 28 / 26 / 27 | 0 |
| M1 0.5 | ours | 4.7 / 7.7 / 6.5 | 6.7 / 3.8 / 4.5 | 10.1 | 0 / 0 | 27 / 25 / 27 | 0 |
| M1 0.5 P | uniform | 5.6 / 5.5 (s0, s1) | 0.7 / 0.9 | 6.2 | 0 / 0 | 28 / 26 | 0 |
| M1 0.5 P | ours | 9.0 / 10.0 (s0, s1) | 0.9 / 0.6 | 10.0 | 0 / 0 | 26 / 25 | 0 |
| M2 0.25 | uniform | 2.6 / 2.5 / 3.3 | 3.9 / 4.0 / 3.2 | - | 0 / 0 | 18 / 16 / 18 | 13, 32.5 s |
| M2 0.25 | ours | 3.6 / 4.2 / 4.3 | 3.1 / 2.8 / 3.0 | - | 0 / 0 | 14 / 12 / 14 | 20, 38.9 s |
| M2 0.5 | uniform | 0.6 / 1.2 / 0.0 | 5.9 / 5.3 / 6.5 | - | 0 / 0 | 21 / 21 / 21 | 17, 13.7 s |
| M2 0.5 | ours | 6.4 / 6.7 / 3.8 | - / - / 3.1 | - | 0 / 0 | 18 / 19 / 18 | 3, 5.0 s |
(critical window: M1 robot crosses within [-T_reach, +0.25 s] of hold onset; M2 within +-T_sweep/2 of the hand reaching the TCP line. "-" = no pass
of that seed fell in the window.) Near-human speed p99 max 0.472-0.493 m/s in every group.
- M1 hand-over: ours keeps the larger distance in every seed at both speeds (min gap +1.7 to +3.8 cm over uniform's seed range; hold gap 10.1 vs 5.8-5.9 cm)
  with no loss of throughput (25-27 vs 26-28 passes). With P the critical deficit drops to <= 0.9 cm in both conditions and ours' min gap rises to 9.0-10.0 cm (2 seeds).
- M1 torso: the hand-over placement has the torso 0.26 m from the sweep; ours had torso cost on 0-55 steps per run, 0-1.4 % of the summed per-step
  maximum cost. Small; the hand dominates.
- M2 crossing: no penetration anywhere. At 0.5 m/s uniform came to 0.0-1.2 cm, ours stayed at 3.8-6.7 cm. At 0.25 m/s ours waits more in front of the
  hand (20 stalls, 38.9 s over 3 runs vs 13, 32.5 s) and completes fewer passes (12-14 vs 16-18 per 60 s); at 0.5 m/s ours waits less (3, 5.0 s vs
  17, 13.7 s) and completes 18-19 vs 21. (interpretation) with the hand resting inside the path the larger zone makes ours wait for the hand to leave
  rather than squeeze past; not checked frame by frame.
Figure: goal1/overnight/wp4_phase_gap.png. Numbers: logs/meff_isaac/on_wp4_summary.json, on_wp4_runs.json, on_wp4_passes.json, on_wp4_motion_check.json.

