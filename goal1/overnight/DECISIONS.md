# DECISIONS

Format: time / situation / alternatives / choice and reason / original criterion and its verdict.

## D0-1 23:24 regression baseline
- Situation: baseline runs on_base_gk000_s0 and on_base_gain_s0 (V2 front, single pass, 304 steps, B1 log off, cost aggregation off) vs the V2 records
  v1_front_0.38h1_gk000_s0 / v1_front_0.38h1_gain_s0 (B1 log on, aggregation on): joint commands bit-equal over 304 steps (max diff 0.0).
  Repeat of R1 r1_v050_gk000_s0 (60 s round trip): bit-equal over 3600 steps.
- Choice: the V2 records are tonight's regression reference. After any code change, on_base_gk000_s0 and on_base_gain_s0 are re-run and compared with them.
- Original criterion: bit-equal with V2 records. Verdict: pass.

## D1-1 23:30 WP1 phase metric
- Situation: plan asks for gap per phase (t_cross - time the hand is fully extended) instead of hand state.
- Alternatives: (a) t_cross = time of the pass minimum; (b) t_cross = time the TCP passes the fingertip of the hold pose along the A->B line (robot-side timing only); (c) keep hand states.
- Choice: (b), because it does not depend on the gap itself. Reference event = hold onset (fingertip fully extended). Critical window = [-T_reach, +0.25 s]
  (robot crosses while the hand is still reaching or has just arrived); hold window = [+0.5, +1.25 s]. Deficit = eta - gap at the pass minimum.
- Seed spread of the critical-window metrics from R1 seeds 0-2 (base): max deficit 0.9-2.2 cm, min gap 0.1-2.2 cm, mean deficit 1.4-3.3 cm.
  A variant counts as better only if it beats base seed 0 by more than this spread.

## D1-2 00:00 WP1 combination and repeat
- Situation: L02 and P gave the largest critical-window improvement at 1.0 m/s (ours max deficit 13.3 -> 3.5 / 5.0 cm, min gap 2.0 -> 7.5 / 6.1 cm).
  L02 raised the near-human robot speed p99 to 0.66 (uniform) / 0.76 (ours) m/s at 1.0 m/s, above the 0.5 m/s cap that the profile is meant to keep;
  P kept p99 0.476-0.478. Speed-cap variants V075 / V100 did not reduce the deficit (ours max deficit 12.3 / 12.3 cm) and lowered the minimum gaps.
- Alternatives: combine P+L02 (best two by deficit); combine P+L01 (milder lead); only repeat P with seed 1.
- Choice: run P+L02 (plan: "best two combined") and P with seed 1 (single-seed results otherwise), 8 runs, ~16 min. Within WP1 budget.
- Note (interpretation, not checked): the lead variant moves the shapes along the file velocity; during retraction it moves them away from the robot,
  so the speed-cap distance test sees a farther hand and the robot speeds up near the true hand.

## D1-3 00:18 WP1 outcome used later
- Clearly better variant: P (--meff-motion cv), 2 seeds, keeps the speed cap. Used for WP4 (M1 0.5 m/s, both conditions, plan rule) and as the
  Kinect candidate. L01 was effective too (ours 1.0 m/s max deficit 13.3 -> 9.1 cm, beyond spread), so WP5 adds "lag 100 ms + lead 0.1 s" (plan rule).
- Own addition for WP5: "lag 100 ms + P", because P is the recommended Kinect setting and its tolerance to lag is the practical question. 2 runs.

## D2-1 00:20 WP2 posture checks
- Situation: the first check flagged every posture, including the B2a reference, because the upper-arm capsule overlaps the chest capsule at the
  shoulder (by construction) and the wrist axis-angle includes forearm twist. Class (nai) measurement method.
- Choice: overlap test = forearm / hand shapes vs every non-arm shape, upper arm vs non-arm shapes except the chest and right shoulder it attaches to;
  wrist split into twist (pro/supination, limit 90 deg) and swing (limit 70 deg); elbow hyper-extension <= 6 deg allowed (P4 has 3 deg).
- P3_bend90 and P6_handover: the hanging upper arm overlapped the abdomen capsule by 0.9 / 1.4 cm. Fixed by 14 deg abduction of the upper-arm target.
- P6_handover palm-up target: with twist <= 80 / swing <= 60 deg and the abducted upper arm the optimiser reaches palm 30 deg from up, elbow 48 deg
  (target 60), forearm 13 deg off. Palm weight 1.0 and 2.0 were tried (2 tries): both collapsed the elbow to 4-5 deg. Kept weight 0.5 (palm 30 deg off). Recorded as a limit.
- Zone thickness first run measured hand directions from the hand COM, which lies outside the thin hand capsules (class (nai)); fixed by starting from the
  nearest capsule axis point. Only the corrected numbers are reported.

## D2-2 00:30 WP3 posture pick
- Rule (written before looking): greedy max-min distance of the 9-direction ours thickness vector, the reference reach pose counted as already present.
- Result: P3_bend90, P6_handover, P4_side, P2_bend45. P3 is pulled far by forearm_top 37 cm (the ray above the forearm runs inside the hanging upper
  arm's zone). P4_side is close to the reach pose once rotated toward the robot, but its hand is rolled 90 deg (inner / outer side thickness swapped).
- Alternative considered: replace P4 with P7_down (light hand, mu max 1.7 kg) for a likely "no difference" case. Not done: the pre-set rule stands.
