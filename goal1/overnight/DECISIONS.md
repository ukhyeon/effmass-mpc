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
  shoulder (by construction) and the wrist axis-angle includes forearm twist. 원인 분류 (나) 측정 방법.
- Choice: overlap test = forearm / hand shapes vs every non-arm shape, upper arm vs non-arm shapes except the chest and right shoulder it attaches to;
  wrist split into twist (pro/supination, limit 90 deg) and swing (limit 70 deg); elbow hyper-extension <= 6 deg allowed (P4 has 3 deg).
- P3_bend90 and P6_handover: the hanging upper arm overlapped the abdomen capsule by 0.9 / 1.4 cm. Fixed by 14 deg abduction of the upper-arm target.
- P6_handover palm-up target: with twist <= 80 / swing <= 60 deg and the abducted upper arm the optimiser reaches palm 30 deg from up, elbow 48 deg
  (target 60), forearm 13 deg off. Palm weight 1.0 and 2.0 were tried (2 tries): both collapsed the elbow to 4-5 deg. Kept weight 0.5 (palm 30 deg off). Recorded as a limit.
- Zone thickness first run measured hand directions from the hand COM, which lies outside the thin hand capsules (원인 분류 (나)); fixed by starting from the
  nearest capsule axis point. Only the corrected numbers are reported.

## D2-2 00:30 WP3 posture pick
- Rule (written before looking): greedy max-min distance of the 9-direction ours thickness vector, the reference reach pose counted as already present.
- Result: P3_bend90, P6_handover, P4_side, P2_bend45. P3 is pulled far by forearm_top 37 cm (the ray above the forearm runs inside the hanging upper
  arm's zone). P4_side is close to the reach pose once rotated toward the robot, but its hand is rolled 90 deg (inner / outer side thickness swapped).
- Alternative considered: replace P4 with P7_down (light hand, mu max 1.7 kg) for a likely "no difference" case. Not done: the pre-set rule stands.

## D3-1 00:36 WP3 infeasible scenes
- Situation: placement search found 5 of 12 scenes (P3 tipfront; P4 tipfront, top; P2 tipfront, top). All forearm-side scenes and all P6 scenes,
  plus P3 top, fail. Binding constraint in every case: torso shapes >= 0.35 m from the off-path robot sweep (best torso gap 0.06-0.30 m).
  With a bent arm the hand is 0.3-0.45 m from the chest; with the forearm alongside the straight path, the shoulder and chest end up next to the path.
- 원인 분류 (마) limit of the scene construction (fixed straight robot path at hand height + torso rule), not of the method under test.
- Alternatives: (1) drop the infeasible scenes and record them (rule in the plan); (2) lower the torso threshold (would let ours' torso zone, up to
  25 cm, act, which defeats the rule's purpose; not mine to change); (3) build new robot paths (new goals + IK) per scene: large change, no time
  budget for calibrating new paths.
- Choice: (1). The reach-pose front and top scenes enter the table with their V2 results (seeds 0-2, same settings, uniform u065 = gk000 bit-equal).
- Original criterion: 4 postures x 3 directions. Verdict: 5 of 12 built; forearm-side direction not covered at all.

## D4-1 00:40 WP4 M1 placement
- Situation: M1 hold pose = P6_handover; with the hand 2.5 cm from the off path the torso is at best 0.26 m from the robot sweep (D3-1).
- Alternatives: (1) keep torso >= 0.35 m by holding the hand ~11 cm from the path (uniform zone 6.5 cm would never act; not a hand-over);
  (2) torso >= 0.25 m (the WP3 rule is a WP3 scene rule; WP4 has none), and report the torso's share of the executed-state cost;
  (3) use another posture for M1 (not a hand-over).
- Choice: (2), same for both conditions. M2 keeps 0.35 m (falls back to a smaller crossing angle first).

## D3-2 (user instruction, received 00:48) WP3 retry of the 7 dropped scenes
- Source: user instruction during the night (also appended to PLAN.md). Not my own decision.
- Content: after the current WP3 batch, retry P3 top, P3 forearmside, P6 tipfront / top / forearmside, P4 forearmside, P2 forearmside with the torso
  rule relaxed to >= 0.25 m (same relaxation as WP4 M1). Calibration as before (off and uniform only). Torso cost share reported offline.
- Reporting: the 0.35 m scenes and the 0.25 m scenes are kept as separate groups; the original rule (0.35 m) and its verdict (5 of 12) stay in the report.

## D3-3 00:58 WP3 under the added rules (relax -> modify -> exclude)
- Status before: with torso >= 0.25 m (D3-2) 7 of 12 scenes; still excluded P3 forearmside, P6 top, P6 forearmside, P4 forearmside, P2 forearmside
  (5 of 12 = 42 %, >= 30 %), and the forearm-side direction was missing, so the WP3 purpose (all 3 directions, >= 2 bent-arm postures) was not met.
- Step 1, relax: torso distance 0.35 m (original) -> 0.25 m (user, D3-2) -> 0.18 m (mine) for the scenes still missing.
  Scan of the best achievable torso gap: P6 top 0.192, P6 forearmside 0.233, P4 forearmside 0.191, P3 forearmside 0.092, P2 forearmside 0.063 m.
  0.18 m chosen as the smallest value that adds the forearm-side direction without going near the uniform zone (6.5 cm) of the torso.
  What the torso rule protects: no torso cost, so that the arm is what the conditions differ on. Checked instead by the offline executed-state torso
  cost (on_analyze cost_split): steps with any torso cost and torso share of the per-step maximum, reported per scene and condition.
- Built at 0.18 m: P6 top18, P6 forearmside18, P4 forearmside18. Calibration (off, uniform):
  P6 top18 and P6 forearmside18 pass (off gap 2.5 cm, uniform arrives, infeasible 0).
  P4 forearmside18 try 1 placed the human between the robot base and the path (chest x 0.12 m; my search had no side check) and uniform did not arrive.
  Try 2 with "chest beyond the path (x > 0.60 m)": no placement. Excluded after 2 tries (no third geometric option at 0.18 m).
  Every other scene was re-checked against the side condition: all chests at x 0.81-1.42 m.
- Step 2, modify: not needed for the purpose after step 1 (all 3 directions present; bent-arm postures P2, P3, P6).
- Step 3, exclude: P3 forearmside, P2 forearmside (torso <= 0.09 m from the path), P4 forearmside (no human-side placement): 3 of 12 = 25 %.
- Groups in the report: torso >= 0.35 m (5 scenes, original rule), 0.25 m (P3 top25, P6 tipfront25, user rule), 0.18 m (P6 top18, P6 forearmside18).

## D4-2 01:19 WP4 motions as generated
- M1: hold = P6_handover at the WP3 tipfront placement with torso >= 0.25 m (D4-1; the placement equals scene P6_tipfront25). Retract D = 0.38 m
  (smallest with hand >= 0.35 m from the off sweep; R1 had 0.33). Peak tip speed 0.253 / 0.500 m/s, line deviation 0.8 mm, period 10.2 / 7.3 s.
- M2: P7_down fingertip swept 0.40 m sideways through the TCP line. My own start values: torso >= 0.35 m and the largest crossing angle (90 deg first).
  No angle met 0.35 m (relax, added rule 1): torso >= 0.25 m gives a 40 deg crossing angle (torso 0.277 m). What the torso value protected: no torso cost;
  checked offline (torso cost share) like WP3. The hand rests 1.5 s at each end: on the human side 3 cm from the off sweep, on the far side inside it
  (-3 to -4 cm), so the robot meets a hand that sits in its path. This is the intended "crossing" task, kept for both conditions.
- Runs: M1, M2 x 0.25, 0.5 m/s x uniform / ours x seeds 0-2 (seed 2 = reserve item 1) + M1 0.5 m/s with P (WP1 best variant, D1-3) seeds 0-1.

## D6-0 01:52 WP6 tuning goals (written before any tuning run)
- Problem chosen (1 of <= 2): near-human robot speed p99 above the 0.52 m/s must-limit in single passes: WP3 uniform P6_tipfront25 s1 0.546,
  P3_tipfront s2 0.518 (2 of 54 WP3 runs; R1 / WP4 round trips stayed <= 0.493). This is a must-criterion, so it outranks the gap/eta goal.
  gap/eta was considered as a second problem but the seed-0 means are already at the target (tune set uniform 0.948, ours 0.983); its seed spread
  (0.056-0.094 between seeds, seed 2 always lowest) is larger than any plausible setting effect, so it is not tuned.
- Must (all scenes): infeasible 0, arrival 100 %, shape penetration 0, near-human speed p99 <= 0.52 m/s, RTF >= 1.0.
  Then: gap/eta near 0.95 with small spread across scenes, pass time <= +20 % vs uniform under t1, same passing side across seeds.
- Shared settings only, one change each: c1 speed_cap_w 3e5 -> 1e6; c2 speed_cap_dist 0.4 -> 0.5 m. k_gain, mu_ref, eta_max untouched.
- Split (WP3 scenes, torso-confounded P6_top18 left out): tune = P2_tipfront, P4_top, P6_tipfront25, P3_top25; check = P3_tipfront, P4_tipfront, P2_top, P6_forearmside18.
- Runs (<= 25): each candidate on the tune set, uniform and ours, seed 1 (the seed of the 0.546 case) = 16; the better candidate on the check set,
  seed 2 (the seed of the 0.518 case) = 8. Total 24.
- Improvement must exceed the WP3 seed spread of the 4-scene mean: gap 0.57-0.75 cm (tune), 0.54-0.59 cm (check); ratio 0.056-0.094.
  A candidate is dropped if the check set gets worse beyond that spread on any must item or on gap / pass time.
- Original-setting reference = the WP3 runs with t1 (same scenes, same seeds).

## D5-1 02:12 WP5 worst two and extension levels
- Worst two (pre-set rule: largest critical-window max deficit or lowest min gap vs the base seed range, either condition): lag100 (ours max deficit
  11.1 cm vs base 6.1-7.8) and jit2 (ours min gap 4.9 vs base 5.4-6.4; uniform 1.8; highest jerk). Both rerun with seed 1 (4 runs).
- Every planned level (lag 50-150 ms, jitter 1-2 cm, drop 30 %) stayed without penetration and mostly inside the base seed range, so the plan's
  levels give only a lower bound of tolerance. WP5 purpose (added rule 3): a tolerance level for each of lag, jitter, dropout. Added levels
  lag 300 ms, jitter 4 cm, drop 60 % (uniform and ours, seed 0; 6 runs) to look for the level where it breaks. Within the WP5 time budget.

## D6-1 02:37 WP6 verdict
- c2 (speed_cap_dist 0.5 m): no effect on the tune set (most runs identical to t1, P6_tipfront25 uniform still 0.546 m/s) -> dropped after the tune set.
- c1 (speed_cap_w 1e6): tune set (seed 1) and check set (seed 2) both meet every must item; worst p99 0.546 -> 0.511 and 0.518 -> 0.499 m/s.
  Mean p99 reduction 0.015 m/s vs the WP3 seed spread of p99 (median per scene / condition) 0.031 m/s -> by the pre-set rule not an improvement.
  Other method for the same question: paired runs (same scene, condition, seed) p99 lower in 15 of 16.
  Gap, gap / eta, pass time (uniform +4.5 % tune, +1.3 % check; ours 0 / -2 %) and passing side stayed within the seed spread.
- Written as goal1/config/tuned_t2a.yml with the verdict in its header; t1 and all defaults untouched. Adoption left to the user.

## DR2-1 02:38 reserve 2 scenes
- Gain knob on P4_tipfront (largest ours - uniform, +5.6 cm, mu 1.30 kg) and P2_top (light direction, mu 0.59 kg, no difference). k 0 and 0.10 exist
  (WP3, seeds 0-2); added k 0.05 and 0.15, seeds 0-2 (12 runs). Expected: P4_tipfront gap grows with k until eta saturates or the path changes;
  P2_top barely changes (mu near mu_ref, eta stays ~ d_base for any k).

## DR3-1 02:47 reserve 3 (M3 bent-arm forearm task)
- Motion: P6_handover (elbow 48 deg) at the WP3 placement P6_forearmside18 (torso >= 0.18 m, the only forearm-side placement that exists, D3-3);
  fingertip moved +-5 cm along the forearm axis (10 cm stroke), min-jerk, peak 0.25 m/s, 0.5 s rest at each end, period 2.5 s, 60 s.
- Measured part: forearm + hand (as in WP3). Uniform and ours, seeds 0-2 (6 runs). Expected: ours keeps the larger forearm/hand gap as in the static
  P6_forearmside18 scene (11.2 vs 5.6 cm), slightly lower because the hand moves along the path.

## DR4-1 02:56 reserve 4 skipped
- The ROS live path (g1_meff_run.sh) replays a fixed Kinect recording (NPZ hard-coded to ~/hrc_ws/pose_npz/human_state_1781524848.npz) in scene g1;
  replaying the R1 motion would need a new pose file plus a script change (scene g2 / front path, round trip). Plan: "only with existing scripts".
  Not done; listed under "needs your decision" only as an option for the Kinect stage.

## DF1 02:57 follow-up F1 (question first)
- Question: WP1 recommends P (--meff-motion cv) for the Kinect stage. Does P also help in the crossing task M2, where uniform came to 0.0-1.2 cm at
  0.5 m/s and ours waited more in front of the hand at 0.25 m/s?
- Expected: at 0.5 m/s P raises both conditions' minimum gaps (the hand crosses with a clear velocity); at 0.25 m/s the stalls stay because the hand
  rests inside the path (velocity 0 there, P = static).
- Runs: M2 0.5 and 0.25 m/s x uniform / ours x seeds 0-2 with P = 12 runs (limit 12).

## DF2 02:57 follow-up F2 (question first)
- Question: P extrapolates with the frame-to-frame velocity. Jitter makes that velocity noisy; does P then make things worse than static
  (WP5 measured jitter only with static)? And does R1 1.0 m/s P hold for a third seed?
- Expected: with 2 cm jitter the velocity noise is ~0.6 m/s per segment (2 cm x 30 Hz x sqrt 2), larger than the hand speed at 0.5 m/s; P could
  push shapes around and raise jerk; gaps may still be fine because the zone grows rather than shrinks on average.
- Runs: R1 0.5 m/s jit2+P, jit4+P, lag100+jit2+P x uniform / ours, seed 0 (6) + R1 1.0 m/s P seed 2 x uniform / ours (2) = 8 runs.

## DF3 03:26 follow-up F3 (question first)
- Question (from F2): does a simple low-pass filter on the human velocity make P safe under jitter?
- Change: replay option --human-vel-alpha a (exponential moving average of the delivered segment velocity, v_f = a v + (1 - a) v_f_prev, per new frame;
  0 = off, previous behaviour). Replay side only; the node itself is not changed. Baselines re-run for the bit check after the change.
- Expected: with a = 0.2 (time constant ~0.15 s at 30 Hz) the jitter velocity noise drops ~3x; uniform penetration disappears at 2 cm, maybe not at 4 cm;
  ours' jerk falls toward the no-jitter value. The fast-hand benefit of P may shrink (filter lag).
- Runs: jit2+P+F, jit4+P+F x uniform / ours (4) + R1 1.0 m/s P+F (no jitter) x uniform / ours (2) = 6 runs + 2 baseline bit-check runs.

## DF4 03:41 follow-up F4 (question first)
- Question: is the revised Kinect candidate (P + velocity filter) still better than static when lag and jitter come together, as they will with
  the live sensor (100 ms, 2 cm), and does it hold over 2 seeds at 0.5 m/s and at 1.0 m/s?
- Expected: at 0.5 m/s P+F >= static on min gap with jerk ~1.5x static; at 1.0 m/s P+F clearly better on the critical deficit (prediction offsets lag).
- Runs (12): R1 0.5 m/s lag100+jit2 {static, P+F} x uniform / ours x seeds 0, 1 (8); R1 1.0 m/s lag100+jit2 {static, P+F} x uniform / ours, seed 0 (4).

## D7-1 04:10 WP7 M1 camera
- The V2 camera (eye 0.58,-1.22,0.62) looks through the hand-over human, who stands between camera and robot in the P6_tipfront25 placement;
  the first M1 recording shows mostly the forearm. Re-recorded M1 with a camera on the other side (eye 0.50,1.15,0.80 -> target 0.50,-0.15,0.48),
  same for both conditions. Static P4 and M2 keep the V2 camera. Bit check repeated for the new M1 recordings.

## DF5 04:29 follow-up F5 (question first)
- Question (from F4): can the shared speed-cap settings tested in WP6 (c1 weight 1e6, c2 radius 0.5 m) bring the near-human speed p99 back under
  0.52 m/s for lag 100 ms + jitter 2 cm + P + filter at 1.0 m/s, without losing the gap?
- Expected: c1 lowers p99 by a few cm/s (WP6: -0.015 mean), probably not enough for 0.56-0.62; c2 may help more here because the hand is mispredicted
  by up to ~0.1 m (1 m/s x 100 ms), which a larger radius covers.
- Runs: R1 1.0 m/s seed 0, lag100+jit2+P+F with profile t2a_c1 and t2a_c2, uniform and ours = 4 runs.

## DF6 04:49 follow-up F6 (question first)
- Question (from F5, single seed): does the 0.5 m/s speed-cap radius (c2) systematically cost gap under lag 100 ms + jitter 2 cm + P + filter, or was
  that seed noise; and does c2 change anything at 0.5 m/s hand speed?
- Expected: at 1.0 m/s c2 keeps p99 <= 0.52 in seed 1 too; the gap loss is within the 2-3 cm seed spread. At 0.5 m/s c2 ~ t1 (p99 was already < 0.5).
- Runs (6): R1 1.0 m/s seed 1 lag100+jit2+P+F with t1 and with c2, uniform / ours (4); R1 0.5 m/s seed 0 lag100+jit2+P+F with c2, uniform / ours (2).

## D7-2 04:48 WP7 videos done early
- WP7 videos recorded and composed at 03:39-04:48 (scenes fixed by then). Frame and recording folders of tonight (rec 3.1 GB, frames 1.5 GB) deleted
  after ffprobe (645 / 645 / 255 frames, 1920x1080 h264) and the bit check (6 of 6 recordings bit-equal to the original runs).

## DF7 05:03 follow-up F7 (question first)
- Question: does the full Kinect candidate (P + velocity filter 0.2 + speed-cap radius 0.5 m) keep the R1 results on clean data (no lag, no jitter)?
- Expected: at 0.25 / 0.5 m/s the min gaps are equal or larger than R1 base, hold gaps unchanged, p99 < 0.5, jerk near base.
- Runs (8): R1 0.25 and 0.5 m/s, uniform / ours, seeds 0-1 with P+F+C2 (0.25 m/s via on_wp1.sh).

## DF8 05:15 follow-up F8 (question first)
- Question: does the full Kinect candidate also hold in the collaboration motions (M1 hand-over, M2 crossing) at 0.5 m/s?
- Expected: M1 like R1 (gaps up, no cost); M2 gaps up as in F1, waiting in front of the resting hand unchanged.
- Runs (8): M1, M2 0.5 m/s x uniform / ours x seeds 0-1 with P+F+C2.

## D1118-1 11-18 figure checks requested by the user (no new runs, existing logs only)
- WP2 thickness: the overnight ray thickness used the union of all right-arm shapes, so a ray from one part could continue in a neighbouring part's
  zone (P3 forearm top 37.3 cm = forearm ray running into the upper-arm zone). Changed to own-part thickness (hand / forearm / upper arm; elbow =
  forearm + upper arm). 14 of 63 cells change; P3 forearm top 37.3 -> 17.9 cm. WP3 pick unchanged (same set with own-part values).
  Old values kept: on_wp2_zone_overnight.json and the 'ours' fields of on_wp2_zone.json.
- WP3 pass minimum: P6_forearmside18 ours 11.2 cm in all 3 seeds was the start pose (t 0.02 s, TCP 0.2 cm from the start). Rule: pass-section
  minimum = steps with the TCP more than 0.10 m from the start and goal TCP (path 0.74 m). Other 8 scenes unaffected (identical at 5 / 10 / 15 cm).
  P6_forearmside18 ours 11.2 -> 12.1 cm mean (class unchanged). Old summary kept: on_wp3_summary_overnight.json.
- WP3 class rule: added 'small, uncertain' for 1 cm < |d| <= seed range s (the old rule called these 'no difference'). P2_top (d +1.4, s 2.6 cm)
  moves to 'small, uncertain (ours farther)'; no other scene changes.
- Same end-point scan over the 194 non-WP3 analysed runs: run minimum within 0.10 m of a path end in 5 runs: R3 M3 ours s0-s2 (8.6 / 7.8 / 9.0 ->
  pass section 10.4 / 8.0 / 9.5), F8 on4_M1_v050_gain_s1_P_F_C2 (9.9 -> 10.1), WP6 on6_P6_forearmside18_gain_s2_c1 (11.2 -> 11.8; the t1 run it was
  compared with was also the start pose, 11.2 -> 11.6, so the WP6 verdict does not change). Noted in REPORT (R3); other tables unchanged.
