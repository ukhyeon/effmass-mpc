# live_prep2 STATUS

- 2026-11-18 21:01:39 start; bundle 0 (cleanup)
- 21:02:20 Kinect not on USB (kernel: whole device disconnected 19:37:20, after run5); Kinect-dependent steps wait, others continue
- 21:03:03 bundle 0: committed (default part, LV_ARM removed, 10 files parked/handled); Kinect smoke test pending (no Kinect)
- 21:04:44 bundle 1: A (human 2.8x) pen 20 steps min -7.7 cm; B (real time, pr_live_run3_part) pen 7 steps min -1.0 cm -> variations on B
- 21:07:53 bundle 1: variation runs nocap/hr1 crashed (attribute name clash step_frames from my playback edit, fixed -> step_frame_map); rerun
- 21:11:59 bundle 1 done: A pen 20 steps -7.7 cm; B 7 steps -1.0 cm; B no-cv 15/-4.4, B no-cap 7/-8.0, B hand+1cm 2/-1.0. Residual B overlaps: hand moving toward the robot 0.39 m/s while the robot sphere already moves away 0.41-0.46 m/s (speed cap 0.5), and a tracking jump (hand 2.75 m/s). Conclusion: large overlap = slow display (human 2.8x); residual 1 cm not removed by one change.
- 21:18:22 bundle 2 started: options added to step2b (opt-in, defaults = previous): --render-every, --render-hz, --gui-viz-defer, --gui-res, --gui-hide-ui, --gui-aa. Measured with lv_live_run.sh sensing 0 (replayed person, no Kinect), GUI, VIEW of the user.
- 21:18:22 bundle 3 code: sensing wrapper LV_SENSING_SAVEVIS (raw colour, capture time stamp, writer thread), lv_live_kinect.sh moves frames to goal1/live/<tag>/color, lv_video_prep.py, step2b --playback-q/--human-step-frames/--viz-record-every, g1_viz --viz-eta-shell arm (+ gain-mode thickness), lv_video_compose.py, lv_make_video.sh

## bundle 1 result (run3 saved human, per-part reference, headless, live_t1, seed 0, frame live_frame.yml)
| run | human time | change | hand min gap (cm) | overlap steps | first overlap (t s, robot link, TCP y z) |
|---|---|---|---|---|---|
| lp2_run3A_part | 2.8x (as live, rate 83.3 Hz) | - | -7.66 | 20 | 24.2, leftfinger, 0.27 0.49 |
| pr_live_run3_part | real time (B) | - | -1.02 | 7 | 68.6, hand, 0.01 0.36 |
| lp2_run3B_static | real time | motion prediction off | -4.41 | 15 | 60.5, hand, 0.18 0.39 |
| lp2_run3B_nocap | real time | speed cap off | -8.03 | 7 | 74.7, hand, -0.07 0.56 |
| lp2_run3B_hr1 | real time | hand radius +1 cm | -0.98 | 2 | 76.7, leftfinger, 0.00 0.60 |
B residual overlaps: hand moving toward the robot 0.2-0.39 m/s while the nearest robot sphere already moves away 0.41-0.46 m/s
(speed-capped retreat, pos_err 0.38-0.40); hr1 overlap at a tracking jump (hand 2.75 m/s). Details bundle1_events.json.
Conclusion: the deep overlap (-7.7 cm, 20 steps) comes from the slow display (human 2.8x faster than simulation); at real time
a 1 cm overlap remains in 7 steps that no single change removes (prediction and speed cap both help; +1 cm radius 7 -> 2 steps).

## bundle 0 (Kinect back at ~21:50)
- lp2_smoke300: lv_live_kinect.sh, live_frame_user.yml, 300 steps, isaac exit 0, RTF 0.537, infeasible 0, colour frames 641 saved / 0 dropped.
  Not strictly without a person: someone sat at the back desk (sensing sent 193 human messages).

## bundle 2 (GUI speed; physics, MPC, cost, 60 Hz unchanged; VIEW 1.6,-2.4,1.3,0.75,-0.05,0.35)
A. without sensing (Kinect absent at the time), replayed person (lv_live_run.sh sensing 0), 150 s each:
| change (cumulative) | RTF |
|---|---|
| before (render every step, 1280x720) | 0.488 |
| render every 2nd step | 0.631 |
| + 960x540 | 0.627 |
| + hide UI | 0.642 |
| + anti-aliasing off | 0.667 |
| render every 3rd step (+ above) | 0.722 |
| + human capsules moved only before a redraw (defer) | 0.726 |
| wall-clock 15 Hz redraw + defer | 0.773 |
| wall-clock 10 Hz redraw + defer | 0.815 |
| every 4th step + defer | 0.813 |
| final: 10 Hz, 960x540, AA off, defer, UI kept | 0.832 (repeat with viewport shots 0.801) |
| same with UI hidden | 0.810 |
B. with sensing on (real Kinect; a person was in the room and tracked by sensing), replayed person for Isaac, 150 s each:
| setting | RTF | loop ms | solver ms | physics+render ms |
|---|---|---|---|---|
| before | 0.386 | 42.9 | 16.3 | 12.7 |
| final | 0.577 | 28.7 | 14.8 | 6.7 |
| final + colour saving (bundle 3) | 0.621 | 26.7 | 14.1 | 6.6 |
| 5 Hz redraw + saving (reference only) | 0.629 | 26.3 | 15.2 | 5.7 |
| live lv_live_kinect.sh lp2_rec20 (final + saving, person reaching) | 0.562 | 29.4 | 14.2 | 8.9 |
Final settings are lv_live_kinect.sh defaults (LV_RENDER_HZ=0 restores the previous display). 0.8 not reached with sensing:
the rest is the solver slowed by sensing on the same GPU (11.2 -> 14-15 ms) and physics/state in GUI mode (about +4 ms over
headless); 5 Hz redraw gives no more. Viewport shots gui_final_s600.png / gui_final_s1200.png: robot and human capsules shown.
Regression: lv_base.sh lp2 bit-equal (gk000, gain).

## bundle 3 (recording)
- colour: sensing's own colour image (packet bg_img, no overlay render) per frame, capture time burned in, writer thread,
  goal1/live/<tag>/color (vis_output/*.jpg, vis_times.csv). lp2_rec20: 1264 frames saved, 0 dropped, about 24 fps.
- lv_make_video.sh lp2_rec20: 1200 steps re-rendered (playback joints, human per step, zone shells on forearms and hands, VIEW of
  the run), mp4 35.4 s, panel time difference median 0.010 s, p95 0.030 s, max 0.045 s (limit 0.1 s); 2 min 15 s to make.
- Playback check (run5_ours): re-rendered joints vs recorded max 0.09 deg. Synthetic colour test on run5 (b3_synthetic_sync_run5.json):
  max 0.031 s; synthetic frames removed.
