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
