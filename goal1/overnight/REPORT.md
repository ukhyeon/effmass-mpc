# Overnight report (2026-11-17 to 2026-11-18)

(Front page is written at the end, in WP7.)

## WP0 Preparation (23:18-23:25)
- Deleted the 6 frame / recording folders (r1/rec, r1/frames_r1_main, r1/frames_r1_fast, v2/rec, v2/frames_v2_main, v2/frames_v2_gain_knob, about 5.1 GB) after
  ffprobe (r1_main/r1_fast 645 frames, v2_main 236, v2_gain_knob 172, all 1920x1080 h264) and the existing bit checks (r1_video_bitcheck.json, v2_video_bitcheck.json, all true).
- Local commit 2927641 "r1: moving hand replay" (R1 scripts, R1 summary json, R1 figures, .gitignore widened for R1 code, summary json, figures and overnight notes).
- Regression baseline: tonight's runs reproduce the V2 records bit for bit (uniform s0 and ours s0, 304 steps), and an R1 60 s round trip bit for bit (3600 steps).
  Joint commands do not depend on B1 logging or cost aggregation (V2 records had both on, tonight's runs have both off).
- Wall time: single pass 27.4 s, 60 s round trip 75.6 s. All planned run counts fit; seeds were not reduced (STATUS.md).

