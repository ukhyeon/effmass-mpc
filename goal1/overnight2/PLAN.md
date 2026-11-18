# overnight2 plan (start 2026-11-18 23:28, end 2026-11-19 10:00; no new runs after 09:30)

| bundle | work | budget | target end |
|---|---|---|---|
| 0 | finish compare_tip (live3m, path in front of fingertip / elbow): 18 comparison runs, table side approach vs tip pass | 0.5 h | 00:00 |
| 1 | re-render with translucent SMPL mesh; 3-panel clips per motion for tip pass and side approach; zone shape figures x3; 2 GIFs; real-time video (lv_make_video.sh live3m); size limits 20 MB / 10 MB | 2.5 h | 02:30 |
| 2 | goal1/public_draft: table for every file in analysis/, remakes, hand/elbow analysis text, changed-numbers list, README draft, pre-push check (no push) | 3 h | 05:30 |
| 3 | (a) MPC solve time distribution (replay, cost aggregation off; with sensing if Kinect present), (b) 60 -> 1000 Hz bridging simulation (delays, jitter, drops; hold / linear / low-pass / smooth), FR3 limits from the official page | 2 h | 07:30 |
| 4 | safety options (default off, bit-equal when off): loss -> stop in place, near -> slow from 12 cm, stop at 3 cm; tests on lp2 first live test and live3m | 1.5 h | 09:00 |
| 5 | surface-point effective mass vs segment COM (calculation only) | 0.5 h | 09:30 |
| - | goal1/overnight2/REPORT.md | 0.5 h | 10:00 |

Rules: date at each bundle start; +30 min over budget -> wrap up and move on; Isaac <= 10 min per run, one at a time, check before each run;
decisions in DECISIONS.md (question / cause class / options / choice and reason / result); commit after each bundle; no push.
