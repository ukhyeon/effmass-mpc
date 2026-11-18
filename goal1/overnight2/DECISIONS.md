# overnight2 decisions (question / cause class / options / choice and reason / result)

## D0-1 (before overnight2, compare_tip) path offset 3 cm gave negative gaps
- question: offset 3 cm beyond the fingertip gave human-cost-off gaps -4.6 / -6.1 / -6.3 cm; keep 3 cm?
- cause class: criterion not suitable (3 cm is a TCP-to-tip distance; the table gap is robot sphere surface + 0.02 buffer, the gripper body is above the TCP)
- options: (a) keep 3 cm; (b) calibrate the offset per motion so that the off gap is about 3 cm (as overnight WP3 placed postures for an off gap of 2.5 cm)
- choice: (b), the stated purpose is "about 3 cm when passing without human cost"
- result: offsets 0.115 / 0.205 / 0.15 m (static IK gap about 3 cm)

## D0-2 representative posture = median over the whole motion
- question: whole-motion median is 7-22 cm away from where the hand actually is most of the time
- cause class: criterion not suitable (the motion window includes raising / re-positioning; motion 3 changes pose at 13 s)
- options: (a) keep the whole median; (b) median over held frames (target within 4 cm of the middle-third median, perpendicular to the path), metrics on held frames; (c) shorter window by hand
- choice: (b), reproducible rule, still a median of the recorded posture, data unchanged
- result: held share 73 / 35 / 33 %

## D0-3 motion 1 and 3 paths too near the robot base, motion 2 out of reach
- question: one pass took 8-9 s and left the line by 6-15 cm (motion 1, 3); motion 2 far end without IK
- cause class: setting not suitable (virtual robot base position)
- options: (a) shorten the line; (b) height +-0.10 m; (c) move the virtual base
- choice: (c) after (a) and (b) did not give a two-sided pass (motion 2: only +-0.05 m)
- result: base shifts (-0.30, 0), (+0.10, +0.30), (-0.10, 0) m (Franka base frame = world, no rotation); copies of the frame file in compare_tip/

## D0-4 closest direction in the front pass was mostly "up"
- question: front-pass difference (+0.9 / +1.5 / +1.2 cm) is far below the static V2 front (+4.7 cm); is the measurement wrong?
- cause class: method limit (path definition), not a measurement error: the arm points 14-15 deg down and the gripper body (large spheres above the TCP) reaches the hand from above first, where the hand's effective mass is small (eta 6.5-7.4 cm at the closest moment)
- options: (a) report as is; (b) lower the path by up to 10 cm so that the fingers, not the gripper body, pass in front; (c) tilt the line with the arm axis
- choice: (a) within the bundle 0 budget; (b) noted as the next step (allowed height range), not run tonight
- result: table with the side of the closest pass; interpretation in compare_tip/REPORT.md

## D0-5 earlier compare/REPORT.md motion 3 numbers
- question: compare/REPORT.md interpretation table gives motion 3 "3.3 -> 3.8"; the arm-gap table gives 3.0 -> 3.4
- cause class: measurement error (my transcription used the hand column)
- options: (a) edit the earlier report; (b) leave the earlier folder unchanged (rule) and state the correction in compare_tip/REPORT.md and here
- choice: (b); the difference (+0.4 cm) is unchanged
- result: correction note in compare_tip/REPORT.md

## D3-1 bridging simulation: spline buffer dropped on-time packets
- question: spline5 showed accelerations up to 36x the limit on smooth commands
- cause class: execution error (my code): the on-time test arr <= t_k + Db - extra failed by float rounding for some packets, so the spline stopped at the last accepted knot
- options: (a) exact rational times; (b) a 1 us tolerance in the test
- choice: (b)
- result: rerun of all 288 cases

## D3-2 MPPI particles 300 not possible
- question: the solve-time study planned particles 300 / 200 and iterations 4 / 3
- cause class: method limit (cuRobo sample set layout: shape [1, 1, 285, 30, 7] invalid for 60060 values)
- options: (a) try other counts (e.g. 320); (b) keep 200 and the iteration variants
- choice: (b); 200 and iterations 3 / 4 already bracket the 1 % target
- result: table without 300
- follow-up: my first fix put the comment on the same line and commented out the knot definitions (NameError); moved the comment, reran; spline5 worst acceleration 1.12x (was 36x)

## D4-1 near stop held the robot even when it wanted to move away
- question: with the near rule on, overlap steps rose (run3 7 -> 305, live3m 0 -> 1938, min gap -11.5 cm) although the robot speed within 3 cm was ~0.03 m/s
- cause class: criterion not suitable: "stop below 3 cm" also blocked moves that increase the distance, and the recorded human does not react, so the arm kept moving into the stopped robot
- options: (a) keep the hard hold (robot never moves fast near the human, but cannot escape); (b) do not scale a command whose robot spheres are not closer to the human than now (retreat allowed); (c) scale only the velocity component toward the human
- choice: (b), simple and keeps "no fast approach", option --safety-allow-retreat (default 1 inside the new safety layer; 0 = the first version)
- result: see bundle 4 table (on = hold, on2 = retreat allowed)
