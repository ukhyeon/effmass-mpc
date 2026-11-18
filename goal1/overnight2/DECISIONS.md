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
