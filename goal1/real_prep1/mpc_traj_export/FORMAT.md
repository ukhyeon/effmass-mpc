# mpc_traj_export: MPC targets and horizon plans, one npz per run

Each file `<setting>/<setting>_<scene>.npz` holds one simulated run (np.savez_compressed; load with `numpy.load`). Only robot joint values are stored, no human data.

- **setting:** `acc10` = MPC joint acceleration limit 10 rad/s^2 (profile `goal1/config/real_acc10.yml`), `slowc_acc10` = low-speed profile with the same acceleration limit (`goal1/config/real_slowc_acc10.yml`). 24 + 2 files.
- **scene:** `m1b` = path in front of the extended fingertip, `m3b` = path in front of the elbow (live3m corrected path). `AB` / `BA` = start end of the path. `uni` = uniform avoidance, `part` = effective-mass avoidance. `s0`-`s2` = MPPI seed.

## Time base

- One MPC step per physics step of the simulator, dt = 1/60 s.
- `t_step[i]` = (step + 1) / 60 s is the simulated time since the control loop started. This is the time base.
- The simulator ran slower than real time (realtime factor about 0.5-0.8 for these runs). Wall time is in `t_wall_s` / `t_wall_ns` and is not the time base.
- Horizon point k of step i is planned for the time `t_step[i] + horizon_t[k]`.

## Joints and units

- `joint_names` = `fr3_joint1` ... `fr3_joint7` (FR3 joint order). The model in the simulator calls them `fr3v2_1_joint1..7` (`model_joint_names`); the order is the same.
- Angles in rad, velocities in rad/s, accelerations in rad/s^2, times in s.

## Keys

N = number of MPC steps in the run, H = 30 horizon points, J = 7 joints. Arrays are float32 unless noted.

| key | shape | meaning |
|---|---|---|
| `step` | (N,) int32 | MPC step index (0 = first step) |
| `t_step` | (N,) float64 | (step + 1) / 60, s |
| `t_wall_s` | (N,) float64 | wall time of the step from the first step, s |
| `t_wall_ns` | (N,) int64 | wall clock of the step (time.time_ns) |
| `target_q`, `target_qd`, `target_qdd` | (N, J) | the target the MPC exported at this step: position, velocity, acceleration (cuRobo `js_action`) |
| `horizon_q`, `horizon_qd`, `horizon_qdd` | (N, H, J) | the whole plan of this step: position, velocity, acceleration at the H horizon points |
| `horizon_t` | (H,) | time of each horizon point from the step, s (the same for every step) |
| `plan_start_q`, `plan_start_qd` | (N, J) | start state the plan was integrated from (the solver's filtered state) |
| `sent_q` | (N, J) | position target actually sent to the simulator's joint controller. Equals `target_q` unless the speed limiter scaled the step (`limiter_scale` < 1) |
| `limiter_scale` | (N,) | 1 = command not scaled; < 1 only with the low-speed profiles |
| `meas_q`, `meas_qd` | (N, J) | measured joint position / velocity at the start of the step (simulator) |
| `feasible` | (N,) int8 | 1 = solver result used; 0 = the step held the measured position |
| `first_point_resid` | (N,) | max abs(horizon_q[:, 0] - target_q), rad (check, should be 0) |
| `setting`, `scene`, `source_tag`, `robot_cfg`, `sphere_speed_limit` | scalar | run metadata |

## How the plan is built

- cuRobo's MPPI optimises 30 joint accelerations; that is its action sequence. The plan is the sequence of the best rollout, integrated with semi-implicit Euler (v_k = v_{k-1} + a_k dt_k, q_k = q_{k-1} + v_k dt_k) from the filtered start state, with the same dt_k as the solver.
- Point 0 of the plan is the exported target (`first_point_resid` = 0).
- The start state is the filtered state cuRobo plans from: the previous command blended with the measurement (filter coefficients 0.1). It is not the measured state; the difference was about 1-2 cm-rad in these runs (`plan_start_q` vs `meas_q`).
- `horizon_t`: the first 15 points are 1/60 s apart (0.0167 ... 0.25 s), then the spacing grows linearly to 0.04 s; the last point is at 0.675 s.

## The same runs in the earlier CSV format

`../export_mpc_cmd/<setting>/<setting>_<scene>.csv` has the columns of `goal1/export_mpc_cmd/`: step, t_s, q_cmd1..7 (= `sent_q`), q_meas1..7, feasible.

## Low-speed profile: what is limited

In `slowc_acc10` the hard speed limiter acts only on the position target sent to the simulator (`sent_q`). `target_q/qd/qdd` and the `horizon_*` arrays are the MPC plan before the limiter. With the speed cost of this profile the plan itself mostly stays near 0.2 m/s, but single horizon points reach 0.28-0.30 m/s of robot-sphere speed. A receiver that follows the horizon points (JointTrajectory with K > 1) is therefore not held to 0.25 m/s by this profile; only `sent_q` is.
