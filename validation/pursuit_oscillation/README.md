# Pure Pursuit trajectory comparison — 2026-10-15

Read-only capture: localization, local odometry, Gazebo GT, tracking path,
lookahead marker and three velocity command stages. No controller parameters changed.
Each capture lasts 35 seconds of wall time. before.json and mixed_manual.json
include user-confirmed manual operation. samples.json is the subsequent window after
requesting hands-off operation; exclusive autonomous input was not independently verified.

Latest capture:
- One latched nonempty tracking path received, no replacement observed.
- LIO vs GT yaw: interpolate GT to LIO timestamps, subtract constant median yaw
  offset; p95 absolute residual 0.181 degrees. This does not certify absolute map
  alignment, latency or translational accuracy.
- Raw controller commands zero: 79.4%; final /cmd_vel zero: 82.5% by received sample count.
- Raw angular command p05/p95: -0.387/+0.314 rad/s.
- Logs show cross_track_limit stops; current limit 0.4 m, lookahead 0.35 m.
- Body-to-nearest-path-point p50/p95 0.428/0.493 m is an IMU-origin diagnostic,
  NOT base_link cross-track error. Mount offset and segment projection are not included.

Conclusion: no observed repeated path replacement or large differential yaw noise.
Controller response, command interruptions and input arbitration need a controlled
hands-off trial. Short lookahead is a hypothesis, not established causality.
Do not relax cross-track safety bounds to hide the observed deviation.
Next experiment: same raw path and speed with a longer lookahead, measuring lateral
error and command reversals; confirm controller consumes navigation input exclusively.
