# Static Initialization on MH_01

**Branch:** `codex/static-initialization` (isolated worktree).

## What was implemented

`initialize_static_imu(...)` estimates roll/pitch from mean stationary specific
force and gyroscope bias from mean angular rate. It initializes velocity and
accelerometer bias to zero. Because a static IMU cannot observe global yaw or
position, the MH_01 benchmark mode deliberately preserves ground-truth position
and heading only, isolating the effect of IMU-derived tilt and gyro bias in
raw-world metrics.

The initializer rejects a candidate interval unless it has at least 100 samples,
finite values, mean angular-rate norm at most 0.05 rad/s, and mean acceleration
magnitude within 0.2 m/s² of 9.81 m/s². These are validity checks, not tuning
knobs for a NEES result.

## MH_01 result

The first 100 MH_01 IMU samples fail the stationarity check:

| Quantity | Measured | Acceptance limit | Result |
|---|---:|---:|---|
| Mean angular-rate norm | 0.1327 rad/s | 0.05 rad/s | Rejected |

The raw samples also show changing angular rate and acceleration magnitude below
gravity. The sequence begins in motion, so it does not offer a valid static
prefix. An earlier permissive run treated the interval as static and eventually
failed at camera observation 2,977 because its innovation covariance was not
positive definite. That run is rejected as invalid evidence.

Therefore there is no legitimate initialized MH_01 accuracy/NEES/performance
number yet. Reporting a full result would turn an invalid prior into a benchmark
claim. The normal MH_01 benchmark remains the existing truth-initialized
baseline: ATE 0.696915 m, RPE translation 0.0484806 m/s, rotation
0.00552155 rad/s, mean NEES 217,927, and approximately 113.3 ms/frame.

## Performance impact

A valid static start runs once before the first frame and is O(N) over 100 IMU
samples. It adds no work to the 200 Hz propagation or 20 Hz camera-update hot
paths. On this MH_01 sequence it exits before filtering because the prerequisite
is not met, so there is no meaningful end-to-end timing comparison to report.

## Next step

Use a short visual-inertial alignment that supports motion, or a dataset/launch
configuration with a verified stationary prefix. The static initializer remains
useful where a vehicle is known to be resting at startup; it is not applicable
to MH_01 as currently packaged.

