# Benchmarks

This file tracks every metric and benchmark gate used to judge estimator
behavior. Unit tests still own pass/fail enforcement; this document records what
is measured, why it matters, and where the value comes from.

## Current Gates

| Metric | Source | Scenario | Current value / bound | Status |
|---|---|---|---|---|
| Synthetic IMU timestamp spacing | `tests/synthetic_test.cpp` | 200 Hz, 0.02 s | 5,000,000 ns between samples | Passing |
| Sample-grid retention | `tests/synthetic_test.cpp` | 200 Hz, 0.29 s (`0.29 * 200` is not exactly representable) | 59 samples, last at 290,000,000 ns | Passing |
| Sample-grid truncation | `tests/synthetic_test.cpp` | 200 Hz, 0.0071 s | 2 samples, last at 5,000,000 ns | Passing |
| Ground-truth sampling | `tests/synthetic_test.cpp` | 200 Hz, 0.02 s, non-zero start timestamp | 5 states matching `state_at` within `1e-9` | Passing |
| Cross-stream time origin | `tests/synthetic_test.cpp` | EuRoC-scale start timestamp, 200 Hz, 0.02 s | Ground-truth and IMU timestamps identical | Passing |
| Bias plumbing | `tests/synthetic_test.cpp` | Trajectory accel/gyro bias, stationary | Reported truth and IMU stream carry the same bias within `1e-9` | Passing |
| Invalid configuration rejection | `tests/synthetic_test.cpp` | Zero/negative/NaN rate, negative duration, negative density | Error returned, no abort | Passing |
| Stationary accelerometer convention | `tests/synthetic_test.cpp` | Identity pose, zero motion | `[0, 0, 9.81] m/s^2` | Passing |
| Stationary propagation error | `tests/synthetic_test.cpp` | 200 Hz, 1.0 s | Position, velocity, orientation unchanged within `1e-9` | Passing |
| Constant acceleration propagation error | `tests/synthetic_test.cpp` | 200 Hz, 1.0 s | Position and velocity match analytic truth within `1e-9` | Passing |
| Constant yaw propagation error | `tests/synthetic_test.cpp` | 200 Hz, 1.0 s | Orientation matches analytic truth within `1e-9` rad | Passing |
| Fixed-bias propagation error | `tests/synthetic_test.cpp` | 200 Hz, 1.0 s | Position and velocity norm error below `4e-3`; orientation below `1e-9` rad | Passing |
| Fully excited propagation error | `tests/synthetic_test.cpp` | 200 Hz, 2.0 s | Position and velocity norm error below `4e-3`; orientation below `1e-9` rad | Passing |
| Timestep convergence | `tests/synthetic_test.cpp` | Fully excited trajectory, 200/400/800 Hz | Error ratios must be below `0.75` | Passing |
| Stereo projection sanity | `tests/synthetic_test.cpp` | Landmark at `(0,0,5)`, `fx=500`, baseline `0.2 m` | Cam0 `(320,240)`, cam1 `(300,240)`, disparity `20 px` | Passing |
| Stereo visibility rejection | `tests/synthetic_test.cpp` | Behind-camera and out-of-frame landmarks | No observations emitted | Passing |
| Stereo disparity threshold | `tests/synthetic_test.cpp` | Threshold `0.5 px`, landmarks at 300 m and 100 m depth | Only the 100 m landmark survives, disparity `1.0 px` | Passing |
| Non-rectified rig rejection | `tests/synthetic_test.cpp` | Negative x baseline, y baseline, relatively rotated cam1 | Error returned rather than empty output | Passing |
| Seeded IMU noise reproducibility | `tests/synthetic_test.cpp` | Explicit IMU noise seed | Same seed reproduces samples; noise differs from noiseless output | Passing |
| Injected noise vs. calibration | `tests/synthetic_test.cpp` | 200 Hz, 100 s, densities `0.01` and `0.002` | Residual stddev within 5% of `density * sqrt(rate)` | Passing |
| Seeded pixel noise reproducibility | `tests/synthetic_test.cpp` | Explicit pixel noise seed | Same seed reproduces observations; noise differs from noiseless output | Passing |
| Stereo triangulation covariance | `tests/triangulation_test.cpp` | 0.2 m baseline, 0.1 px independent pixel noise | Symmetric PSD; principal axis aligns with viewing ray; Monte Carlo covariance within 3% | Passing |
| Stereo uncertainty range scaling | `tests/triangulation_test.cpp` | 5 m and 10 m depth | Depth standard deviation ratio 4; lateral standard deviation ratio 2 | Passing |
| Landmark augmentation integration | `tests/landmark_augmentation_test.cpp`, `tests/slam_integration_test.cpp` | Dense robot covariance, 5 landmarks, fully excited 200 Hz trajectory | Symmetric PSD active covariance; stable allocation through add/propagate/remove | Passing |
| Measurement model optical axis | `tests/measurement_model_test.cpp` | `T_WB = T_BS = I`, landmark on optical axis | Pixel equals principal point within `1e-12` | Passing |
| Measurement model ray-depth invariance | `tests/measurement_model_test.cpp` | Same ray at depths 1 m and 5 m | Identical pixel prediction within `1e-12` | Passing |
| Measurement model inverse-pose convention | `tests/measurement_model_test.cpp` | Nontrivial `R_WB` and `p_WB` | Body-frame landmark and pixel match hand-derived values within `1e-12` | Passing |
| Measurement model camera-baseline split | `tests/measurement_model_test.cpp` | Cam0 identity, cam1 positive x baseline | Cam0 `(320,240)`, cam1 `(300,240)` within `1e-12` | Passing |
| Measurement model boundary contract | `tests/measurement_model_test.cpp` | Behind-camera and out-of-frame landmarks | Prediction returned; visibility gating left outside `h(.)` | Passing |
| Measurement model image-border projection | `tests/measurement_model_test.cpp` | Landmarks on the exact image edges | Pixels equal `(640,480)` and `(0,0)` within `1e-12` | Passing |
| Measurement model nonpositive-depth contract | `tests/measurement_model_test.cpp` | Landmarks at `Z = 0` and `Z = -3` | Success returned; zero depth is non-finite, negative depth can be finite, caller must gate | Passing |
| Measurement model vs. synthetic harness | `tests/measurement_model_test.cpp` | 20 Hz, 1 s, accelerating and yawing trajectory, 5 landmarks, stereo rig | Every noiseless harness pixel reproduced within `1e-9` | Passing |
| EuRoC IMU-only smoke position error | `tests/propagation_test.cpp` | MH_01_easy, 1 s from first ground truth | `< 2.0 m` | Passing |
| EuRoC IMU-only smoke velocity error | `tests/propagation_test.cpp` | MH_01_easy, 1 s from first ground truth | `< 3.0 m/s` | Passing |
| EuRoC IMU-only smoke orientation error | `tests/propagation_test.cpp` | MH_01_easy, 1 s from first ground truth | `< 0.5 rad` | Passing |
| Propagation covariance validity | `tests/propagation_test.cpp`, `tests/synthetic_test.cpp` | Synthetic and EuRoC smoke propagation | Finite, symmetric, minimum eigenvalue above `-1e-8` or tighter test tolerance | Passing |
| Monte Carlo covariance consistency | `tests/propagation_test.cpp` | 6,000 samples, 200 steps at 5 ms, excited state, non-trivial `P0` | Max normalized deviation `< 0.15` | Passing |
| Covariance transition blocks | `tests/propagation_test.cpp` | Identity orientation, `dt = 0.1`, `P0 = I` | Hand-computed `Phi P Phi^T` blocks within `1e-9` | Passing |
| Process-noise block placement | `tests/propagation_test.cpp` | Densities `2, 3`, random walks `4, 5`, `dt = 0.1` | Velocity `0.4 I`, orientation `0.9 I`, accel bias `1.6 I`, gyro bias `2.5 I`; position block exactly zero | Passing |
| Timestep validation | `tests/propagation_test.cpp` | Negative, NaN, infinite, and zero `dt` | Error naming `timestep_seconds` for the first three; zero is an exact no-op | Passing |

## Recorded Measurements

These are measured values worth preserving because they explain current
tolerances. Recompute them when the propagation integrator or synthetic
trajectory changes.

Most rows are the values behind a gate in the table above. The drift-growth and
step-cost rows are not: they come from ad-hoc harnesses run against
`libekf_slam_parser`, and nothing enforces them. They are recorded because they
describe how the estimator behaves, not whether it is correct. Promote either to
a test before treating it as a regression gate.

| Date | Metric | Configuration | Value |
|---|---|---|---|
| 2026-08-13 | Fully excited timestep convergence | 200 Hz, 2.0 s, sinusoidal world acceleration | Combined position+velocity error `0.00137943` |
| 2026-08-13 | Fully excited timestep convergence | 400 Hz, 2.0 s, sinusoidal world acceleration | Combined position+velocity error `0.000689604`; ratio `0.499918` |
| 2026-08-13 | Fully excited timestep convergence | 800 Hz, 2.0 s, sinusoidal world acceleration | Combined position+velocity error `0.000344773`; ratio `0.499959` |
| 2026-08-13 | Sample-grid retention | 200 Hz, 0.29 s, fully excited trajectory | Combined error `0.000530228` with the final sample kept, against `0.00902549` when a plain `floor` dropped it — the discrepancy was sampling, not integration |
| 2026-08-13 | Injected noise discretization | 200 Hz, 100 s (20,001 samples), seed 3 | Accelerometer residual stddev `0.142245` against `0.01 * sqrt(200) = 0.141421`; gyroscope `0.0281825` against `0.002 * sqrt(200) = 0.0282843` |
| 2026-08-14 | Monte Carlo covariance deviation | 6,000 samples, 200 steps at 5 ms, seeds 1/2/3 | `0.039283`, `0.040966`, `0.030136` against the `0.15` gate; the floor is `Phi = I + F dt` truncation, not sampling |
| 2026-08-14 | EuRoC IMU-only smoke, measured | MH_01_easy, 1 s (200 IMU steps) from first ground truth, initialized from ground truth | Position `0.0173 m`, velocity `0.0216 m/s`, orientation `0.00120 rad`; minimum covariance eigenvalue `6.6e-8`. Gates are `2.0 m / 3.0 m/s / 0.5 rad`, so the run sits ~100x inside them |
| 2026-08-14 | EuRoC IMU-only drift growth | MH_01_easy, initialized from first ground truth, no updates | 1 s: `0.017 m`; 2 s: `0.053 m`; 5 s: `0.215 m`; 10 s: `0.364 m`; 20 s: `3.75 m`; 30 s: `19.33 m`. Reported position sigma `sqrt(trace(P_pp))` over the same horizons: `0.007`, `0.034`, `0.342`, `2.44`, `18.7`, `62.4 m` |
| 2026-08-14 | Propagation step cost | MH_01_easy, 6,001 sequential `propagate(...)` calls, `RelWithDebInfo`, Apple silicon dev machine | `1.84`–`1.91 us` per step, against a `5000 us` budget at 200 Hz. Not a Jetson number |
| 2026-08-14 | Landmark augmentation cost | 5 metric XYZ births, 0.2 m baseline, `RelWithDebInfo`, Apple silicon dev machine | `7,300 ns` per landmark in the integration test. Not a Jetson number |

## Future Estimator Metrics

### ATE

Absolute Trajectory Error measures global trajectory accuracy after aligning
estimated poses to ground truth.

Initial reporting fields:

| Field | Unit | Notes |
|---|---|---|
| RMSE | m | Primary headline value |
| mean | m | Useful for bias |
| median | m | Robust central tendency |
| max | m | Worst-case excursion |
| alignment | text | Record whether SE(3), Sim(3), yaw-only, or none |
| sequence | text | EuRoC sequence name or synthetic scenario name |

### RPE

Relative Pose Error measures local drift over fixed time or distance windows.

Initial reporting fields:

| Field | Unit | Notes |
|---|---|---|
| translation RMSE | m | Local translational drift |
| rotation RMSE | rad or deg | Local rotational drift |
| interval | s or m | Window length used for the comparison |
| sequence | text | EuRoC sequence name or synthetic scenario name |

### NEES

Normalized Estimation Error Squared measures whether the covariance is
statistically consistent with the actual state error.

Initial reporting fields:

| Field | Unit | Notes |
|---|---|---|
| NEES mean | dimensionless | Mean over evaluated timestamps |
| expected dimension | dimensionless | State subspace dimension used in the test |
| lower confidence bound | dimensionless | Chi-square interval lower bound |
| upper confidence bound | dimensionless | Chi-square interval upper bound |
| pass fraction | percent | Fraction of samples inside the confidence interval |
| state block | text | Pose, velocity, biases, landmarks, or full state |

## Reporting Policy

- Keep deterministic synthetic metrics separate from EuRoC metrics.
- Record units for every metric.
- Record scenario configuration when a value depends on timestep, duration,
  alignment, noise seed, or sequence.
- Treat synthetic gates as bug isolation tests and EuRoC metrics as realism
  checks.
- When a tolerance is empirical, record the measured value that justified it.
- Do not replace unit tests with this file; add or update tests first, then
  update this file with the metric name and current result.
