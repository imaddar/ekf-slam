# Benchmarks

This file tracks every metric and benchmark gate used to judge estimator
behavior. Unit tests still own pass/fail enforcement; this document records what
is measured, why it matters, and where the value comes from.

## Current Gates

| Metric | Source | Scenario | Current value / bound | Status |
|---|---|---|---|---|
| Real MH_01 frontend smoke run | `tests/euroc_frontend_test.cpp` | First 30 stereo frames (1.45 s); truth-initialized, 100-landmark budget | Peak 377 tracks; 356 augmentations; 317 updates applied; 480 gated; final position error `5.81 mm` | Passing smoke test; not an ATE result |
| Real MH_01 frontend preliminary run | `tests/euroc_frontend_test.cpp` | Full MH_01, 3,682 stereo frames; truth-initialized, 100-landmark budget | Peak 233 tracks; 24,103 augmentations; 187,726 applied and 57,964 gated updates; final position error `2.02 m`; frontend `101.2 ms/frame`, update `5.71 ms/frame` | Completes stably; misses 20 Hz budget and needs tracking/noise tuning |
| MH_01 pixel-noise sensitivity | `tests/euroc_frontend_test.cpp` | First 100 frames; independent scalar detector-noise sweep | `sigma=0.5 px`: 1,190 gated / 2,423 applied, `2.87 cm`; `sigma=1.0 px`: 758 gated / 3,116 applied, `4.94 cm` | Neither is a calibrated model; do not tune by gate count alone |
| MH_01 full trajectory benchmark | `mh01_benchmark.cpp` | Full 3,682-frame filter pass; 3,638 camera timestamps shared with ground truth; truth-initialized, `sigma=0.5 px`, 100-landmark budget | ATE position RMSE `0.696915 m`; 1 s RPE translation RMSE `0.0484806 m/s`; rotation RMSE `0.00552155 rad/s`; mean 15-dof NEES `217,927` | Completes, but covariance is severely over-confident; not a consistency pass |
| MH_01 frontend stage profile | `mh01_benchmark.cpp` | Same full pass; per-stage wall clock from `FeatureFrontend::stage_timings()` | Frontend `110.7 ms/frame` against a `50 ms` budget at 20 Hz; `detect` `59.1 ms` (53.4%), `stereo_new` `29.6 ms` (26.8%), whole temporal tracking chain `19.3 ms` (17.5%) | Misses the 20 Hz budget by ~2.2x; cost is concentrated in feature acquisition |

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
| Stereo measurement Jacobian vs. central differences | `tests/measurement_update_test.cpp` | Nontrivial pose, landmark at 6 m, right/local rotation perturbation | Analytic blocks match within `1e-5`; a left/global perturbation mismatches by `> 1e-3` | Passing |
| Sequential update vs. batch oracle | `tests/measurement_update_test.cpp` | 3 and 5 landmarks, dense robot/landmark correlation | Error state and covariance agree within `1e-12` | Passing |
| Sweep order invariance | `tests/measurement_update_test.cpp` | All 6 permutations of a 3-landmark frame | States agree within `1e-10` | Passing |
| Joseph vs. simple covariance form | `tests/measurement_update_test.cpp` | 5-landmark frame | Agree within `1e-10`, as required under the optimal gain | Passing |
| Update covariance validity | `tests/measurement_update_test.cpp` | 5-landmark sweep | Symmetric within `1e-14`; positive definite; Loewner decrease within `1e-12` after undoing the reset | Passing |
| Reset Jacobian vs. numerical differentiation | `tests/slam_state_test.cpp` | `delta theta = [0.10, -0.14, 0.09]` | Matches within `1e-8`; changes `P` by `> 1e-3`, so the test has power | Passing |
| Outlier rejection leaves no trace | `tests/measurement_update_test.cpp` | 5 landmarks, one displaced 50 px | Gated; remaining result matches a run with that observation removed within `1e-12` | Passing |
| Closed-loop drift reduction | `tests/slam_integration_test.cpp` | 2.0 s, 200 Hz IMU, 20 Hz stereo, 5 landmarks, seed 11 | Updated position error strictly below the propagation-only baseline | Passing |
| Closed-loop landmark accuracy | `tests/slam_integration_test.cpp` | 2.0 s, seed 23 | Mean landmark position error `< 0.5 m` | Passing |
| Propagation-only Monte Carlo NEES | `tests/slam_integration_test.cpp` | 50 runs, 15 dof, initial error drawn from the initial covariance | Inside `[13.52, 16.56]` | Passing |
| Updated-filter Monte Carlo NEES | `tests/slam_integration_test.cpp` | 50 runs, 15 dof, same scenario with camera updates | Regression ceiling `< 30`; the consistency bound `16.56` is **not** met | Passing as a ceiling, failing as consistency |

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
| 2026-08-15 | Closed-loop position error | Synthetic 2.0 s, 200 Hz IMU + 20 Hz stereo, 5 landmarks, accel bias `[0.05, -0.03, 0.02]`, 0.5 px noise, seed 11 | Propagation-only `0.163 m`; with camera updates `0.0545 m`, a 3.0x reduction. 195 observations applied, 5 gated |
| 2026-08-15 | Closed-loop landmark error | Same scenario, seed 23 | Mean landmark position error `0.0703 m` against a `0.5 m` gate |
| 2026-08-15 | Monte Carlo NEES, propagation only | 50 runs, 15 dof, 2.0 s, initial error drawn from the initial covariance | `13.56` against the `[13.52, 16.56]` 95% interval. Consistent |
| 2026-08-15 | Monte Carlo NEES, with camera updates | Same 50 runs and scenario | `24.23`, outside the interval. Per-block against an expected `3.0`: position `2.73`, velocity `4.05`, orientation `6.35`, accel bias `7.16`, gyro bias `4.81` |
| 2026-08-15 | Sequential update cost | 5 landmarks in state, ~5 stereo observations per frame, `RelWithDebInfo`, Apple silicon dev machine | `16,225 ns` per frame against a `50,000 us` budget at 20 Hz. Small-`N` number; the `O(m n^2)` term does not dominate yet. Not a Jetson number |

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

#### Current NEES finding

The camera update is measurably over-confident, and the cause has been isolated
by experiment rather than asserted. Propagation alone sits at `13.56` inside the
`[13.52, 16.56]` interval; adding camera updates moves the same scenario to
`24.23`.

Per-block, against expected `3.0` each, plus the orientation error split along
gravity (yaw, 1 dof) and across it (tilt, 2 dof):

| Block | Yaw-dominant trajectory | Roll/pitch excited |
|---|---|---|
| Position | `2.73` | `2.47` |
| Velocity | `4.05` | `5.11` |
| Orientation | `6.35` | `6.47` |
| Accelerometer bias | `7.16` | `3.88` |
| Gyroscope bias | `4.81` | `3.70` |
| Yaw (expected `1.0`) | `1.41` | `1.15` |
| Tilt (expected `2.0`) | `4.81` | `5.23` |
| **Total** | **`24.23`** | **`21.44`** |

Four experiments narrow this down.

**It is not the sweep, the Jacobians, or the covariance container.** The
sequential update is asserted equal to a dense batch update within `1e-12`, the
Jacobians match central differences with a test that fails under the wrong
rotation convention, and the propagation-only control passes in the same
harness.

**It is not gate-induced selection bias.** Widening the chi-square threshold
from `9.4877` to `13.2767` to effectively infinite moves NEES only
`24.23 -> 24.03 -> 23.66`.

**It is not accumulation over updates.** NEES is already `20.99` after `0.5 s`
and plateaus: `21.95` at `1 s`, `24.23` at `2 s`, `23.52` at `4 s`. Spurious
information accumulating over many updates would grow monotonically.

**It is not the unobservable yaw direction.** This is the result that overturns
the obvious hypothesis. Yaw NEES is `1.41` against an expected `1.0`, and
improves to `1.15` with more excitation. The inconsistency lives in *tilt*
(`4.81` against `2.0`), which is the observable part of the orientation, pinned
by gravity. The classic Huang/Mourikis unobservable-subspace effect is not the
dominant term here.

**It is second-order linearization error.** Shrinking the initial tilt error
while holding everything else fixed:

| Initial tilt sigma | Total NEES | Tilt NEES (expected `2.0`) | Yaw NEES (expected `1.0`) |
|---|---|---|---|
| `0.01 rad` | `24.23` | `4.81` | `1.41` |
| `0.003 rad` | `17.24` | `2.65` | `0.89` |
| `0.001 rad` | `15.92` | `1.86` | `0.97` |

At `0.001 rad` the filter is *consistent*: `15.92` sits inside `[13.52, 16.56]`.
Converging to consistency as the operating point approaches truth is the
signature of the EKF's first-order approximation, not of a defect. It
concentrates in tilt because rotation is the only state that enters the
measurement nonlinearly -- position and landmark position enter exactly linearly
for a fixed `R`.

Two consequences for what to fix:

- **An iterated EKF is the targeted fix**, not first-estimates Jacobians. FEJ and
  OC-EKF address spurious information in the unobservable directions, and yaw is
  already consistent here. Relinearizing toward the posterior is what attacks a
  second-order error.
- **The scenario is pessimistic.** A `0.01 rad` (`0.57 deg`) initial tilt error
  is worse than a real system starts from; static-start gravity alignment from a
  stationary accelerometer reaches `~0.001 rad`, which is the regime where this
  filter is already consistent. Initialization is tracked as its own open
  decision in `ARCHITECTURE.md`.

Separately, the accelerometer-bias over-confidence (`7.16`) was mostly a
under-excited test trajectory rather than a filter property: adding roll/pitch
excitation drops it to `3.88` without touching tilt. Accelerometer bias and tilt
are confounded over short windows because a tilt error of `dtheta` mimics a
horizontal acceleration of `g * dtheta`, and yaw rotation does not break that
degeneracy.

`SlamClosedLoopTest.DISABLED_MonteCarloRobotNeesMeetsTheConsistencyTarget` is the
acceptance test. The live test asserts a regression ceiling of `30` so the number
cannot silently worsen. Loosening that ceiling to make a change pass would be
hiding a defect; tightening it to `16.56` is the goal.
`SlamClosedLoopTest.DISABLED_NeesDiagnosticSweep` reproduces the horizon and gate
experiments above.

## MH_01_easy Results Writeup

`mh01_benchmark` runs the complete image/IMU stream and evaluates only the
3,638 camera timestamps overlapping EuRoC ground truth; all 3,682 frames still
reach the filter. ATE is raw world-frame position RMSE, deliberately without
trajectory alignment because the run is initialized in EuRoC's world frame and
alignment would hide accumulated drift. RPE compares relative poses over the
nearest later camera sample at least one second away and reports rate-normalized
translation and rotation RMSE. NEES uses the posterior 15-state robot covariance
and the project error-state convention.

The `0.697 m` ATE and `4.85 cm/s` translational RPE show a working offline VIO
loop on real images, but the `217,927` mean NEES is orders of magnitude above
the 15-dof expectation. This is a calibration/consistency failure, not an
accuracy victory: the filter reports uncertainty far smaller than its actual
state error. The `110.7 ms/frame` frontend also exceeds EuRoC's 50 ms camera
period. The next work is frontend optimization and noise/linearization
consistency work before presenting MH_01 as a reliable estimator.

The earlier `113.3 ms/frame` figure divided by the 3,638 ground-truth-overlapping
frames rather than the 3,682 frames actually processed, so it overstated
per-frame cost by about 1%. Timing now divides by processed frames. The
remainder of the difference is run-to-run spread. Where that time goes is broken
down in the frontend stage profile below.

## MH_01 Frontend Stage Profile

`FeatureFrontend` accumulates wall-clock totals per stage and `mh01_benchmark`
prints them, so frontend cost is attributable without an external profiler.
Timer overhead is roughly 20 us per frame against a 110 ms frame, and the run
reproduces the recorded ATE and NEES exactly, so the instrumentation is not
perturbing either timing or estimates.

Full MH_01, 3,682 frames, `RelWithDebInfo`, single-threaded, Apple silicon dev
machine. PNG decode is `4.57 ms/frame` and sits outside `process`.

| Stage | ms/frame | Share | Calls/frame | us/call |
|---|---|---|---|---|
| `detect` | 59.14 | 53.4% | 1 | 59,137 |
| `stereo_new` | 29.63 | 26.8% | 196.7 | 151 |
| `temporal_klt` | 9.39 | 8.5% | 110.7 | 85 |
| `stereo_tracked` | 6.43 | 5.8% | <= 110.7 | >= 58 |
| `forward_backward` | 3.49 | 3.2% | 110.7 | 32 |
| `pyramid` | 1.77 | 1.6% | 2 | 885 |
| `rectify` | 0.78 | 0.7% | 2 | 392 |
| `bookkeeping` | 0.04 | 0.0% | — | — |
| **total** | **110.67** | | | |

Two facts dominate:

- **Feature acquisition is 80% of the frontend; tracking is 17%.** `detect` plus
  `stereo_new` is `88.8 ms/frame`. The full temporal chain -- KLT, the
  forward-backward check, and the primed stereo match -- is `19.3 ms/frame`.
  Optimizing the tracker is close to irrelevant until acquisition is fixed.
- **`detect` is a fixed cost that ignores demand.** It is `59.1 ms/frame`
  whether or not features are needed, because `detect_corners` scans every pixel
  and rebuilds a 7x7 structure tensor per pixel, recomputing each gradient about
  49 times across overlapping windows. It also runs every frame against a
  100-landmark budget while emitting 196.7 corners, so most of `stereo_new` is
  matching features the filter has no room for.

Per call, an unprimed stereo match costs `151 us` against `>= 58 us` for one
primed by the previous frame's disparity. The ratio is modest; the volume is
not. Read the two together: the acquisition path is expensive because it runs at
full image scale on every frame regardless of need, not because any single
search is pathological.

Run-to-run spread on the same binary is about 5% (`105.3` and `110.7 ms/frame`
on two consecutive full runs), so treat differences below that as noise.

## Frontend Optimization Experiments (300-frame MH_01 prefix)

Measured on the first 300 frames via `mh01_benchmark 300`, so accuracy figures
are a truncated-prefix comparison against each other, not against the
full-sequence rows above. Same machine and build as the stage profile.

| Change | ATE (m) | RPE trans (m/s) | Mean NEES | Applied | Frontend ms/frame |
|---|---|---|---|---|---|
| Baseline | 0.17503 | 0.06595 | 3,583 | 4,458 | 141.4 |
| + O(1)-per-pixel detector | 0.17503 | 0.06595 | 3,583 | 4,458 | 99.5 |
| + KLT coarse-to-fine fix | 0.01908 | 0.00927 | 324 | 11,937 | 189.5 |

**Detector rewrite is behavior-preserving.** Gradients are computed once per
pixel rather than once per window tap, the box sum is separable, and per-cell
bounded selection replaces a full sort over roughly 250k candidates.
`tests/corner_detector_test.cpp` asserts identical corner positions against a
reference transcription of the original loop, and every accuracy figure above is
unchanged. `detect` fell from `57.2` to `16.6 ms/frame`.

The sort was the larger half of that cost, not the structure tensor: about 70%
of all pixels clear `min_eigenvalue = 1e-3`, so ~250k candidates were fully
sorted each frame to select at most 288. Selection now bounds work per grid
cell and widens only when a truncated cell ends under its cap, which keeps the
result identical to a full sort by construction.

**The KLT fix trades runtime for a large accuracy gain.** `track_feature`
returned from the whole function on convergence at any pyramid level instead of
refining at the next finer one, so a measured 90% of calls returned a
coarsest-level estimate scaled up by 8 (`L0 conv=191, L1 388, L2 1096, L3
20933`). Coarse estimates failed the `0.5 px` forward-backward check, tracks
died early, and the detector re-acquired features every frame. Correcting it
improves prefix ATE by 9x and NEES by 11x, and applies 2.7x more measurements,
at 2x the frontend cost because every call now runs all four levels.

Full-sequence numbers in the rows above predate the KLT fix and need a re-run
before they can be compared to anything measured after it.

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
