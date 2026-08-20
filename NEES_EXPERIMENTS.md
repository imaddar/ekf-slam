# NEES Reduction Experiments

**Branch:** `codex/nees-experiments` (isolated worktree).

## Method and baseline

The 50-run synthetic closed loop is unchanged: 2 s, 200 Hz IMU, 20 Hz stereo,
five landmarks, 0.5 px generated pixel noise, and initial error sampled from
the covariance supplied to the filter. Robot NEES has 15 dof and a 95% mean
interval of `[13.52, 16.56]`. Timings are wall-clock time around the camera
update, averaged over all update frames; compare ratios across configurations,
not absolute nanoseconds across machines.

| Configuration | Mean NEES | Position error (m) | Update ns/frame | Verdict |
|---|---:|---:|---:|---|
| Propagation only | 13.5576 | 0.21749 | 0 | Consistent control |
| Standard EKF | 24.2257 | 0.03410 | 16,229 | Over-confident baseline |

The control isolates the fault to camera updates. Opening the innovation gate
only reaches 23.66, so selection bias is not the cause. The excess is tilt,
not yaw; FEJ/OC-EKF is therefore not the first candidate.

## Candidate 1: iterated EKF

`UpdateOptions::max_iterations` is an opt-in prototype. Each iteration
relinearizes all previously accepted observations around a temporary retracted
state. Every covariance sweep starts from the same frame prior and only the
final sweep is committed, so the pixels are not assimilated repeatedly.

| Iterations | Mean NEES | Update ns/frame | Cost |
|---:|---:|---:|---:|
| 1 | 24.2257 | 16,229 | 1.00x |
| 2 | 22.9574 | 29,482 | 1.82x |
| 3 | 22.7277 | 42,745 | 2.63x |
| 5 | 23.5813 | 69,856 | 4.30x |
| 8 | 24.8138 | 108,651 | 6.70x |

Three iterations deliver the best result, but still miss the bound by 37% and
cost 2.63x. More iterations worsen NEES, showing this undamped fixed-point form
is not reliably convergent. Keep it as a research prototype, not a merge
candidate. A future version needs damping/convergence tests and a timing sweep
at the intended landmark budget.

## Candidate 2: matched initialization quality

Generated orientation error and the filter's initial covariance changed together,
so this remains a valid NEES experiment.

| Initial orientation sigma | Mean NEES | Position error (m) | Runtime |
|---:|---:|---:|---|
| 0.010 rad (0.57 deg) | 24.2257 | 0.03410 | baseline |
| 0.003 rad (0.17 deg) | 17.2352 | 0.03172 | unchanged |
| 0.001 rad (0.057 deg) | 15.9176 | 0.03180 | unchanged; passes |

This is the strongest result: a static-start gravity/bias initializer should
target roughly 0.001 rad tilt before normal camera updates. It is a real system
improvement with no camera-update cost, but needs validation using an estimator
that achieves this accuracy rather than an assumed initial condition.

## Candidate 3: assumed pixel-noise inflation

Pixels still contain 0.5 px standard deviation; only the filter's `R` changed.
This is a sensitivity study, not calibration.

| `R` scale | Mean NEES | Position error (m) | Runtime | Meaning |
|---:|---:|---:|---|---|
| 1.0 | 24.2257 | 0.03410 | baseline | nominal model |
| 1.5 | 18.3500 | 0.03372 | unchanged | still over-confident |
| 2.0 | 15.5757 | 0.03374 | unchanged | passes, but mismatched |
| 3.0 | 13.0403 | 0.03390 | unchanged | under-confident |
| 5.0 | 11.1918 | 0.03460 | unchanged | substantially under-confident |

Do not commit 2x `R` as a fix: in this synthetic setup it hides unmodelled
error with a false likelihood. It does motivate a EuRoC innovation-covariance
study after rectification, KLT, and stereo matching. If that supports an
effective 0.71 px standard deviation, the larger `R` is calibration; otherwise
it is tuning to pass NEES.

## Recommendation

1. Implement and validate static-start tilt/bias initialization first.
2. Preserve the iterated-update prototype only for further research.
3. Calibrate real residual covariance before changing `R`; never tune it solely
   to meet the confidence interval.

## Reproduction

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/homebrew
cmake --build build --target slam_integration_tests
./build/slam_integration_tests --gtest_also_run_disabled_tests \
  --gtest_filter='SlamClosedLoopTest.DISABLED_NeesDiagnosticSweep'
```

