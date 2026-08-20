# MH_01 Prefix Measurement-Noise Calibration

**Branch:** `codex/r-calibration` (isolated worktree).

## Reusable scenario

The benchmark accepts `mh01_benchmark [frames] [pixel_sigma]`. This experiment
uses the deterministic first 100 stereo frames, 78 of which overlap ground
truth. It keeps the EuRoC-YAML-derived IMU process noise Q, frontend, gating
threshold, landmark budget, and initialization unchanged. Only the diagonal
per-pixel covariance changes: `R = sigma_px^2 I_4`.

This prefix is a fast regression/calibration scenario, not a substitute for a
full-sequence evaluation. Its ATE/RPE/NEES numbers must only be compared with
other 100-frame runs.

## Results

| Pixel sigma | ATE RMSE | Translation RPE | Mean 15-dof NEES | Applied / gated | Update ms/frame |
|---:|---:|---:|---:|---:|---:|
| 0.5 px | 0.04895 m | 0.05116 m/s | 1897.0 | 2762 / 555 | 2.77 |
| 1.0 px | 0.03553 m | 0.04084 m/s | 292.8 | 3222 / 270 | 3.27 |
| 1.5 px | 0.03851 m | 0.03360 m/s | 221.7 | 3311 / 191 | 3.35 |
| 2.0 px | 0.04713 m | 0.03647 m/s | 190.6 | 3372 / 154 | 3.65 |

Increasing sigma clearly lowers over-confidence, proving the original 0.5 px
model is too aggressive for the effective real frontend residual. It does not
fix NEES: even the largest tested sigma is still far above the expected
15-dof scale. The scalar model is standing in for correlated tracking error,
rectification/timing error, imperfect associations, and camera-model mismatch.

## Decision

Do **not** change the project default from 0.5 px yet. On this prefix, 1.0 px is
the best ATE point and 1.5 px is the best RPE point, while 2.0 px has the
smallest NEES but worse ATE. There is no single defensible optimum and choosing
one by NEES alone would be tuning to the metric.

The next calibration should log un-gated residual vectors and predicted
`HPH^T` over this same prefix, then estimate a full effective residual
covariance (including stereo correlation). It should be validated on a
disjoint MH_01 segment before an R change lands.

## Longer-prefix confirmation

The first 200 frames provide an independent check that the 100-frame result is
not a short-prefix artifact. All settings except scalar pixel sigma remain
fixed.

| Pixel sigma | ATE RMSE | Translation RPE | Mean 15-dof NEES | Applied / gated | Update ms/frame |
|---:|---:|---:|---:|---:|---:|
| 0.5 px | 0.15854 m | 0.06193 m/s | 3684.0 | 4246 / 820 | 2.16 |
| 1.0 px | 0.09483 m | 0.05172 m/s | 314.6 | 5089 / 352 | 2.68 |

The 1.0 px candidate again improves both trajectory error and NEES, but the
remaining NEES excess confirms that scalar R inflation is mitigation, not the
complete consistency solution.

## Temporal-correlation experiment

KLT observations of the same landmark in successive camera frames are not
independent, but the standard EKF update treats them as such. The benchmark's
third argument therefore performs an intentionally simple experiment: it keeps
the frontend and landmark lifecycle running every frame, but applies mapped
landmark updates only every Nth frame. This is a diagnostic, not yet a frontend
API policy.

All rows use the 200-frame prefix and `sigma_px = 1.0`.

| Update stride | ATE RMSE | Translation RPE | Rotation RPE | Mean NEES | Update ms/frame |
|---:|---:|---:|---:|---:|---:|
| 1 (every frame) | 0.09483 m | 0.05172 m/s | 0.01214 rad/s | 314.6 | 2.68 |
| 2 | 0.10601 m | 0.04542 m/s | 0.00930 rad/s | 266.3 | 1.32 |
| 3 | **0.08869 m** | **0.03961 m/s** | 0.00805 rad/s | 206.8 | 0.88 |
| 5 | 0.11740 m | 0.04068 m/s | **0.00679 rad/s** | **186.4** | 0.50 |

Stride three improves both accuracy and consistency over the every-frame
baseline while reducing update time by 67%. Stride five reduces NEES further but
loses position accuracy, so it is over-thinning for this prefix. This is strong
evidence that repeatedly assimilating correlated KLT tracks contributes to
over-confidence. It still does not reach the NEES target, so it must be paired
with improved residual covariance and frontend quality checks.

## Combined KLT, R, and correlation results

The corrected coarse-to-fine KLT implementation was applied to this isolated
benchmark branch, then the same 200-frame comparison was repeated. The KLT fix
substantially increases surviving tracks and therefore raises frontend/update
cost, but it improves the estimator much more than the cost increase.

| Configuration | ATE RMSE | Translation RPE | Rotation RPE | Mean NEES | Frontend ms/frame | Update ms/frame |
|---|---:|---:|---:|---:|---:|---:|
| Before KLT fix, 1.0 px, every frame | 0.09483 m | 0.05172 m/s | 0.01214 rad/s | 314.6 | 140.2 | 2.68 |
| KLT fix, 1.0 px, every frame | 0.02200 m | 0.01085 m/s | 0.00743 rad/s | 244.0 | 231.6 | 4.14 |
| KLT fix, 1.0 px, every 3rd frame | 0.02209 m | 0.01103 m/s | 0.00453 rad/s | 156.5 | 232.5 | 1.38 |
| KLT fix, 1.5 px, every 3rd frame | **0.01942 m** | 0.01147 m/s | **0.00412 rad/s** | **134.4** | 232.8 | 1.40 |

The combination improves ATE by 4.9x and NEES by 2.3x over the pre-fix 1.0 px
baseline. Update thinning retains essentially all position accuracy after the
KLT fix while reducing update cost by 67% and lowering NEES by 36%. The frontend
is now the runtime bottleneck: corrected KLT runs every pyramid level and
raises frontend cost to about 233 ms/frame. These are prefix results only and
must be validated on a later, disjoint segment before defaults change.

## Full-rate correlation-aware covariance experiment

Rather than skipping mapped observations, the experimental frontend records the
pixel and timestamp of each *accepted* track update. On the next observation it
uses `R_track = (1 + alpha * c^2) R_base`, where `c` decreases from one to zero
as either elapsed time reaches 150 ms or pixel parallax reaches the configured
decorrelation threshold. Gated observations do not reset this history. This is
O(1) track metadata and leaves every camera-frame update enabled.

All rows use the corrected KLT, the 200-frame prefix, `sigma_px = 1.5`, and no
update thinning.

| Inflation alpha | Decorrelation parallax | ATE RMSE | Mean NEES | Update ms/frame |
|---:|---:|---:|---:|---:|
| 0 (baseline) | — | 0.01956 m | 172.5 | 4.19 |
| 4 | 3 px | 0.01961 m | 171.9 | 4.19 |
| 4 | 10 px | 0.01963 m | 162.0 | 4.19 |
| 4 | 20 px | **0.01838 m** | **153.3** | 4.15 |

The original 3 px threshold was mostly inactive because corrected tracks often
move farther than 3 px per frame. At 20 px the model improves both ATE and NEES
without dropping observations or adding measurable hot-path cost. It remains a
prefix-tuned approximation and needs a disjoint-prefix validation before it
becomes a default.
