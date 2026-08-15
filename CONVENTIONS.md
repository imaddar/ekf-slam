# ESEKF-VIO Conventions Reference

This is the single source of truth for mathematical conventions that must stay
consistent across propagation, stereo geometry, measurement prediction,
triangulation, landmark augmentation, and future update/marginalization code.
These are exactly the places where a change can compile, run, and still be
silently wrong if two modules disagree.

When code and this document disagree, fix the smaller surface area: usually that
means updating the new derivation before touching verified code. If the code is
wrong, update the code and this document in the same change.

## 1. Error-State Ordering

The robot error state is 15-dimensional:

```text
kPositionIndex          = 0   delta p_W
kVelocityIndex          = 3   delta v_W
kOrientationIndex       = 6   delta theta_B
kAccelerometerBiasIndex = 9   delta b_a
kGyroscopeBiasIndex     = 12  delta b_g
```

Position and velocity errors are additive world-frame quantities. Rotation error
is a right/local body-frame perturbation. This mixed convention is intentional
and is the convention implemented by `propagation.cpp`, `augmentation_jacobians.cpp`,
and their tests.

Position comes before orientation. Any derivation written as `[delta theta,
delta p]` must be reordered before inserting blocks into `F`, `H`, or an
augmentation Jacobian.

Landmarks are appended after index 15 as contiguous metric XYZ world positions:

```text
state dimension = 15 + 3 * active_landmarks
```

`NominalState` stores fields in the same conceptual order used by propagation:
position, velocity, orientation, accelerometer bias, gyroscope bias. Parsed
`GroundTruthState` stores gyroscope bias before accelerometer bias because that
is EuRoC CSV order; do not confuse the struct field order with covariance block
order.

## 2. Rotation Error Convention

Rotations compose on the right:

```text
R_WB_perturbed = R_WB * Exp(delta theta)
```

`delta theta` is expressed in the body-local frame. This matches
`orientation * Sophus::SO3d::exp(...)` in `propagation.cpp`.

Do not turn this into a left/global convention when copying formulas from a
paper. A left/right mismatch silently changes signs and transposes Jacobian
blocks.

## 3. Position and Velocity Error Convention

Position and velocity errors are additive in the world frame:

```text
p_WB_perturbed = p_WB + delta p_W
v_WB_perturbed = v_WB + delta v_W
```

Do not use `p_WB + R_WB * delta p` with the current covariance layout. That is a
different body-local translation convention and would require changing
propagation, augmentation, measurement Jacobians, and tests together.

This convention is why metric landmark augmentation has:

```text
d l_W / d delta p_W = I
d l_W / d delta theta = -R_WB [l_B]_x
```

## 4. Skew-Symmetric Sign Convention

The project uses:

```text
[v]_x w = v cross w
```

`propagation.cpp` intentionally applies leading negative signs in several
blocks, for example:

```text
F_v_theta = -R_WB [a_B]_x
F_theta_theta = -[omega_B]_x
F_theta_bg = -I
```

Do not assume a newly derived bare `[.]_x` term has the right sign. Check it
against the perturbation convention in this document and against a trivial
numeric case.

## 5. Frame Names

Names follow `A_from_B`: the transform maps a B-frame point into frame A.

Examples:

```text
body_from_camera_transform  maps l_C -> l_B
camera_from_body_transform  maps l_B -> l_C
R_WB                        maps body vectors -> world vectors
```

If a helper already returns `camera_from_body_transform`, do not invert it again
at the call site. The helper already computes the `T_BS` inverse internally.

`CameraCalibration::t_bs` and `ImuCalibration::t_bs` are body-from-sensor
transforms from EuRoC `T_BS`. For cameras:

```text
l_B = R_BS * l_C + p_BS
```

For the IMU, measurements are already represented in the body/IMU convention
used by propagation. Online IMU-camera extrinsic calibration is not part of the
current state.

## 6. SE(3) Inversion

Pose inversion is not a bare transpose applied to the world point. Translation
must be subtracted first:

```text
l_B = R_WB^T * (l_W - p_WB)
```

The incorrect form is:

```text
l_B != R_WB^T * l_W - p_WB
```

Prefer existing geometry helpers (`world_point_to_camera`,
`camera_point_to_world`, `camera_from_body_transform`, `body_from_camera_transform`)
over hand-assembling inverse transforms at call sites.

## 7. Landmark Parameterization

Landmarks are metric XYZ points in the world frame, 3 dimensions each.

This was chosen over anchored inverse depth for the current EuRoC stereo scope:
indoor 1-10 m ranges keep depth uncertainty manageable, and 3D landmarks keep
the bounded covariance smaller. Inverse depth remains a future option for
long-range, outdoor, or monocular initialization.

Keep interfaces factored so a future parameterization swap is possible:

```text
H_landmark = d pixel / d l_W * d l_W / d x_l
```

For the current metric XYZ state, `d l_W / d x_l = I`.

## 8. Measurement Model

`predict_pinhole_pixel(...)` is pure geometry:

```text
world -> body -> camera -> normalized -> pixel
```

It depends on pose, one world landmark, and one camera calibration. It does not
know about velocity, biases, covariance, noise, process dynamics, landmark
storage, or feature association.

Visibility and image-bound gating stay outside `h(.)`:

```text
caller must check Z > 0 before using an innovation
```

`Z == 0` usually produces non-finite output. `Z < 0` can produce finite but
physically invalid pixels, so `isfinite()` is not a visibility test.

The synthetic frontend uses half-open image bounds:

```text
0 <= u < width
0 <= v < height
```

Tests for the pure measurement model may project exactly to a border to verify
geometry, but synthetic observation generation rejects pixels outside the
half-open image domain.

Distortion is also outside `h(.)`. Raw pixel measurements must be undistorted
before forming `z - h(x)`. Distortion coefficients are fixed calibration
constants, not estimated state, so modeling distortion inside `h(.)` adds
Jacobian complexity without changing `dh/dx`.

## 9. Measurement Jacobian Structure

For a landmark prediction:

```text
l_B = R_WB^T * (l_W - p_WB)
l_C = R_CB * l_B + p_CB
```

The shared projection factor is:

```text
J_pixel_C =
[ fx / Z,      0, -fx * X / Z^2
       0, fy / Z, -fy * Y / Z^2 ]
```

With the repo's additive world-position and right-local rotation convention:

```text
H_pose = J_pixel_C * R_CB * [ -R_WB^T, 0, [l_B]_x, 0, 0 ]
H_landmark = J_pixel_C * R_CB * R_WB^T
```

The nonzero pose blocks land at position columns `0..2` and orientation columns
`6..8`; velocity and bias columns are structurally zero for a direct camera
measurement. The observed landmark contributes one 3-column block. Every other
landmark block is structurally zero.

Do not materialize dense `H` by default. Build or apply only the active nonzero
blocks unless a test or debug path explicitly needs the dense matrix.

## 10. Stereo Triangulation and Augmentation

`triangulate_stereo(...)` accepts already-rectified, distortion-free stereo
pixels and rectified pseudo-calibrations. Raw EuRoC cam0/cam1 calibration does
not satisfy this contract until an undistortion/rectification layer is added.

A rectified rig means:

```text
cam0 and cam1 share orientation
cam1 is at positive cam0 x baseline
baseline y and z are zero within tolerance
cam0/cam1 rectified intrinsics match
```

Only under that contract is disparity:

```text
disparity = u0 - u1
```

The triangulation covariance is anisotropic:

```text
R_tri = J_tri * Sigma_pixels * J_tri^T
```

Landmark augmentation uses:

```text
l_W = R_WB * (R_BS * l_C + p_BS) + p_WB
J_robot = [ I, 0, -R_WB [l_B]_x, 0, 0 ]
J_lC = R_WB * R_BS
```

When inserting a new landmark, preserve covariance correlations:

```text
P_rl,new   = P_rr * J_robot^T
P_old,new  = P_old,r * J_robot^T
P_new,new  = J_robot * P_rr * J_robot^T + J_lC * R_tri * J_lC^T
```

The implementation exploits the sparsity of `J_robot`; keep that property when
changing augmentation code.

## 11. Propagation and SLAM Covariance Blocks

`F` is continuous-time. `Phi` is discrete:

```text
Phi = I + F * dt
P_rr_new = Phi * P_rr * Phi^T + Q_d
P_rl_new = Phi * P_rl
P_ll_new = P_ll
```

Landmarks do not move during IMU prediction, so `P_ll` is unchanged.

`Q_d` is currently first-order:

```text
Q_d = G * Q_raw * G^T * dt
```

Higher-order discretization or Van Loan integration is a future numerical
upgrade and must be benchmarked against the existing Monte Carlo covariance
gate.

IMU acceleration measurements are body-frame specific force. Propagation removes
accelerometer bias, rotates the result into world, then adds gravity:

```text
a_W = R_WB * (a_meas - b_a) + g_W
g_W = [0, 0, -9.81]
```

Gyroscope measurements are body-frame angular velocity. Propagation removes gyro
bias and right-multiplies the nominal orientation:

```text
R_WB_new = R_WB * Exp((omega_meas - b_g) * dt)
```

The gravity constant is duplicated in `propagation.cpp` and `synthetic.cpp`; keep
the two values aligned until it is promoted to shared configuration.

Zero `dt` is a legal no-op. Negative and non-finite timesteps are errors because
they would run covariance propagation backwards or produce non-finite state.

## 12. Update and Marginalization Decisions

The first camera update should use sequential stereo observations. One landmark
keeps the innovation covariance at `4x4`; batching `m` landmarks makes it
`4m x 4m`. Sequential and batched updates are mathematically equivalent under
uncorrelated measurement noise, which is the current assumption, but their
implementation shape is different.

Landmark removal is moment-form row/column deletion and dense compaction of
`P`, not a Schur complement. Schur complements belong to information-form or
factor-graph marginalization, not this covariance-form EKF state.

## 13. Verification Rules

Every new formula needs three checks before it should be trusted:

1. A first-principles derivation under this document's conventions.
2. A trivial numeric sanity case, usually identity transforms and zero values.
3. A test that would fail for at least one plausible convention bug.

Reference papers are useful, but their frame and perturbation conventions must
be translated explicitly before copying a formula into this project.

## 14. Data and Calibration Units

Timestamps use integer nanoseconds:

```text
using TimestampNs = std::uint64_t
```

Durations passed to propagation and synthetic generation are seconds as `double`.
Convert explicitly at boundaries.

EuRoC ground-truth quaternions are parsed in `qw, qx, qy, qz` order and stored
as `Eigen::Quaterniond(qw, qx, qy, qz)`.

Camera intrinsics are:

```text
[fx, fy, cx, cy]
```

Camera distortion coefficients are parsed and stored, but current measurement
prediction and rectified triangulation do not consume raw distorted pixels.

IMU calibration noise values are continuous-time densities/random walks from
EuRoC YAML. The filter squares them into variances when building `Q_raw`.
Synthetic IMU noise follows the same convention:

```text
per-sample stddev = continuous_density * sqrt(rate_hz)
```

Synthetic pixel noise is different: it is configured directly as per-pixel
standard deviation in pixels.

## 15. Synthetic Harness Conventions

The synthetic trajectory owns the time origin and true biases. Ground truth, IMU
samples, and stereo observations generated from one trajectory must use that
same origin and those same biases.

Samples land on a `1 / rate_hz` grid from the trajectory start. Durations that do
not land exactly on the grid truncate to the last grid point on or before the
requested duration. Products near an integer number of steps are rounded before
truncation to avoid floating-point representation dropping the final intended
sample.

Synthetic truth uses:

```text
R_WB(t) = R_WB(0) * Exp(omega_B * t)
```

IMU synthesis emits specific force using the same gravity convention as
propagation:

```text
a_meas = R_WB^T * (a_W - g_W) + b_a
omega_meas = omega_B + b_g
```

Seeded noise must be reproducible across standard libraries. Use the existing
deterministic sampler pattern rather than `std::normal_distribution` when adding
new seeded Gaussian synthetic streams.

## 16. SLAM State Storage Conventions

`SlamState` owns one preallocated dense covariance matrix:

```text
storage_dim = 15 + 3 * max_landmarks
active_dim = 15 + 3 * active_landmarks
```

Only the top-left `active_dim x active_dim` block is live. Inactive covariance
storage and inactive landmark positions are filled with `NaN` so accidental
reads fail visibly.

Landmark storage is dense by active slot. External `LandmarkId` values are not
state indices; resolve them through `landmark_offset(id)` immediately before
using an offset.

Adding or removing a landmark changes the active layout. Do not retain
covariance block views, landmark offsets, or active-dimension assumptions across
`add_landmark(...)` or `remove_landmarks(...)`.

`add_landmark(...)` receives the complete new covariance column with shape:

```text
(old_active_dim + 3) x 3
```

The bottom `3x3` block is the new landmark covariance. The upper rows contain
robot/new-landmark and old-landmark/new-landmark cross-covariances. The
container mirrors the off-diagonal row for symmetry; callers are responsible for
supplying a mathematically valid covariance column.

Batch landmark removal compacts survivors and rebuilds the ID registry. It is a
moment-form covariance row/column deletion and copy operation, not a
free-list-based storage policy.
