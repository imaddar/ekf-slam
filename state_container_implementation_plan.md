# State Container and Augmentation Implementation Plan

This plan covers the transition from the current IMU-only propagation path to
the bounded metric XYZ EKF-SLAM state. Each step has an explicit gate. A
dependent step must not begin until its gate passes.

## Architectural Decisions

- Use metric XYZ landmarks with `kLandmarkDim = 3`.
- Use `kRobotDim = 15` for the robot error state.
- Allocate one full covariance matrix at construction:

  ```text
  (15 + N_max * 3) x (15 + N_max * 3)
  ```

- Track active landmark count separately and operate on the active top-left
  covariance view.
- **Covariance storage is private.** All mutation goes through the container's
  API. The no-reallocation guarantee is only an invariant if callers cannot
  reach the matrix directly.
- **Active landmark count is derived from registry operations, never set
  directly.** A count-setter exposes or hides state without touching the data
  underneath: increasing it reveals uninitialized memory, decreasing it orphans
  covariance blocks.
- Keep the existing 15x15 covariance container as an IMU-only
  development/testing artifact, not as the future SLAM architecture.
- Use compile-time-sized Eigen block access for fixed-size landmark blocks.
  Dimensions are template parameters; offsets may be runtime values.
- Keep the verified right/local rotation-error convention.
- Use an explicit landmark-ID registry; landmark IDs are never state indices.
- Use batch compaction for landmark removal and rebuild state offsets after
  removal.
- Keep inverse depth as future work for larger spatial scales, long-range
  landmarks, or monocular initialization. For this bounded stereo use case,
  the additional parameterization and bookkeeping complexity outweigh its
  benefits.

## Step 0: Confirm Prerequisites

Verify the existing synthetic harness and IMU propagation baseline.

### Gate 0

- The fully excited synthetic trajectory passes.
- Existing IMU covariance tests pass.
- Monte Carlo propagation consistency passes.
- The right/local rotation-error convention is documented and verified against
  `propagation.cpp`.

## Step 1: Joint State Storage

Implement the future SLAM state container.

### Work

- Private covariance storage; no public handle that permits
  `conservativeResize` or any other reallocating operation.
- Active landmark count is read-only from outside; it changes only as a
  consequence of registry add/remove operations introduced in Step 2.
- **Initial `P_rr` must reproduce the existing propagation path's starting
  uncertainty.** Zero-initialization is not correct: a zero covariance block
  asserts infinite confidence. Identify what the IMU-only path currently uses
  for initial pose, velocity, and bias uncertainty, and where that
  initialization lives in the new container.
- The covariance region beyond the active dimension is undefined. It is written
  by augmentation and must never be read before that. Assert this rather than
  relying on zero-fill.

### Gate 1

- `N = 0` matches the old 15x15 path exactly, **including the initial
  covariance values** — not merely the dimensions.
- The source of initial `P_rr` is documented, and a test asserts it against the
  IMU-only baseline.
- Covariance dimensions are correct for multiple `N_max` values.
- Activating landmarks does not change the covariance allocation or data
  pointer.
- Active top-left views have correct dimensions.
- Fixed-size landmark block read/write round-trips correctly.
- No public API permits reallocation of the covariance storage.

The immediate task is to close any missing Gate 1 evidence before advancing.

## Step 2: Landmark Registry and Batch Compaction

Add landmark bookkeeping independently of filter mathematics.

### Work

- Store metric XYZ landmark values in the joint state.
- Maintain an explicit `landmark_id -> active state offset` registry.
- Landmark offsets are computed only inside the registry. No caller performs
  arithmetic on landmark number to reach a state index.
- Implement batch compaction for removal.
- Rebuild offsets after compaction.
- Return an error when removing a missing landmark ID.
- Active landmark count updates as a consequence of add/remove, and is not
  settable by any other path.

### Gate 2

- Interior landmark removal preserves all surviving IDs.
- Landmark state values remain unchanged.
- Covariance blocks for surviving landmarks remain unchanged.
- Batch removal performs one compaction pass.
- Missing IDs fail loudly.
- Add/remove/add behavior is deterministic.
- Augmentation attempted at `N_max` follows a documented policy — reject, or
  trigger removal first — rather than asserting or reallocating.

## Step 3: SLAM Covariance Propagation

Extend prediction to the joint covariance.

For each IMU step:

- Propagate `P_rr` using the existing robot transition.
- Propagate `P_rl` by left-multiplying with the robot transition.
- Leave `P_ll` unchanged.
- Never construct a full augmented transition matrix.

### Gate 3

- `P_ll` is bit-identical after prediction.
- `P_rl` matches an explicit augmented-matrix reference implementation.
- `P_rr` matches the existing IMU-only result.
- The covariance remains symmetric.
- `P_rr` is positive **definite** — with process noise present, the weaker PSD
  check would mask a problem. Assert the stronger property where it holds.
- The full joint covariance remains positive semidefinite.
- `N = 0` remains numerically identical to the old propagation path.

## Step 4: Stereo Forward Map

Implement and test the metric XYZ world-to-camera initialization geometry:

```text
l_W = R_WB (R_BS l_C + p_BS) + p_WB
```

Keep parameterization-specific logic behind a narrow interface.

### Gate 4

- The identity-transform sanity case passes.
- World-to-camera-to-world round-trip passes to machine precision.
- Nonzero translation and rotation extrinsics pass hand-computed tests.
- Extrinsics come from `CameraCalibration`, never hardcoded.

## Step 5: Augmentation Jacobians

Implement analytical Jacobians for metric XYZ augmentation. Build the
finite-difference checker before relying on these Jacobians. The checker is
reused for `H` in the measurement-update phase, so it is not
single-use tooling.

Required structure:

- `J_r` is `3 x 15`.
- The position block is identity.
- The orientation block follows the verified right/local error convention.
- Velocity and bias blocks are zero.
- `J_lC` is derived directly from the forward map.

### Gate 5

- Analytical and numerical `J_r` agree to approximately `1e-6`.
- Analytical and numerical `J_lC` agree to approximately `1e-6`.
- Position and zero blocks are structurally exact.
- Orientation perturbations produce the expected skew structure.
- The convention is verified against `propagation.cpp`.

## Step 6: Triangulation Uncertainty

Implement anisotropic stereo triangulation covariance:

```text
R_tri = J_tri Sigma_pixels J_tri^T
```

Required behavior:

- Depth uncertainty grows approximately with `Z^2`.
- Lateral uncertainty grows approximately with `Z`.
- Uncertainty is oriented primarily along the viewing ray.
- Near-zero disparity is rejected.

An isotropic stub is easy to write and then never revisit. The resulting
overconfidence surfaces much later as a NEES failure that would naturally be
blamed elsewhere.

### Gate 6

- `R_tri` is symmetric and positive semidefinite.
- Its largest eigenvector aligns with the viewing ray.
- Depth scaling is verified at multiple ranges.
- Lateral scaling is verified.
- Monte Carlo empirical covariance matches the analytical covariance.
- Degenerate disparity produces a controlled error.

## Step 7: Landmark Augmentation

Insert new landmarks into the joint state and covariance:

```text
P_ll,new  = J_r P_rr J_r^T + J_lC R_tri J_lC^T
P_rl,new  = P_rr J_r^T
P_old,new = P_old,r J_r^T
```

Maintain covariance symmetry when writing the new blocks.

### Work — exploit `J_r` sparsity

`J_r` has only two nonzero `3x3` blocks: position and orientation. Therefore
`P_rr J_r^T` requires only the pose columns of `P_rr` — a `15x6` slice times a
`6x3` — not a dense `15x15` multiply. The same applies to `P_old,r J_r^T`.

This runs on every landmark birth. The naive dense form is the obvious one to
write and should be explicitly avoided.

### Gate 7

- `P_rl` is nonzero after augmentation.
- **Velocity and bias rows of `P_rl,new` are nonzero**, despite `J_r` having
  zero columns there, because `P_rr` is dense. If these rows come out zero,
  something is wrong. This is the mechanism by which landmark observations
  eventually correct accelerometer bias, and it is a sharper check than
  "`P_rl` is nonzero."
- Covariance remains symmetric and positive semidefinite.
- Zero pose uncertainty produces the expected measurement-only covariance.
- Nonzero pose uncertainty contributes correctly, and dominates `P_ll,new` at
  realistic magnitudes.
- Same-frame landmarks become mutually correlated.
- Different-frame landmarks inherit correlation through old cross-covariance.
- Existing covariance blocks remain unchanged.
- Monte Carlo landmark covariance matches the analytical result.

## Step 8: End-to-End Synthetic Integration

Connect propagation, registry, augmentation, and removal.

### Gate 8

- The fully excited synthetic trajectory remains symmetric and positive
  semidefinite.
- Landmarks can be added and batch-removed without index corruption.
- No covariance reallocation occurs within the configured landmark budget —
  enforced by the private-storage invariant from Step 1, not merely observed at
  current call sites.
- Augmentation cost per landmark is measured.
- Timing results are recorded in `BENCHMARKS.md`.

## Step 9: Final Documentation and Architecture Review

Before beginning camera measurement updates:

- The old IMU-only container is clearly marked as a development/testing
  artifact.
- Metric XYZ is documented as the current parameterization.
- Inverse depth is documented as future work for larger spatial scales or
  monocular initialization.
- The right/local error convention is consistent across propagation and
  augmentation.
- Batch compaction is documented as the removal policy.
- `ARCHITECTURE.md`, `scope.md`, and `present.md` describe only implemented
  behavior and settled design decisions.

### Decision to force before the update phase

**Batch versus sequential measurement updates.** This is unresolved and shapes
how the update step is structured.

`S = H P H^T + R` scales with the *measurement* dimension, not the state
dimension. One stereo observation gives four numbers, so `S` is `4x4`.
Observing `m` landmarks as a batch makes it `4m x 4m`.

Sequential updates keep `S` at `4x4` permanently and avoid the larger
inversion, at the cost of `m` separate covariance updates instead of one. The
two are mathematically equivalent when measurement noise is uncorrelated across
landmarks, which holds here. The tradeoff is real rather than free and should
be measured.

Note that the dominant cost in either case is the covariance update
`P+ = (I - K H) P`, which is `O(n^2)` at best — not the innovation inversion.

---

The next implementation action is to close Gate 1. In particular, initial
`P_rr` values and private covariance storage are the two outstanding items. The
landmark registry must wait until that gate passes.