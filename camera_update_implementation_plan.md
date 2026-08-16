# Sequential Camera Update Implementation Plan

This plan covers the first camera measurement update: a sequential, per-landmark
stereo EKF update over the bounded metric XYZ state in `SlamState`. It picks up
where [state_container_implementation_plan.md](state_container_implementation_plan.md)
stops — the state container, triangulation, and augmentation exist; nothing
subtracts information from `P` yet.

Each step has an explicit gate. A dependent step must not begin until its gate
passes.

**Status: executed 2026-08-15.** All eight steps are implemented and every gate
passes, with one documented exception: Gate 6's NEES consistency bound. The
filter is measurably over-confident with camera updates enabled (`24.23` against
an upper bound of `16.56`), isolated to update linearization rather than a
covariance defect. The live test asserts a regression ceiling instead, and
`SlamClosedLoopTest.DISABLED_MonteCarloRobotNeesMeetsTheConsistencyTarget`
carries the real bound as the acceptance test for the FEJ/OC-EKF work. See the
NEES finding in `BENCHMARKS.md`.

## Architectural Decisions

- **Sequential per-landmark updates, not batch.** Under block-diagonal `R` and a
  shared linearization point, sequential and batch are algebraically identical
  (both reduce to `P^-1 + sum_i H_i^T R_i^-1 H_i` in information form).
  Sequential is chosen for per-landmark chi-square gating, `4x4` innovation
  inversions instead of one `4m x 4m`, and a small fixed working set. Batch is
  implemented only as a test oracle, never on the hot path.
- **The stereo pair is the atomic update unit.** One landmark contributes 4 rows
  (`[u0, v0, u1, v1]`), matching `triangulate_stereo(...)` pixel ordering. Two
  2-row updates would be equivalent under frozen linearization, but the 4-row
  block keeps the disparity constraint in one step and makes 4-dof gating the
  natural statistical unit for "is this stereo match good".
- **Linearization is frozen at the prior for the whole frame.** All `m`
  predictions and Jacobians are computed from the nominal state as it stands at
  frame entry, before any of them touches `P`. Relinearizing mid-sweep makes the
  result order-dependent and manufactures information in the unobservable
  directions (global position, yaw), which is the direction the
  observability-constrained / first-estimates-Jacobian open decision in
  `ARCHITECTURE.md` needs to move away from, not toward.
- **Injection is deferred to one operation at end of frame.** The nominal state
  stays fixed during the sweep; the error state accumulates. This is what makes
  the sweep equal the batch answer, and it means the SO(3) injection and the
  covariance reset Jacobian each happen exactly once per frame.
- **The innovation carries the accumulated-error term.** Step `i` uses
  `nu_i = y_i - H_i * delta_x_hat`, not the raw residual `y_i`. Dropping
  `-H_i * delta_x_hat` double-counts corrections already absorbed by landmarks
  `1..i-1`. This is the single easiest thing to get wrong here and Gate 4 exists
  specifically to catch it.
- **Gating uses the prior covariance, not the running covariance.** `P` shrinks
  monotonically through the sweep, so a strict-sequential gate is tighter for
  later landmarks and can reject a good measurement because earlier updates
  already over-shrank `P`. Prior gating is order-independent, matches the
  diagonal blocks of the batch `S`, and errs loose rather than tight.
- **Gating needs no snapshot of `P`.** `H_i P H_i^T` depends only on the `9x9`
  sub-block of `P` at rows/columns `{0..2, 6..8, offset..offset+2}`. Gather that
  per landmark (81 doubles) instead of copying the full active covariance.
- **`H` is never materialized.** Each `H_i` has exactly 9 nonzero columns
  (`CONVENTIONS.md` §9). `H_i P` is assembled from three `4x3` blocks times three
  `3 x n` row strips of `P`.
- **Joseph form, evaluated in factored order.** `P+ = M - (M H^T) K^T + K R K^T`
  with `M = P - K (H P)`. Three rank-4 `n^2` updates instead of the simple
  form's one. Do not algebraically simplify this: with the optimal gain, Joseph
  collapses to `P - K(H P)` in exact arithmetic, so its entire value is that it
  is computed differently and degrades more gracefully under roundoff across
  `m` successive updates.
- **Never update a landmark in the frame that gave birth to it.** Its
  triangulated position and covariance are deterministic functions of those
  exact pixels, so the "measurement" is perfectly correlated with the prior and
  the Kalman independence assumption fails outright. New landmarks are augmented
  after the update completes.
- **The update does not augment.** Observations of unknown landmark IDs are
  reported as skipped; the caller decides whether to call `augment_landmark`.
  This keeps augmentation's structural view invalidation (`CONVENTIONS.md` §16)
  out of the middle of a sweep that holds landmark offsets.
- **Deterministic sweep order.** Order-independence holds in exact arithmetic,
  not bitwise. Iterate in ascending landmark state offset so runs are
  reproducible and the order-invariance test compares to a tolerance rather than
  to bit equality.

## Step 0: Confirm Prerequisites

No new code. Establish the baseline the update will be measured against.

### Gate 0

- The existing 90-test suite passes.
- `make_measurement_jacobian_blocks(...)` and `augment_landmark(...)` tests pass.
- The right/local rotation convention (`CONVENTIONS.md` §2) is confirmed against
  `propagation.cpp` and `augmentation_jacobians.cpp`.
- Record propagation-only drift on the synthetic trajectory in `BENCHMARKS.md`.
  Without this number, Step 6 has nothing to show improvement against.

## Step 1: Stereo Measurement Blocks

New files `measurement_update.hpp` / `measurement_update.cpp`. This step is pure
geometry and linearization; no covariance is touched.

### Work

Stack the two existing per-camera calls into one 4-row stereo block:

```cpp
struct StereoMeasurementBlocks {
    Eigen::Matrix<double, 4, 6> pose;      // [delta p_W | delta theta_B]
    Eigen::Matrix<double, 4, 3> landmark;
    Eigen::Vector4d prediction;            // [u0, v0, u1, v1]
    double depth_cam0;                     // Z in cam0, for the visibility gate
    double depth_cam1;
    int landmark_offset;
};

ParseResult<StereoMeasurementBlocks> make_stereo_measurement_blocks(
    const NominalState& robot,
    const Eigen::Vector3d& landmark_world,
    const CameraCalibration& cam0,
    const CameraCalibration& cam1,
    int landmark_offset);
```

Two `predict_pinhole_pixel(...)` calls and two `make_measurement_jacobian_blocks(...)`
calls; rows 0-1 from cam0, rows 2-3 from cam1. Both share the same pose and the
same landmark, so both `pose` halves and both `landmark` blocks stack directly.

Depths are surfaced rather than gated here, keeping the `CONVENTIONS.md` §8
contract that visibility lives outside `h(.)`. The sweep in Step 4 rejects an
observation unless both depths are positive.

The `pose` block keeps the compact `[delta p, delta theta]` ordering documented
on `MeasurementJacobianBlocks`. Scattering to the non-contiguous state columns
`0..2` and `6..8` happens in the sparse kernel, in one place.

### Gate 1

- Central-difference Jacobian check against `make_stereo_measurement_blocks`,
  perturbing on the manifold: position additively, orientation as
  `R * Exp(eps * e_k)` per `CONVENTIONS.md` §2, landmark additively. Agreement to
  `1e-6` relative.
- A test that fails under a left/global rotation perturbation, so the check has
  power against the convention bug it exists to catch.
- Identity-extrinsics, identity-rotation sanity case with hand-computed values.
- Rows 2-3 differ from rows 0-1 only through the cam1 extrinsic, verified on a
  rectified rig where the difference is a pure baseline shift.

## Step 2: Error-State Injection and Covariance Reset

`SlamState` currently exposes `landmark_position(id)` as a getter only, and the
nominal robot state has no injection path. Both are needed before any update can
write its result back.

### Work

Add to `SlamState`:

```cpp
// Injects a full active-dimension error state into the nominal state and
// applies the rotation reset Jacobian to P. Error-state ordering follows
// CONVENTIONS.md section 1.
ParseResult<void> inject_error_state(
    const Eigen::Ref<const Eigen::VectorXd>& error_state);
```

Putting this on the container rather than in the update keeps
`landmark_positions_` private and keeps the one manifold operation in one place.
It validates `error_state.size() == active_dim()` and finiteness.

Nominal injection:

```text
position           += delta p
velocity           += delta v
orientation         = orientation * Exp(delta theta)     // right/local
accelerometer_bias += delta b_a
gyroscope_bias     += delta b_g
landmark_i         += delta l_i
```

Covariance reset. This settles the "State injection and reset" open decision in
`ARCHITECTURE.md`; implement it rather than skipping it. Derivation under the
repo's right/local convention, with true rotation `R = R_hat Exp(delta theta)`:

```text
R_hat+ = R_hat Exp(delta theta_hat)
R      = R_hat+ Exp(delta theta+)
=>  Exp(delta theta+) = Exp(-delta theta_hat) Exp(delta theta)
```

Writing `delta theta = delta theta_hat + e` and using the SO(3) right Jacobian,
`Exp(delta theta_hat + e) ~= Exp(delta theta_hat) Exp(J_r(delta theta_hat) e)`,
so `delta theta+ ~= J_r(delta theta_hat) e` and

```text
G_theta = J_r(delta theta_hat) ~= I - 0.5 * [delta theta_hat]_x
P <- G P G^T,   G = blkdiag(I3, I3, G_theta, I3, I3, I_{3N})
```

Only rows/columns `6..8` differ from identity, so apply it as
`P.middleRows<3>(6) = G_theta * P.middleRows<3>(6)` then the symmetric column
operation — `18n` flops, once per frame.

Confirm what the vendored Sophus version actually exposes before assuming a
right-Jacobian helper exists; if it only offers a left Jacobian, use
`J_r(x) = J_l(-x)`, and otherwise implement the closed form directly.

### Gate 2

- Zero error state is an exact no-op on both nominal state and `P`.
- Injection round-trips: a known `delta_x` produces the hand-computed nominal
  state, with the rotation checked as `Exp` composition rather than addition.
- Reset Jacobian verified against a numerically differentiated
  `delta theta+ (delta theta)` map at a `delta theta_hat` large enough
  (~0.1 rad) that `G_theta` is measurably different from identity. A test that
  passes with `G_theta = I` has no power.
- Size and finiteness rejection paths return `ParseResult` errors, no panics.
- The NaN poison beyond `active_dim` is untouched.
- Existing `slam_state_tests` still pass.

## Step 3: Sparse Kernel and Single-Landmark Update

The numerical core. Still no gating, no sweep — one landmark, applied to `P`.

### Work

The sparse row-block product, the routine everything else is built on:

```text
column set C = {0,1,2} U {6,7,8} U {offset, offset+1, offset+2}

HP  = pose.leftCols<3>()  * P.middleRows<3>(0)
    + pose.rightCols<3>() * P.middleRows<3>(6)
    + landmark            * P.middleRows<3>(offset)      // 4 x n

S   = HP(:, C) * H(:, C)^T + R                           // 4 x 4, reuse the same three blocks
K   = HP^T * S^-1                                        // n x 4, via 4x4 LLT
```

`P H^T = (H P)^T` by symmetry of `P`, so `K` needs no second sparse product.

Covariance update, factored Joseph:

```text
M  = P - K * HP
P <- M - (M(:, C) * H(:, C)^T) * K^T + K * R * K^T
P <- 0.5 * (P + P^T)
```

`M H^T` reuses the same 9-column gather, costing `n * 9 * 4` rather than
`n * n * 4`. Three rank-4 `n^2` updates total, roughly `12n^2` per landmark.

Preallocate `HP` (`4 x storage_dim`), `K` (`storage_dim x 4`), and `M`. `M` is
`n x n` and must not be allocated per landmark; either reuse one scratch buffer
owned by the caller or restructure to update `P` in place. Per-frame heap
traffic in the camera path is the thing the preallocated-covariance decision in
`ARCHITECTURE.md` exists to avoid.

### Gate 3

- Single-landmark update matches a dense reference (materialize the full
  `4 x n` `H`, run textbook `K = P H^T S^-1`, `P = (I - K H) P`) to `1e-12`.
- `P` stays symmetric to `1e-14` and positive definite (smallest eigenvalue
  `> 0`) after the update.
- `P` strictly decreases in the Loewner sense: `P_prior - P_post` is PSD.
- A zero-information case (`R` scaled to `1e12`) leaves `P` and `delta_x`
  essentially unchanged.
- Landmark offsets resolved through `landmark_offset(id)`, never assumed.

## Step 4: Frame Sweep with Deferred Injection

The step the whole plan is about.

### Work

```cpp
struct StereoObservation {
    LandmarkId id;
    Eigen::Vector2d pixel_cam0;
    Eigen::Vector2d pixel_cam1;
};

struct LandmarkUpdateDiagnostics {
    LandmarkId id;
    bool applied;
    double mahalanobis_distance;   // against the prior
    Eigen::Vector4d innovation;    // prior residual y_i
};

struct StereoUpdateResult {
    int applied_count;
    int gated_count;
    int skipped_count;             // unknown id, or behind either camera
    std::vector<LandmarkUpdateDiagnostics> diagnostics;
};

ParseResult<StereoUpdateResult> update_stereo_frame(
    SlamState& state,
    std::span<const StereoObservation> observations,
    const CameraCalibration& cam0,
    const CameraCalibration& cam1,
    const Eigen::Matrix4d& pixel_covariance,
    const UpdateOptions& options = {});
```

`pixel_covariance` is per-pixel detector noise in `[u0, v0, u1, v1]` ordering —
typically `sigma_px^2 * I4`. It is **not** `StereoTriangulation::covariance`.
`R_tri` is the propagated 3D position covariance consumed at augmentation
(`CONVENTIONS.md` §10); using it here would both double-count the pixel noise
and reuse information already sitting in `P_ll` from the landmark's birth.

Sweep structure:

```text
phase A (no writes to P):
  for each observation, in ascending landmark offset:
    resolve offset; unknown id -> skipped
    make_stereo_measurement_blocks(...) from the frame-entry nominal state
    reject unless depth_cam0 > 0 and depth_cam1 > 0
    y_i = z_i - prediction
    gather the 9x9 prior sub-block of P; S_i0 = H_i P H_i^T + R
    gate (Step 5)

phase B (sequential, nominal state still frozen):
  delta_x = 0
  for each surviving observation:
    nu_i = y_i - H_i * delta_x            // sparse: only the 9 entries of delta_x
    S_i, K_i from the current P
    delta_x += K_i * nu_i
    Joseph update of P

phase C:
  state.inject_error_state(delta_x)       // nominal + reset Jacobian, once
```

`H_i * delta_x` touches only the 9 relevant entries of `delta_x`, so it costs 36
flops, not `4n`.

Skipped and gated observations must not contribute to `delta_x`, and their
landmark blocks in `P` must be left alone.

### Gate 4

This gate is the reason batch gets implemented at all.

- **Batch-equivalence oracle.** Build a synthetic frame with `m = 3` observed
  landmarks. Run the sequential sweep. Independently build the dense
  `12 x n` `H`, `12 x 12` block-diagonal `R`, and run one batch update. Assert
  `||delta_x_seq - delta_x_batch||_inf < 1e-12` and the same for `P`.
- **Order invariance.** Shuffle the observation order across all permutations of
  the 3 landmarks; every permutation agrees to `1e-10`.
- These two together catch every failure mode this design has: dropping
  `-H_i delta_x` breaks equivalence; relinearizing mid-sweep breaks both;
  a non-block-diagonal `R` breaks equivalence.
- Repeat the oracle at `m = 8` with overlapping landmarks to confirm the
  cross-covariance paths are exercised, not just the diagonal.
- An observation behind either camera is skipped and provably leaves `P`
  bit-identical.
- An unknown `LandmarkId` is reported, not an error return; a frame of entirely
  unknown IDs is a valid no-op.
- `P` symmetric and PSD after a 20-landmark sweep.

## Step 5: Chi-Square Gating

### Work

```cpp
struct UpdateOptions {
    // 4 dof: 9.4877 at p=0.95, 13.2767 at p=0.99.
    double chi_square_threshold = 9.4877;
    bool use_joseph_form = true;
};
```

Accept observation `i` iff `y_i^T (S_i0)^-1 y_i < threshold`, with `S_i0` built
from the **prior** `P` in phase A. Both the residual and the innovation
covariance come from before any update is applied, so the accept/reject
partition is a function of the frame alone and not of sweep order.

Reuse the phase-A `4x4` LLT for the quadratic form rather than inverting `S_i0`.

### Gate 5

- A clean synthetic frame admits all observations.
- One observation displaced by 50 pixels is rejected while every other
  observation's `applied` flag and resulting `delta_x` are unchanged from the
  clean run to `1e-12` — the outlier leaves no trace.
- The accept/reject set is invariant to observation order (exactly, since phase
  A does not write `P`).
- A threshold of `0.0` rejects everything and leaves `P` unchanged; a huge
  threshold reproduces the ungated result.
- Diagnostics report a Mahalanobis distance for every observation including
  rejected ones, so a gating threshold can be tuned from recorded data.

## Step 6: Synthetic Closed-Loop Integration

### Work

Extend `tests/slam_integration_test.cpp` (or add
`tests/measurement_update_test.cpp` plus a new CMake target
`measurement_update_tests` alongside the existing per-module targets) with a
propagate/augment/update loop over `synthesize_imu(...)` and
`synthesize_stereo_observations(...)` from one `SyntheticTrajectory`.

Per-frame ordering, which the "never update a landmark in its birth frame"
decision forces:

```text
1. propagate_slam(...) over IMU samples up to the camera timestamp
2. update_stereo_frame(...) with observations of landmarks already in the state
3. inject (inside the update)
4. augment_landmark(...) for observations whose IDs are still unknown
```

Augmenting last also means the new landmark is anchored to the post-update pose,
and it keeps augmentation's offset invalidation outside the sweep.

### Gate 6

- Final position error with updates enabled is materially below the
  propagation-only baseline recorded in Gate 0, on the same trajectory and seed.
- Landmark position estimates converge toward the synthetic truth used by
  `make_synthetic_landmarks()`.
- `P_rr` position and yaw variances shrink when landmarks are observed and grow
  during propagation-only gaps.
- Monte Carlo NEES over `>= 50` seeded runs sits inside the chi-square bounds
  for the robot state dimension. Expect this to be the gate that actually fails
  first — an over-confident yaw is the documented symptom of the linearization
  issue the OC-EKF open decision covers, and it will show in NEES long before
  it shows in position error. If it fails, record the numbers and the
  interpretation; do not tune the gate to pass.
- Deterministic across runs for a fixed seed.

## Step 7: Documentation

### Work

- **`CONVENTIONS.md` §12.** Replace the current two-sentence version. Add the
  frozen-linearization rule, the deferred-injection rule with the
  `nu_i = y_i - H_i delta_x` form, gate-against-prior, and the
  `R_tri`-is-not-`R` warning.
- **`CONVENTIONS.md` §9.** Add the 4-row stereo stacking and the `C` column set.
- **`ARCHITECTURE.md`.** Move "State injection and reset" and
  "Measurement-update covariance form" out of Open decisions into Design
  Decisions with what was built and why. Add the sequential-vs-batch decision
  with the equivalence conditions. Document the new module and public types.
  Leave "Update linearization", "Outlier rejection" (frontend RANSAC is still
  open even with chi-square gating implemented), "Feature frontend", and
  "Initialization" open.
- **`scope.md`.** Check `- [ ] EKF update step`.
- **`BENCHMARKS.md`.** Propagation-only vs. update-enabled error, NEES, and
  per-frame update timing at representative `(N, m)`.
- **`present.md`.** The sequential-vs-batch analysis: the information-form
  equivalence proof, the three traps, the flop and cache-traffic comparison, and
  the batch-oracle test as the verification story. This is the strongest "I
  understood the math rather than copying a formula" material in the project.
- **`CLAUDE.md`.** Update the current-state paragraph and the test count.

### Gate 7

- `ARCHITECTURE.md` describes only what exists.
- No claim in `CONVENTIONS.md` contradicts the implementation.
- Test counts and command output in `CLAUDE.md` match reality.

## Cost Reference

Active dimension `n = 15 + 3N`, `m` observations per frame, all with the sparse
`H` exploited. Dominant terms:

| term | sequential | batch |
|---|---|---|
| `H_i P` | `36mn` | `36mn` |
| `S` | `144m` | `144m^2` |
| factor `S` | `~21m` | `(4m)^3 / 3` |
| `K` | `16mn` | `16m^2 n` |
| covariance update (simple) | `4mn^2` | `4mn^2` |

At `N = 50` (`n = 165`), `m = 20`: sequential ≈ 2.35 MFLOP, batch ≈ 3.58 MFLOP.
Joseph roughly triples the `n^2` term in both. At 20 Hz this is order 100
MFLOP/s — not the constraint.

The real asymmetry is memory traffic. Sequential streams `P` `m` times per
frame, batch about twice. At `n = 165`, `P` is 218 KB and stays in L2 on an
Orin Nano core, so the extra passes are close to free. At `N = 200`
(`n = 615`), `P` is 3.0 MB, falls out of cache, and sequential moves ~120 MB per
frame. **That cache boundary, not the flop count, is the condition under which
this decision should be revisited.** Record measured per-frame update time
against `(N, m)` in `BENCHMARKS.md` so the crossover is observed rather than
assumed.

## Out of Scope

Deliberately not in this plan, each tracked elsewhere:

- Feature detection and tracking, and therefore real data association.
- Raw EuRoC undistortion/rectification. Until it exists, the update runs on
  synthetic rectified stereo only, the same contract `triangulate_stereo(...)`
  already carries.
- Landmark marginalization and the removal policy that decides what leaves the
  map.
- Iterated EKF, OC-EKF, and first-estimates Jacobians. Freezing linearization
  per frame is a prerequisite for FEJ, not an implementation of it.
- Square-root or UD covariance factorization.
- ATE/RPE/NEES as a reusable evaluation module. Step 6 asserts NEES inline;
  the evaluation phase in `scope.md` is separate.
