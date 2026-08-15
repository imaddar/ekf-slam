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
- **Initial `P_rr` is supplied by the caller.** The current IMU-only path does
  not own a default initial covariance; `propagate(...)` receives `P` from its
  caller. The new container must preserve that contract rather than inventing
  an uncertainty value. Zero-initialization is therefore not a valid default.
- The covariance region beyond the active dimension is undefined. It is written
  by augmentation and must never be read before that. NaN-poison it rather than
  relying on zero-fill.

### Gate 1

- `N = 0` matches the old 15x15 path exactly for caller-supplied initial `P_rr`
  values, not merely for dimensions.
- The source of initial `P_rr` is documented, and a test asserts that the
  caller-supplied covariance is preserved exactly.
- Covariance dimensions are correct for multiple `N_max` values.
- The active robot view has the expected dimensions.
- Fixed-size robot-block read/write round-trips through an accessor.
- No public API permits reallocation of the covariance storage.

Active landmark dimensions and landmark-block round trips are deferred to
Step 2, where registry add operations create valid active landmark entries.

Gate 1 passed at commit `b4aefc4`. The residual items below were identified in
review and are carried into Step 1a rather than left implicit.

## Step 1a: Uninitialized-Region Guard and Test Hygiene

Close the one Step 1 work item that was specified but not implemented, and
remove test coverage that asserts less than its name claims.

### Work

- **NaN-fill the region beyond the robot block in every build.** The constructor
  allocates without initializing and writes only `P_rr`, so the landmark region
  currently holds indeterminate values. Zeros were wrong-but-quiet;
  indeterminate values were wrong-and-silent until they reached filter math. An
  unconditional `quiet_NaN` fill makes a premature read fail at the point of
  the bug in the default release-style build as well as in Debug. This runs
  once at container construction, not in the 200 Hz propagation loop.
- **Verify the fill without widening the public API.** Declare the test fixture
  a friend rather than adding a public accessor to storage. A public
  `storage_covariance()` would reopen exactly the hole Step 1 closed.
- **Delete `ActivatingStorageDoesNotReallocate`.** It activates nothing —
  activation does not exist yet — and all three assertions compare pointers to
  the top-left element of the same column-major buffer, which is
  unconditionally true. The real before/after pointer check belongs to Step 2's
  first `add_landmark`.
- **Delete or rewrite `DoesNotExposeResizableCovarianceMember`.** Its body only
  checks `storage_dim()`. Non-exposure is a compile-time property; either make
  it a CMake `try_compile` negative test or drop it and let the private member
  carry the guarantee.
- **Require the initial `NominalState` in `create(...)`.** `NominalState robot{}`
  does not zero its members: Eigen's fixed-size default constructor is
  user-provided, so value-initialization leaves position, velocity, and both
  biases indeterminate. This is the same defect the caller-supplied `P_rr`
  change just fixed for covariance, one field over, and the same argument
  applies — the container should not invent a starting estimate.
- Drop the unnecessary `.template` disambiguator on `topLeftCorner` calls;
  `SlamState` is not a template, so `covariance_` is not type-dependent.
- Note in a comment that the `max_landmarks` bound guards index arithmetic, not
  memory. Any capacity near that bound throws `bad_alloc` out of the
  constructor, which the `ParseResult` contract does not cover.

### Gate 1a

- Every build fills the region beyond `kRobotDim` with `NaN` at construction,
  verified for at least one `N_max >= 1`.
- The test covers both robot-landmark cross strips and the landmark-landmark
  block; the robot block remains finite and caller-initialized.
- No public accessor reaches storage beyond the active dimension.
- `create(...)` requires both initial `P_rr` and initial `NominalState`, and a
  test asserts both are preserved exactly.
- Every remaining `slam_state_test.cpp` case fails if the behavior it names is
  broken. No case passes by tautology.
- The full suite still builds `-Wall -Wextra -Werror` clean and passes.

## Step 2: Landmark Registry and Batch Compaction

Add landmark bookkeeping independently of filter mathematics.

### Work

- **Land the `landmark_block(index)` accessor first, before registry logic.**
  Bound it against `max_landmarks_` (storage capacity), not the active count,
  so it is testable on its own. Two Gate 1 items were deferred here, and
  Architectural Decision "compile-time-sized blocks at runtime offsets" is
  currently unexercised anywhere in the tree. Proving block mechanics separately
  keeps the first registry failure from being ambiguous between offset
  arithmetic and add/remove bookkeeping.
- Store metric XYZ landmark values in the joint state.
- Maintain an explicit `landmark_id -> active state offset` registry.
- Maintain a parallel slot-to-ID index so compaction does not reverse-scan the
  registry. `add_landmark` receives the complete finite new covariance column;
  it never invents zero robot/landmark cross-covariances.
- Landmark offsets are computed only inside the registry. No caller performs
  arithmetic on landmark number to reach a state index.
- Implement batch compaction for removal in place, using fixed-size block
  temporaries rather than full covariance matrices.
- Rebuild offsets after compaction.
- Return an error when removing a missing landmark ID.
- Reject insertion at maximum capacity; do not reallocate or remove another
  landmark implicitly.
- Treat landmark offsets and covariance block views as invalid after add/remove;
  callers resolve them again after the active layout changes.
- Active landmark count updates as a consequence of add/remove, and is not
  settable by any other path.

### Gate 2

Carried over from Gate 1:

- Fixed-size landmark block read/write round-trips correctly at a runtime
  offset, using compile-time block dimensions.
- Adding a landmark does not change the covariance allocation or data pointer —
  captured before and after a real `add_landmark`, not inferred from two views
  of the same buffer.
- The active covariance is fully finite after every add, which is what the
  Step 1a NaN fill exists to detect.
- Removal re-poisons covariance and position storage beyond the new active
  dimension, and empty removal is a no-op.

Registry behavior:

- Interior landmark removal preserves all surviving IDs.
- Landmark state values remain unchanged.
- Covariance blocks for surviving landmarks remain unchanged.
- Batch removal performs one in-place compaction operation.
- Missing IDs fail loudly.
- Add/remove/add behavior is deterministic.
- Augmentation attempted at `N_max` follows a documented policy — reject, or
  trigger removal first — rather than asserting or reallocating.

Gate 2 passes with this Step 2 implementation. The next stage is Step 3: joint
covariance propagation.

## Step 3: SLAM Covariance Propagation

Extend prediction to the joint covariance.

### Work

- **Land the cross-covariance accessors first, before propagation math.** The
  container currently exposes `robot_covariance()` (15x15), `landmark_block(i)`
  (one diagonal `3x3`), and `active_covariance()` (everything). Nothing reaches
  the `15 x 3N` robot-landmark strip that `P_rl` propagation needs, so the first
  `Phi * P_rl` line would either add an accessor or index into
  `active_covariance()` with hand-computed offsets — the arithmetic Step 2
  exists to prevent. Add:

  ```text
  robot_landmark_covariance()      15  x 3N
  landmark_landmark_covariance()   3N  x 3N
  ```

  Both are runtime-sized views of the active region bounded by
  `active_landmarks_`; neither adds storage. Landing them first keeps the first
  propagation failure unambiguous between the transition math and the indexing,
  the same reason `landmark_block` led Step 2.
- Keep the symmetric `P_rl`/`P_lr` write inside the container with a setter, so
  propagation and later augmentation cannot update one strip without the other.
- Gate 3's `P_ll` bit-identity check needs the full landmark-landmark
  submatrix, including `i != j` cross blocks. `landmark_block(i)` returns only
  diagonals, so without the second accessor the test reaches through
  `SlamStateTestAccess` or recomputes offsets itself.

For each IMU step:

- Propagate `P_rr` using the existing robot transition.
- Propagate `P_rl` by left-multiplying with the robot transition.
- Leave `P_ll` unchanged.
- Never construct a full augmented transition matrix.

### Gate 3

- Both cross-covariance accessors exist, are bounded by the active landmark
  count, and are the only path propagation uses to reach those blocks.

- `P_ll` is bit-identical after prediction.
- `P_rl` matches an explicit augmented-matrix reference implementation.
- `P_rr` matches the existing IMU-only result.
- The covariance remains symmetric.
- `P_rr` is positive **definite** — with process noise present, the weaker PSD
  check would mask a problem. Assert the stronger property where it holds.
- The full joint covariance remains positive semidefinite.
- `N = 0` remains numerically identical to the old propagation path.
- The 200 Hz propagation path uses fixed-size per-landmark temporaries and does
  not allocate a dynamic `15 x 3N` covariance product.

Gate 3 passes with the active covariance accessors, shared robot transition,
joint block propagation, and dedicated reference/PSD tests. The next stage is
Step 4: stereo forward-map geometry.

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

Gate 4 passes with the validated metric XYZ frame maps and identity,
round-trip, nonzero-extrinsic, and calibration-driven tests. The next stage is
Step 5: augmentation Jacobians.

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

### Derivation for Augmentation Jacobians
### Derivation

Forward map:

$$l^W = R_{WB}\left(R_{BS}\,l^C + p_{BS}\right) + p_{WB}$$

Define the landmark in body coordinates:

$$l^B \triangleq R_{BS}\,l^C + p_{BS}$$

so that $l^W = R_{WB}\,l^B + p_{WB}$.

#### Position block

$p_{WB}$ enters additively and appears nowhere else:

$$\frac{\partial l^W}{\partial \delta p} = I_{3\times3}$$

#### Orientation block (right/local)

Perturb the nominal rotation on the right:

$$R_{WB} = \hat{R}_{WB}\,\mathrm{Exp}(\delta\theta) \approx \hat{R}_{WB}\left(I + [\delta\theta]_\times\right)$$

Substitute and expand:

$$l^W \approx \hat{R}_{WB}\left(I + [\delta\theta]_\times\right) l^B + \hat{p}_{WB} + \delta p$$
$$= \underbrace{\hat{R}_{WB}\,l^B + \hat{p}_{WB}}_{\text{nominal}} \;+\; \hat{R}_{WB}[\delta\theta]_\times l^B \;+\; \delta p$$

Dropping the zeroth-order term leaves the first-order error:

$$\delta l^W = \delta p + \hat{R}_{WB}[\delta\theta]_\times\, l^B$$

$\delta\theta$ is trapped in the middle, so this is not yet a Jacobian. Apply
$[a]_\times b = -[b]_\times a$:

$$\hat{R}_{WB}[\delta\theta]_\times l^B = -\hat{R}_{WB}[l^B]_\times \delta\theta$$

giving

$$\frac{\partial l^W}{\partial \delta\theta} = -\hat{R}_{WB}\,[l^B]_\times$$

Note the skew-symmetrized vector is $l^B$ — not $l^C$ (wrong frame, before the
extrinsic) and not $l^W$ (after the translation, which the rotation does not
act on).

#### Zero blocks

$\delta v$, $\delta b_g$, $\delta b_a$ do not appear in the forward map:

$$\frac{\partial l^W}{\partial \delta v} = \frac{\partial l^W}{\partial \delta b_g} = \frac{\partial l^W}{\partial \delta b_a} = 0_{3\times3}$$

#### Assembled J_r (3 × 15)

The implemented propagation ordering is $[\delta p,\ \delta v,\ \delta\theta,\ \delta b_a,\ \delta b_g]$:

$$J_r = \begin{bmatrix} I_{3} & 0_{3} & -\hat{R}_{WB}[l^B]_\times & 0_{3} & 0_{3} \end{bmatrix}$$

#### J_lC (3 × 3)

Read directly off the forward map — the composition acting on $l^C$:

$$J_{l^C} = \frac{\partial l^W}{\partial l^C} = R_{WB}\,R_{BS}$$
### Gate 5

- Analytical and numerical `J_r` agree to approximately `1e-6`.
- Analytical and numerical `J_lC` agree to approximately `1e-6`.
- Position and zero blocks are structurally exact.
- Orientation perturbations produce the expected skew structure.
- The convention is verified against `propagation.cpp`.

Gate 5 passes with analytical derivatives, finite-difference checks, exact
sparsity checks, and the propagation ordering correction above. The next stage
is Step 6: triangulation uncertainty.

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

### Work — carried from the Step 2 review

- **Stop `add_landmark` from overwriting the caller's `P_ll`.** The column and
  row writes overlap on the new landmark's own `3x3` corner, and the row write
  lands second, so the stored block is the transpose of what the caller
  supplied. Harmless for a symmetric input, which augmentation produces, but the
  result depends on statement order and a grossly asymmetric block is accepted
  silently. Write only the off-diagonal strip in the second write so the corner
  has one writer.
- **Take the covariance column by `Eigen::Ref<const Eigen::MatrixXd>`.** The
  current `const Eigen::MatrixXd&` materializes a temporary when bound to the
  `P_rr J_r^T` expression, adding a heap allocation to every landmark birth —
  the one path this step exists to keep cheap.

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

### Work — carried from the Step 2 review

- **Narrow compaction's column phase to survivor rows.** It currently walks all
  `active_landmarks_` rows when only survivor rows are read back by the row
  phase; the rest are either overwritten there or NaN-poisoned. Restricting the
  loop to `survivor_indices` drops the block-copy count from `k*N` to `k*k`.
  Do this before removal cost is measured, since these numbers land in
  `BENCHMARKS.md`.

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

Gate 1a passed in commits `a807817` and `56f4b03`. Gate 2 passed at `ad3da23`,
which replaced zero-filled landmark covariance with a caller-supplied column and
made compaction in-place. Step 3 is the next gated stage, and begins with the
cross-covariance accessors rather than the transition math. Two Step 2 review
items were deliberately deferred to the steps that already touch that code:
the `add_landmark` corner-write ordering and its `Eigen::Ref` signature to
Step 7, and compaction's column-phase row bound to Step 8.
