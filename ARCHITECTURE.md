# Architecture

This document describes the architecture of what is currently implemented in
`ekf-slam`. For the full project plan (weekly breakdown, deliverables,
out-of-scope items, and the target filter/system design that hasn't been built
yet) see [scope.md](scope.md).

## What exists

```
CMakeLists.txt      C++ build and GoogleTest test configuration
BENCHMARKS.md       Benchmark and metric registry
parser.hpp          Public C++ parser API declaration
parser.cpp          Top-level dataset loading orchestration
parser_csv.hpp/cpp  Internal CSV parsing and stereo frame pairing
parser_yaml.hpp/cpp Internal EuRoC calibration YAML parsing
propagation.hpp/cpp Public IMU nominal-state propagation
state.hpp           Public nominal ESEKF state and covariance types
slam_state.hpp      Public preallocated SLAM state/covariance storage container
synthetic.hpp/cpp   Public synthetic analytic trajectory and IMU generator
types.hpp           C++ parser output type declarations
roadmap.md          Planned parser API direction
docs/parser.md      Parser implementation notes and future direction
present.md          Technical deep-dive presentation material
tests/parser_test.cpp  C++ parser tests
tests/propagation_test.cpp  C++ propagation tests
tests/state_test.cpp   C++ state-header compile tests
tests/slam_state_test.cpp  SLAM state storage tests
tests/synthetic_test.cpp Synthetic trajectory and IMU propagation tests
```

There is a public nominal ESEKF state layout, IMU-only state covariance type,
preallocated SLAM state storage container, and IMU nominal-state and covariance
propagation. The full error-state struct, landmark registry, state augmentation,
and measurement update do not exist yet. A synthetic data harness exists for
controlled pre-EuRoC validation of propagation behavior.

## C++ root skeleton

`parser.hpp` exposes only the top-level EuRoC dataset loader. `parser.cpp`
orchestrates top-level EuRoC sequence loading into `Dataset`. `parser_yaml.cpp`
implements private EuRoC camera and IMU calibration YAML parsing.
`parser_csv.cpp` implements private IMU measurement CSV parsing, ground-truth
CSV parsing, camera CSV parsing, and stereo-pair matching.

`state.hpp` defines `NominalState` with position, velocity, Sophus SO(3)
orientation, accelerometer bias, and gyroscope bias fields. It also defines
`ImuStateCovariance`, the 15x15 covariance used by the current IMU-only
propagation path. `StateCovariance` remains as a compatibility alias for the
existing tests and call sites. This fixed covariance is a development and
testing artifact for the robot state, not the intended long-term SLAM state
container: once landmarks enter the state, the filter should own a joint state
container with a preallocated maximum-size covariance and a tracked active
dimension.

`slam_state.hpp` defines `SlamState`, the storage-only joint state container for
the upcoming landmark work. `SlamState::create(...)` requires the caller to
provide the initial 15x15 robot covariance because the current IMU-only
propagation API does not define a default initial uncertainty. The container
owns a `NominalState`, a private preallocated dynamic covariance matrix, a
maximum landmark budget, and an active landmark count that is reserved for
registry operations. The allocated covariance dimension is fixed at
construction as `15 + max_landmarks * 3`; the active robot view is exposed
through accessors. It also defines `kRobotDim = 15` and `kLandmarkDim = 3`. It
does not yet own landmark IDs, landmark positions, compaction, propagation, or
augmentation math.

`propagation.hpp` exposes `PropagationResult` and `propagate(...)`, which
returns `ParseResult<PropagationResult>`. The propagation function takes a
nominal state, one IMU measurement, IMU calibration, a timestep in seconds, and
a 15x15 state covariance matrix. It rejects non-finite and negative timesteps,
accepting a zero timestep as a no-op. It removes accelerometer and gyroscope
biases, treats IMU acceleration as body-frame specific force, adds world-frame
gravity, and updates position, velocity, and Sophus SO(3) orientation.
Covariance is propagated with a first-order discrete transition matrix derived
from the continuous error-state dynamics, plus first-order process noise
`Q_d = G Q_raw G^T dt` from the IMU noise densities and bias random walks.

`synthetic.hpp` exposes `SyntheticTrajectory`, `SyntheticImuConfig`,
`SyntheticLandmark`, `SyntheticCameraConfig`, `SyntheticStereoObservation`,
`make_synthetic_imu_calibration(...)`,
`make_synthetic_pinhole_camera_calibration(...)`,
`make_synthetic_landmarks()`, `sample_ground_truth(...)`,
`synthesize_imu(...)`, and `synthesize_stereo_observations(...)`.
`SyntheticTrajectory::state_at(t)` is the continuous truth interface. It
currently models closed-form constant world acceleration with optional
sinusoidal world acceleration and constant body-frame angular velocity from
configurable initial pose, velocity, biases, and start timestamp. The trajectory
alone owns the time origin and the true biases; the per-stream configs carry only
rate, duration, and noise, so ground truth, IMU, and camera samples cannot land
on different conventions. `SyntheticTrajectory::time_at(timestamp)` inverts the
sample timestamp mapping so callers can look up truth at the instant a sample
actually landed on.

Samples land on a `1 / rate_hz` grid from the trajectory start. The step count
rounds to the nearest whole step before truncating, because products such as
`0.29 * 200` are not exactly representable and would otherwise drop the final
sample. A duration that is genuinely not a whole number of steps truncates to the
last grid point on or before it.

IMU synthesis emits body-frame specific force using the same gravity convention
that propagation expects, plus the trajectory's fixed accelerometer and gyroscope
biases. Stereo synthesis projects world landmarks into two pinhole cameras using
a rigid inverse of `T_BS` precomputed once per run, rejects behind-camera and
out-of-frame points, and rejects disparity at or below the configured minimum.
Optional IMU and pixel noise are deterministic and seed-driven. All three
generators validate their configuration and return `ParseResult<T>`.

`tests/parser_test.cpp` contains the GoogleTest coverage for the C++ parser.
Current coverage validates those behaviors through the public `parse_dataset`
entry point: successful camera/IMU YAML parsing, camera calibration
transform-shape rejection, successful IMU/ground-truth/stereo CSV parsing, IMU
and ground-truth field-count rejection, stereo timestamp mismatch rejection, and
top-level dataset loading from both a temporary EuRoC-like directory and the
checked-in `datasets/machine_hall/MH_01_easy` sequence.
`tests/propagation_test.cpp` verifies stationary behavior, acceleration
integration, orientation integration, bias removal, covariance transition
behavior, process-noise block placement, symmetry, and positive
semi-definiteness. It also rejects negative and non-finite timesteps, accepts a
zero timestep as a no-op, and cross-checks the propagated covariance against a
Monte Carlo error covariance. It also smoke-tests one second of IMU-only
propagation on the checked-in `MH_01_easy` sequence against nearby ground truth
with loose sanity bounds.
`tests/state_test.cpp` verifies that `state.hpp` exposes the nominal-state
member types and the IMU-only covariance alias boundary.
`tests/slam_state_test.cpp` verifies explicit robot-covariance initialization,
capacity validation, stable robot covariance views, fixed-size robot covariance
access, and the absence of a public resizable covariance member. Landmark
activation and landmark-block tests belong to the registry stage.
`tests/synthetic_test.cpp` verifies synthetic IMU timestamp spacing, sample-grid
retention and truncation, ground-truth sampling, a shared time origin across
streams, bias plumbing from trajectory to stream, invalid-configuration
rejection, the stationary gravity convention, stationary propagation,
constant-acceleration propagation, constant-yaw propagation, fixed-bias
propagation, and a fully excited translation-plus-rotation trajectory against
analytic truth. It also checks timestep-error scaling, varied landmark depths,
hand-computed stereo projection, visibility rejection, disparity-threshold
rejection, non-rectified rig rejection, seeded noise reproducibility, and
agreement between injected noise and the calibration densities.

`types.hpp` defines the intended C++ parser output data structures:

- `TimestampNs` — alias for `std::uint64_t` nanosecond timestamps.
- `CameraCalibration` and `ImuCalibration` — YAML calibration output structs.
- `StereoPair`, `ImuMeasurement`, and `GroundTruthState` — parsed sensor and
  label records.
- `Dataset` — top-level parsed dataset containing `sequence_root`, the
  calibration structs, stereo pairs, IMU measurements, and ground-truth states.

### Entry Points

- `parse_dataset(sequence_root)` — public. Loads the standard EuRoC `mav0`
  directory layout into `Dataset`.
- `SyntheticTrajectory::state_at(time_seconds)` — public. Returns analytic
  ground truth at a continuous time.
- `SyntheticTrajectory::time_at(timestamp)` — public. Returns elapsed trajectory
  time for a timestamp the trajectory generated.
- `sample_ground_truth(trajectory, rate_hz, duration_seconds)` — public.
  Samples analytic truth into project-native `GroundTruthState` records.
- `synthesize_imu(trajectory, config)` — public. Samples project-native
  `ImuMeasurement` records from analytic truth.
- `make_synthetic_imu_calibration(rate_hz, noise)` — public. Creates an IMU
  calibration whose noise densities match the noise the generator injects.
- `make_synthetic_pinhole_camera_calibration(...)` — public. Creates a
  distortion-free camera calibration for synthetic projection tests.
- `make_synthetic_landmarks()` — public. Returns a small deterministic landmark
  set with varied depths.
- `synthesize_stereo_observations(...)` — public. Samples `SyntheticStereoObservation`
  records from analytic truth and a rectified stereo camera calibration.
- `SlamState::create(max_landmarks, initial_robot_covariance)` — public.
  Fallibly allocates a joint covariance sized for the maximum metric-XYZ
  landmark budget and preserves the caller-provided robot covariance exactly.

Lower-level YAML, CSV, and stereo-pair parsing functions are private
implementation details in `parser_yaml.cpp` and `parser_csv.cpp`. Planned future
reader APIs are tracked in [roadmap.md](roadmap.md), but are not implemented.

## Design Decisions

This section records the intentional choices behind the code that exists, the
alternatives each one was picked over, and what the choice costs. Decisions that
have not actually been made yet are collected under
[Open decisions](#open-decisions) and marked as such, so this section can be read
as a design rationale without implying the code does more than it does.

### How the decisions fit together

Three commitments drive most of the rest:

1. **Errors are values, never panics.** `ParseResult<T> = std::expected<T,
   std::string>` is the single failure channel from the parser up through
   propagation and the synthetic generators. This is what lets the same code run
   under a test harness today and inside a real-time loop on Jetson later,
   where an exception or an abort inside Sophus is not an acceptable outcome.
2. **Correctness is pinned by generated truth, not by eyeballed numbers.** The
   synthetic harness produces trajectories whose closed-form state is known
   exactly, so propagation error can be measured rather than inspected. EuRoC is
   then used only as a realism check on top of that.
3. **Cheap-and-analyzable before exact.** Every numerical choice so far — first
   order `Phi`, first order `Q_d`, constant-input integration — is the simplest
   form whose error is understood and measured. Each has a recorded upgrade path
   and a benchmark that would show the upgrade paying off.

### Filter formulation

- **Error-state EKF over a direct EKF.** The nominal state carries an SO(3)
  rotation, which has no minimal global parameterization; the error state is a
  15-dimensional vector in the tangent space, where a covariance is well defined
  and `F` is a plain matrix. The direct-EKF alternative — covariance over a
  4-component quaternion — needs constraint maintenance and a singular
  covariance, both of which the error-state form removes. Cost: two states to
  keep coherent and an injection/reset step at every update, which is not yet
  written because there are no updates yet.
- **Local (right) error convention: `R_sample = R_nominal * Exp(dtheta)`.** This
  matches the right-multiplied nominal update in `propagate(...)`, and it makes
  the rotation error body-fixed, so gyro bias enters the orientation error
  dynamics as a constant `-I` block rather than a rotation-dependent one. The
  global/left convention is equally valid and common in the literature; mixing
  the two silently transposes blocks of `F`, which is precisely what the Monte
  Carlo test exists to catch.
- **15-state: position, velocity, orientation, accel bias, gyro bias.** No
  gravity magnitude, no IMU-camera extrinsics, no time offset in the state.
  EuRoC provides hardware-synchronized data and factory extrinsics, so
  estimating them online would add unobservable-ish dimensions before the basic
  filter works. Online extrinsic/time-offset calibration is a known extension,
  deliberately deferred.

### Numerical choices in propagation

- **`F` is continuous-time dynamics; `Phi` is the discrete step matrix.** `F`
  says how an infinitesimal error state changes per second at the current
  nominal state. It has units of "per second" in its non-identity coupling
  blocks. `Phi` says how the error moves across one actual IMU interval, so it
  is dimensionless and appears directly in `P_new = Phi P Phi^T + Q_d`. The
  current implementation uses the first-order discretization `Phi = I + F dt`.
  When landmarks are present, only the robot block has dynamics during IMU
  propagation: `P_rr` uses `Phi_rr P_rr Phi_rr^T + Q_d`, robot-landmark
  covariance uses `Phi_rr P_rl`, and landmark-landmark covariance is unchanged.
- **Covariance transition `Phi = I + F dt`, first order.** Alternatives are the
  matrix exponential of `F dt`, a truncated series to second or third order, or
  Van Loan's method (which yields `Phi` and the exact `Q_d` from a single
  matrix exponential). At 200 Hz the truncation error is small but *measurable*:
  it is the dominant term in the Monte Carlo consistency deviation (~0.04
  normalized), not sampling noise. The upgrade is cheap to make and the Monte
  Carlo score is the metric that would demonstrate it.
- **Process noise `Q_d = G Q_raw G^T dt`, first order.** The exact form is
  `integral of Phi(tau) G Q G^T Phi(tau)^T dtau`, which adds `dt^2` and `dt^3`
  cross terms — notably a position/velocity correlation that the first-order
  form omits entirely, leaving the position block of `Q_d` exactly zero. That is
  visible in `AddsFirstOrderProcessNoiseFromImuCalibration`. At 5 ms steps the
  omitted terms are small relative to the `Phi P Phi^T` term, and the practical
  consequence — a slightly optimistic position covariance — is exactly what a
  NEES benchmark would expose once updates exist.
- **Nominal integration is constant-input over the step**, with the second-order
  `0.5 a dt^2` term kept for position. Alternatives: first-order position
  (cheaper, larger error), midpoint/trapezoidal on the IMU pair, RK4, or a
  proper pre-integration formulation. The measured convergence is first order
  overall — halving `dt` halves the error, ratio `0.4999` at 200/400/800 Hz —
  so the second-order position term is not buying an order of accuracy; it is
  buying a constant factor for one multiply. Midpoint integration would be the
  first upgrade worth benchmarking, and there is a TODO in `propagation.cpp` to
  that effect.
- **Gravity is a hard-coded `[0, 0, -9.81]` world vector.** Alternatives: a
  configurable constant, a latitude-dependent WGS-84 model, or estimating
  gravity direction as part of the state. For a single indoor dataset the
  constant is fine; the value has to agree between `propagation.cpp` and
  `synthetic.cpp`, and today it is duplicated in both as a file-local constant
  rather than shared, which is a small latent hazard.
- **Timestep validation instead of trust.** `propagate(...)` rejects non-finite
  and negative timesteps. A negative `dt` — one out-of-order IMU sample — runs
  the covariance update backwards, subtracting information instead of adding it:
  a single reversed 5 ms step lowers `trace(P)` below its starting value, and
  repeated reversed steps drove the minimum eigenvalue to `-2.4e-4`, leaving `P`
  indefinite and any Kalman gain derived from it meaningless. A zero `dt` is an
  exact no-op and stays legal, since duplicate IMU timestamps are benign.
  Detecting IMU *gaps* (a `dt` far larger than the nominal sample period) is a
  separate policy in [scope.md](scope.md) and is not yet implemented.
- **Symmetry is not enforced after the update.** `Phi P Phi^T + Q_d` is
  symmetric in exact arithmetic and tests confirm it holds to `1e-8` over a
  1 s EuRoC window, so the usual `P = 0.5 (P + P^T)` symmetrization and the
  Joseph-form update are not used yet. Both are standard defenses once
  measurement updates start subtracting information from `P`, which is where
  asymmetry and loss of positive definiteness actually appear.

### State and math library

- **Sophus `SO3d` for rotation, Eigen for everything else.** Sophus supplies
  `exp`/`log` and the group operations with a tested convention, which is the
  part that is easy to get subtly wrong by hand. The alternatives were rolling
  the rotation vector math directly (more code to verify, no benefit) or
  carrying `SE(3)` for the pose. `SO(3)` plus a separate position vector was
  chosen over `SE(3)` because velocity is a separate state anyway and the
  `SE(3)` tangent couples translation into the rotation exponential, which makes
  the `F` blocks harder to read against the standard VIO literature.
- **`NominalState` is a plain aggregate, not a class with invariants.**
  Designated-initializer construction keeps the propagation step readable and
  makes the field order explicit at every call site. There is no constructor
  enforcing, for example, a normalized quaternion, because Sophus owns that.
- **Fixed 15x15 `ImuStateCovariance` (`Eigen::Matrix<double, 15, 15>`).** Fixed
  size means stack allocation and full inlining, and it is what makes a
  propagation step ~1.9 us. The older `StateCovariance` name is kept only as a
  compatibility alias for the current IMU-only path; new SLAM code should use
  explicit robot/joint-state names. The landmark filter will use a joint
  covariance allocated once at
  `(15 + N_max * 3) x (15 + N_max * 3)` and operate only on the active
  `15 + active_landmarks * 3` top-left block.
- **`double` throughout.** Float would halve memory traffic and vectorize wider
  on the Jetson, but covariance propagation over long horizons is exactly where
  precision loss shows up as a non-positive-definite `P`. Revisit only with a
  profile that says it matters.

### Planned landmark-state decisions

These choices are not implemented yet, but they are settled design direction for
the next state-augmentation work and replace earlier inverse-depth planning.

- **Metric XYZ landmarks from stereo triangulation.** Each landmark will add a
  3-dimensional Euclidean position in the world frame. This directly matches the
  stereo harness and EuRoC's synchronized stereo setup, keeps Jacobians compact,
  and makes state growth predictable. Inverse depth was the original plan
  because it better conditions far points and usually gives a more Gaussian
  depth error, especially for monocular or long-range initialization. For this
  bounded stereo project, its extra parameters, anchored-frame bookkeeping, and
  more subtle Jacobians are not worth the complexity. If the project later moves
  to larger spatial scale, long-range landmarks, or monocular initialization,
  inverse depth should be reconsidered.
- **Classic EKF-SLAM with a bounded active map.** Landmarks will live in the
  filter state, so the covariance grows as `O(n^2)` in the active landmark
  count. This is intentional for the first full SLAM implementation because it
  keeps the estimator math direct and testable. MSCKF remains the standard
  bounded-compute alternative for production VIO, but switching to it would be a
  structural change, not an incremental refactor.
- **Preallocated joint covariance with active dimension.** The SLAM state should
  allocate the full maximum covariance once, using
  `(15 + N_max * 3) x (15 + N_max * 3)` storage. Landmark insertion changes the
  active dimension and writes into already-allocated blocks; it should not
  resize the matrix in the IMU or camera update path. `SlamState` implements the
  storage and active-dimension foundation of this decision. Covariance storage
  is private so callers cannot resize it, and the initial robot covariance is
  supplied explicitly because the current IMU-only propagation path has no
  default uncertainty. Landmark insertion and removal policy are not
  implemented yet.
- **ID-based landmark registry.** Landmark IDs are external identities, not
  state indices. All landmark state and covariance access should go through an
  `id -> state offset` registry so data association and removal cannot silently
  corrupt covariance blocks.
- **Batch compaction for removal.** Landmark removal will compact all survivors
  in one pass and rebuild the registry. A free-list design would keep offsets
  stable, but it pushes holes into every propagation, update, and debugging path.
  Dense active storage keeps `active_dim` meaningful and is the simpler choice
  unless profiling later proves compaction is a bottleneck.
- **Shared robot propagation math, separate public paths.** The current
  `propagate(...)` API can remain as the IMU-only test/development path while
  SLAM propagation grows a joint-state API. Both should call the same internal
  robot transition/noise builder so `F`, `Phi`, and `Q_d` are not duplicated.
  Putting every mode into one public function would work mechanically, but it
  would blur two different contracts: fixed 15x15 IMU-only propagation versus
  joint covariance propagation with landmark cross-block invariants.

### Error handling and API shape

- **`std::expected<T, std::string>` over exceptions or error codes.** Exceptions
  in a hard-deadline IMU thread are the thing being avoided; error codes lose
  the message. The tradeoff accepted is an allocation on the error path and a
  string that a caller cannot programmatically branch on. If the pipeline later
  needs to distinguish "bad file" from "unusable dataset", the error type needs
  to become a struct, and `roadmap.md` already flags that for validation.
- **One narrow public parser entry point, `parse_dataset(...)`.** YAML, CSV, and
  stereo-pairing helpers are private in `parser_detail`, so the file format
  stays an implementation detail and downstream code has exactly one way to load
  a sequence. The cost is that the whole sequence materializes in memory
  (MH_01: 36,820 IMU samples, 3,682 stereo pairs, 36,382 ground-truth states),
  which is fine offline and wrong for the real-time pipeline; streaming readers
  are sketched in [roadmap.md](roadmap.md).
- **Hand-written YAML and CSV parsing rather than a dependency.** EuRoC's
  `sensor.yaml` uses a small, fixed subset of YAML, and the hand-written version
  is what makes error messages name the field, the line, and the expected vs.
  found shape (`"T_BS must be 4x4, got 4x3"`). A general library would parse more
  of YAML correctly and cost less code, but would report file-format errors in
  its own vocabulary and add a dependency to cross-compile. This choice is only
  defensible while the input stays EuRoC-shaped.

### Synthetic data harness

- **In-memory structs, not generated EuRoC folders.** Filter tests stay focused
  on estimator behavior rather than file-format or parser behavior, and they run
  without touching a disk. The alternative — writing synthetic sequences to a
  temp directory — would additionally exercise the parser, which already has its
  own tests. IMU and ground-truth output reuse the project-native `types.hpp`
  records; stereo output uses a harness-local `SyntheticStereoObservation`
  because no measurement type exists yet.
- **Truth is continuous through `state_at(t)`; sampled truth is derived from
  it.** This avoids interpolation error when future camera timestamps do not
  align with IMU timestamps — a real problem with EuRoC ground truth, which is
  itself a sampled estimate.
- **The trajectory owns the time origin and the true biases.** Duplicating them
  onto the per-stream configs let ground truth and the IMU stream disagree
  silently, which is exactly the failure a bias-estimation or time-alignment
  test is supposed to catch.
- **IMU synthesis emits specific force**, `R_WB^T * (p_ddot_W - [0, 0, -9.81])`,
  so a stationary identity-oriented body reads `[0, 0, 9.81]`, matching
  `propagate(...)`. A sign or convention mismatch here would make every
  propagation test agree with a wrong filter.
- **Noise is configured as continuous-time density, not per-sample stddev.** The
  generator discretizes with `sigma_d = sigma_c * sqrt(rate)` and
  `make_synthetic_imu_calibration(...)` fills the calibration from the same
  config, so injected noise and the filter's `Q` describe one process — the
  precondition for a meaningful NEES number. Specifying per-sample stddev would
  have made the two silently rate-dependent.
- **Box-Muller over raw `mt19937_64` bits rather than
  `std::normal_distribution`,** whose engine-to-variate mapping is
  implementation-defined. Seeded output therefore reproduces across libc++ and
  libstdc++, which matters once Phase 2 cross-compiles to Jetson. The cost is a
  hand-written sampler (and slightly slower draws) in exchange for reproducible
  test failures across machines.
- **Generators validate configuration and return `ParseResult<T>`.** A zero rate
  previously produced NaN sample times and aborted inside Sophus, and a negative
  duration produced a huge `reserve`. Both are now errors, matching the
  no-panics policy for library code.
- **The trajectory model is closed-form: constant world acceleration, optional
  sinusoidal world acceleration, constant body-frame angular velocity.** That is
  enough to exercise translation, rotation, gravity, bias removal,
  rotation-translation coupling, and timestep convergence while keeping the
  analytic truth hand-checkable. A richer model — time-varying angular velocity,
  a spline through EuRoC ground truth — would be more realistic but would need
  numerical differentiation to produce IMU data, reintroducing the very error
  the harness is meant to be free of.
- **Stereo synthesis produces estimator-facing numeric observations, not
  images.** This keeps state-augmentation and measurement-model tests separate
  from feature-tracking realism; the frontend gets tested on real EuRoC imagery
  instead. Consequence: the harness cannot exercise data-association failures,
  which is where a real VIO frontend usually breaks.
- **A rectified rig is required — shared orientation, positive x baseline.**
  `u0 - u1` is only a disparity under that geometry, and without the check a
  rotated or y-baseline rig silently produced zero observations,
  indistinguishable from "nothing was visible".

### Testing and verification strategy

- **Monte Carlo consistency is the primary covariance gate**, not block-value
  assertions. Hand-checked block values are only ever written for simple states
  — identity orientation, zero angular velocity — which makes `R`
  indistinguishable from `R^T` and zeroes the `[w]x` block, so they cannot
  detect a transposed or sign-flipped entry in `F`. The Monte Carlo test
  propagates perturbed trajectories and compares the sample covariance against
  the filter's, normalized by the filter's own standard deviations so every
  block is judged on one scale and the comparison stays signed. It starts from a
  non-trivial `P0`: with zero initial bias uncertainty the bias-coupling blocks
  of `F` carry no weight and a sign flip in them is invisible.
- **The Monte Carlo tolerance is set by truncation, not sampling.** The floor is
  `Phi = I + F dt` error, so it does not tighten with more samples. Correct code
  scores ~0.03–0.04 across seeds and corrupting any block of `F` scores >= 0.23,
  which is what sets the 0.15 threshold. The tradeoff is a ~1 s test; an
  analytic Jacobian check against finite differences would be faster but would
  verify `F` against itself rather than against propagated behavior.
- **Synthetic gates are exact, EuRoC gates are loose.** Synthetic tests assert to
  `1e-9` because the truth is closed form; the EuRoC smoke test asserts only
  `< 2.0 m` over 1 s because ground truth, calibration, and time alignment all
  carry their own error. Keeping them apart means a synthetic failure always
  means a bug, and an EuRoC failure means something about the data pipeline
  changed. Measured EuRoC values sit far inside those bounds (0.017 m at 1 s),
  and they are recorded in [BENCHMARKS.md](BENCHMARKS.md) so the gap stays
  visible.
- **Sample-grid rounding is treated as a correctness concern, not a detail.**
  Products such as `0.29 * 200` evaluate to `57.999999999999993`, and a plain
  `floor` dropped the final sample — which showed up as a 17x worse propagation
  error (`0.009` vs `0.0005`) that looked like an integrator bug and was not.

### Build and tooling

- **`RelWithDebInfo` is the default build type.** Eigen depends on optimization
  to inline its expression templates; the unoptimized default made the test
  suite ~50x slower and put the Monte Carlo check out of reach at a useful
  sample count. Debug info is kept so the optimized build is still debuggable.
- **`-Wall -Wextra -Werror` on every target.** Cheap, and warnings in numeric
  code are frequently real (unused results, narrowing, sign compares).
- **Eigen, Sophus, and GoogleTest come from the system via `find_package`,** not
  vendored or fetched. This keeps the build fast and reproducible on the dev
  machine; the Jetson cross-compile in Phase 2 is where this will need
  revisiting, since `CMAKE_PREFIX_PATH=/opt/homebrew` is a host-specific
  assumption.
- **One library target, `ekf_slam_parser`, holding parser, propagation, and
  synthetic code.** The name is now inaccurate — it stopped being parser-only
  when propagation landed. Splitting into `ekf_slam_core` plus a test-only
  synthetic target would also stop the synthetic generators from shipping into
  the eventual ROS 2 node.
- **C++23.** `std::expected` is the reason; `std::format` and designated
  initializers are used throughout as a result. The cost is toolchain
  sensitivity, which will be felt first on the Jetson's default compiler.
- **A `TRY` macro for error propagation** in `parser.cpp`, `parser_csv.cpp`, and
  `parser_yaml.cpp`, `#undef`ed after use. It is a macro because C++ has no `?`
  operator; the alternative is ~5 lines of boilerplate per fallible call.

### Open decisions

These are not implemented and not settled. They are recorded here so the
tradeoffs are visible before the code exists, and each should move into the
sections above (or out of this file) once it is actually built.

- **Update linearization.** EKF vs. iterated EKF vs. an observability-constrained
  variant. Standard VIO EKFs gain spurious yaw observability from linearizing
  about different states at different times; OC-EKF or a first-estimates-Jacobian
  scheme fixes it. This will show up as an over-confident yaw covariance in NEES
  before it shows up in ATE.
- **Outlier rejection.** Chi-square gating on the innovation is the default
  choice; RANSAC in the frontend, robust cost functions, or per-landmark health
  tracking are alternatives. Nothing is implemented, and a VIO filter without
  gating fails on the first bad match.
- **Feature frontend.** [scope.md](scope.md) leaves KLT vs. descriptor matching
  open. KLT is cheaper and gives sub-pixel tracks on high-rate imagery;
  descriptors survive larger baselines and re-detection. This choice interacts
  with the landmark parameterization and with the Jetson budget.
- **Measurement-update covariance form.** The landmark-state plan uses a
  preallocated dense covariance, but the update formula itself is still open:
  standard covariance update, Joseph form, or eventually a square-root/UD
  factorization. Joseph form is the likely first upgrade once camera updates
  start subtracting information from `P`.
- **State injection and reset.** After each update the error state is injected
  into the nominal state and reset to zero, which requires a covariance
  reset Jacobian for the rotation block. Skipping the Jacobian is common and
  usually a small error; it is a decision, not an oversight, and should be
  recorded when the update lands.
- **Initialization.** The EuRoC smoke test starts from ground truth, which a
  real system does not have. Static-start bias and gravity-direction estimation,
  or a short visual-inertial alignment, are the standard options and neither
  exists.
- **Threading and time handling for Phase 2.** Lock-free queue vs. mutex-guarded
  buffer for IMU handoff, and how out-of-order or delayed camera frames are
  handled (buffer and reprocess, or drop). The current `propagate(...)` contract
  — reject negative `dt` — assumes strictly ordered input, which a real pipeline
  has to guarantee upstream.

## Error handling policy

No panics. Every fallible function returns `ParseResult<T>`. Error messages
name the field, the line number (for CSV), and what was expected vs. found
(e.g. `"T_BS must be 4x4, got 4x3"`, `"IMU measurement line 3 must contain 7
fields, got 3"`). This matches the hard-fail-by-default policy in
[scope.md](scope.md): missing files, malformed YAML, and malformed CSV records
all hard fail. Nothing implements the "camera frame gap → warn, continue"
behavior yet.

## Tests

49 GoogleTest cases across five binaries, run through CTest:

- `tests/parser_test.cpp` — inline YAML/CSV fixtures plus a smoke test against
  `datasets/machine_hall/MH_01_easy`.
- `tests/state_test.cpp` — public state-header declarations.
- `tests/slam_state_test.cpp` — explicit robot-covariance initialization,
  preallocated SLAM state storage, and protected covariance views.
- `tests/propagation_test.cpp` — nominal integration, covariance transition and
  process-noise blocks, timestep validation, a Monte Carlo covariance
  consistency check, and a 1 s EuRoC IMU-only smoke test.
- `tests/synthetic_test.cpp` — the synthetic harness, and propagation against
  analytic truth up to the headline fully excited case.

Metric values and the reasoning behind each tolerance live in
[BENCHMARKS.md](BENCHMARKS.md).

## Keeping this document current

This file must reflect only what is actually implemented. Update it
whenever a change adds, removes, or restructures modules, public types, or
entry points — see the instruction in [CLAUDE.md](CLAUDE.md).
