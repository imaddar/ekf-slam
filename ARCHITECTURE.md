# Architecture

This document describes the architecture of what is currently implemented in
`ekf-slam`. For the full project plan (weekly breakdown, deliverables,
out-of-scope items, and the target filter/system design that hasn't been built
yet) see [scope.md](scope.md).

## What exists

```
CMakeLists.txt      C++ build and GoogleTest test configuration
BENCHMARKS.md       Benchmark and metric registry
CONVENTIONS.md      Mathematical convention reference for estimator code
parser.hpp          Public C++ parser API declaration
parser.cpp          Top-level dataset loading orchestration
parser_csv.hpp/cpp  Internal CSV parsing and stereo frame pairing
parser_yaml.hpp/cpp Internal EuRoC calibration YAML parsing
image.hpp/cpp       Owned grayscale image and project pixel sampling
image_io.hpp/cpp    OpenCV-backed PNG decode behind a project-native image API
rectification.hpp/cpp Calibrated stereo remapping and rectified pseudo-cameras
image_pyramid.hpp/cpp Hand-written Gaussian image pyramid
corner_detector.hpp/cpp Grid-bucketed Shi-Tomasi detection
klt_tracker.hpp/cpp Pyramidal inverse-compositional KLT tracking
stereo_matcher.hpp/cpp Row-constrained rectified stereo association
feature_frontend.hpp/cpp Stateful track lifecycle and estimator-facing observations
evaluation.hpp/cpp  Ground-truth association plus ATE/RPE/NEES trajectory metrics
mh01_benchmark.cpp  Reproducible MH_01 evaluator executable; optional `[max_frames]`
                    truncates profiling runs and `--trace-dir <directory>` exports CSV traces
benchmark_trace.hpp/cpp  Opt-in MH_01 trace writer for IMU, camera-frame, and per-observation CSVs plus run metadata
measurement_model.hpp/cpp Pure pinhole prediction h(.) and sparse Jacobian blocks
measurement_update.hpp/cpp Sequential per-landmark stereo EKF update and gating
propagation.hpp/cpp Public IMU nominal-state propagation
state.hpp           Public nominal ESEKF state and covariance types
slam_state.hpp/cpp  Public SLAM state, registry, and covariance storage
stereo_geometry.hpp/cpp Camera/body/world metric XYZ frame transforms
augmentation_jacobians.hpp/cpp Metric XYZ augmentation derivatives
triangulation.hpp/cpp    Rectified stereo XYZ triangulation and covariance
landmark_augmentation.hpp/cpp  Stereo landmark insertion and covariance augmentation
synthetic.hpp/cpp   Public synthetic analytic trajectory and IMU generator
types.hpp           C++ parser output type declarations
roadmap.md          Planned parser API direction
docs/parser.md      Parser implementation notes and future direction
present.md          Technical deep-dive presentation material
tests/parser_test.cpp  C++ parser tests
tests/measurement_model_test.cpp Pure camera measurement-model tests
tests/measurement_update_test.cpp Sequential update, batch-equivalence, and gating tests
tests/propagation_test.cpp  C++ propagation tests
tests/state_test.cpp   C++ state-header compile tests
tests/slam_state_test.cpp  SLAM state storage tests
tests/stereo_geometry_test.cpp Metric XYZ frame-transform tests
tests/synthetic_test.cpp Synthetic trajectory and IMU propagation tests
tests/triangulation_test.cpp Stereo triangulation covariance tests
tests/landmark_augmentation_test.cpp Landmark covariance augmentation tests
tests/slam_integration_test.cpp End-to-end SLAM state and closed-loop filter tests
tests/corner_detector_test.cpp Detector equivalence against a reference implementation on a real frame
tests/euroc_frontend_test.cpp Real MH_01 rectification, tracking, and closed-loop smoke tests
tests/evaluation_test.cpp  Ground-truth association and trajectory-metric tests
```

There is a public nominal ESEKF state layout, IMU-only state covariance type,
preallocated SLAM state storage container with metric XYZ landmark registry and
batch compaction, IMU nominal-state and covariance propagation, joint SLAM
covariance propagation, metric XYZ stereo geometry, a pure pinhole camera
measurement model and sparse analytical Jacobian blocks for one world landmark,
rectified stereo triangulation uncertainty, analytical augmentation Jacobians,
metric XYZ landmark covariance augmentation, error-state injection with the
rotation reset Jacobian, and a sequential per-landmark stereo camera update with
chi-square gating, calibrated raw-EuRoC stereo rectification, and a stateful
KLT feature frontend. Filter marginalization does not exist yet. A
synthetic data harness exists for controlled pre-EuRoC validation of
propagation, SLAM-state, and closed-loop filter behavior.

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

`slam_state.hpp/cpp` defines `SlamState`, the bounded joint state container for
landmark bookkeeping and the upcoming filter math. `SlamState::create(...)` requires the caller to
provide both the initial `NominalState` and the initial 15x15 robot covariance;
the current IMU-only propagation API does not define defaults for either. The
container owns a private preallocated dynamic covariance matrix, a maximum
landmark budget, and an active landmark count reserved for registry operations.
The allocated covariance dimension is fixed at construction as
`15 + max_landmarks * 3`; the active robot and capacity-bounded landmark block
views are exposed through accessors. Storage beyond the active dimension is
unconditionally filled with NaN at construction so an early read fails visibly
instead of looking like a valid zero covariance. Landmark positions use a
preallocated metric XYZ matrix, and an ID registry maps external IDs to dense
active slots. Batch removal compacts survivors and rebuilds that registry. The
active `P_rl` and `P_ll` views are bounded by the active landmark count, and
`propagate_slam(...)` updates the robot state, `P_rr`, and `P_rl` without
allocating an augmented transition matrix. `P_ll` remains unchanged during
prediction. `augment_landmark(...)` composes stereo triangulation, world-frame
conversion, analytical Jacobians, and the new covariance column before
inserting the landmark.

`stereo_geometry.hpp/cpp` defines the rigid frame maps used by metric XYZ
initialization. `CameraCalibration::t_bs` is body-from-camera (`T_BS`), so a
camera point maps through `R_BS` and `p_BS`, then through the robot's
world-from-body pose. Both directions validate the calibration extrinsic.

`measurement_model.hpp/cpp` defines `predict_pinhole_pixel(...)`, the pure
camera measurement model `h(.)`. It takes only a world-from-body rotation
`R_WB`, world-from-body position `p_WB`, one world landmark `l_W`, and one
camera calibration. It predicts body coordinates, camera coordinates,
normalized pinhole coordinates, and pixel coordinates. It does not take
`NominalState`, velocity, biases, covariance, process-noise terms, or landmark
storage; landmark iteration and state slicing belong to the future update step.
Visibility gating (`Z <= 0`) and image-bound checks are also outside this API.
`MeasurementJacobianBlocks` supplies a compact `2x6` pose block ordered
`[delta p_W, delta theta_B]`, a `2x3` metric-XYZ landmark block, and the
resolved landmark state-column offset. The future update inserts the pose halves
at robot columns `0..2` and `6..8`, avoiding a dense mostly-zero Jacobian.

`measurement_update.hpp/cpp` defines the camera update.
`make_stereo_measurement_blocks(...)` stacks the two per-camera predictions and
Jacobians into one 4-row stereo block ordered `[u0, v0, u1, v1]` and surfaces
both depths plus a `visible` flag, leaving the Jacobians zero when either depth
is non-positive. `update_stereo_frame(...)` runs the sequential sweep: it
predicts, linearizes, and chi-square gates every observation against the prior
covariance without writing to `P`, then applies the surviving observations one
at a time in ascending landmark-offset order, then injects once. It reports
per-observation diagnostics (outcome, Mahalanobis distance, prior residual) and
the injected error state. Observations of unknown landmarks are skipped rather
than treated as errors.

`augmentation_jacobians.hpp/cpp` defines the analytical metric XYZ augmentation
derivatives. It uses the right/local orientation convention and the established
propagation ordering `[position, velocity, orientation, accelerometer bias,
gyroscope bias]`; it does not insert landmarks or mutate covariance storage.

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
`propagate_slam(...)` shares this robot transition/process-noise construction,
then applies it to the active joint covariance block structure.

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
`tests/slam_state_test.cpp` verifies explicit robot-state and covariance
initialization, capacity validation, fixed-size covariance access, the NaN guard
over inactive storage, metric XYZ landmark storage, ID lookup, capacity and
error handling, allocation stability during insertion, and batch compaction.
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
- `SlamState::create(max_landmarks, initial_robot, initial_robot_covariance)` —
  public. Fallibly allocates a joint covariance sized for the maximum metric-XYZ
  landmark budget and preserves both caller-provided robot initialization values
  exactly.
- `SlamState::landmark_block(storage_index)` — public. Returns a bounded,
  compile-time-sized `3x3` covariance block view at a capacity slot.
- `SlamState::inject_error_state(error_state)` — public. Injects an
  active-dimension error state into the nominal state, composing the rotation on
  the right, and applies the rotation reset Jacobian to `P`.
- `make_stereo_measurement_blocks(robot, landmark_world, cam0, cam1, offset)` —
  public. Stacks both cameras into one 4-row stereo Jacobian and prediction, and
  reports depth and visibility.
- `update_stereo_frame(state, observations, cam0, cam1, pixel_covariance, options)` —
  public. Runs the sequential per-landmark stereo update for one frame: gate
  against the prior, sweep, inject once.
- `make_stereo_rectification(cam0_raw, cam1_raw)` — public. Derives OpenCV remap
  tables plus distortion-free pseudo-calibrations whose extrinsics describe the
  rotated rectified camera frames.
- `FeatureFrontend::process(cam0_raw, cam1_raw, timestamp)` — public. Rectifies
  a pair, tracks and stereo-matches features, then returns mapped observations,
  augmentation candidates, and deferred landmark removals.
- `FeatureFrontend::stage_timings()` — public. Returns the `FrontendStageTimings`
  wall-clock totals accumulated per stage across every processed frame, so a
  caller can attribute frontend cost without an external profiler.
- `SlamState::robot_landmark_covariance()` and
  `SlamState::landmark_landmark_covariance()` — public. Return active-region
  views of `P_rl` and `P_ll`; inactive landmark capacity is excluded.
- `SlamState::set_robot_landmark_covariance(covariance)` — public. Validates and
  writes `P_rl` and its symmetric `P_lr` counterpart together.
- `SlamState::apply_robot_transition(transition)` — public. Validates all
  transformed cross-covariance blocks before atomically applying the transition
  and mirroring both covariance strips.
- `SlamState::add_landmark(id, position, covariance_column)` — public. Adds a
  finite metric XYZ landmark and its complete new covariance column to the next
  dense active slot. The column is shaped `(active_dim + 3) x 3` before the
  insertion and must be finite; insertion rejects duplicate IDs and a full
  capacity.
- `SlamState::landmark_offset(id)` and `SlamState::landmark_position(id)` —
  public. Resolve an external landmark ID to its state offset or stored position.
- `SlamState::remove_landmarks(ids)` — public. Removes a batch of IDs with one
  dense compaction pass and rebuilds the ID registry.
- `propagate_slam(slam_state, measurement, imu_calibration, timestep_seconds)` —
  public. Propagates the nominal robot state and joint covariance in place.
- `camera_point_to_world(robot, camera, point_camera)` and
  `world_point_to_camera(robot, camera, point_world)` — public. Apply the
  validated metric XYZ frame transforms using `CameraCalibration::t_bs`.
- `body_from_camera_transform(camera, field_name)` and
  `camera_from_body_transform(camera, field_name)` — public. Validate and return
  the rigid `T_BS` transform or its precomputed inverse for callers such as the
  synthetic projection harness and the pinhole measurement model. `field_name`
  defaults to `camera` and prefixes validation errors, so a multi-camera caller
  reports `cam1.t_bs: ...` rather than a generic field.
- `predict_pinhole_pixel(R_WB, p_WB, landmark_world, camera)` — public. Applies
  the pure pinhole measurement model for one landmark and one camera without
  visibility gating, noise, covariance access, or update-step state slicing.
- `make_measurement_jacobian_blocks(prediction, R_WB, camera, landmark_offset)` —
  public. Returns compact pose and metric-XYZ landmark pixel-Jacobian blocks.
- `make_augmentation_jacobians(robot, camera, point_camera)` — public. Returns
  the `3x15` robot Jacobian and `3x3` camera-point Jacobian for metric XYZ
  initialization.
- `triangulate_stereo(pixel_cam0, pixel_cam1, cam0, cam1, pixel_covariance)` —
  public. Validates an already-rectified horizontal rig and returns a camera-0
  metric XYZ point, its `3x4` pixel Jacobian, and anisotropic `R_tri`. Raw EuRoC
  cam0/cam1 calibration must first be converted into rectified pseudo-cameras.
- `augment_landmark(state, id, pixel_cam0, pixel_cam1, cam0, cam1,
  pixel_covariance)` — public. Computes the world landmark and complete new
  covariance column, then inserts it without changing preallocated storage.

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
4. **Every equivalence claim has an oracle.** Where two formulations are
   supposed to agree, the more expensive one is implemented as a test and the
   agreement is asserted numerically rather than argued. The sequential camera
   update is checked against a dense batch update; augmentation and propagation
   Jacobians are checked against central differences. This is what makes it
   possible to attribute a NEES failure to linearization rather than to a
   covariance bug, which is exactly the call the camera update forced.

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

### Landmark-state decisions

These choices are settled design direction for the landmark filter and replace
earlier inverse-depth planning. Full camera measurement updates remain future
work.

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
- **Body-from-camera extrinsics.** `CameraCalibration::t_bs` is `T_BS`, mapping
  camera-frame points into the body frame. The metric XYZ forward map applies
  `l_W = R_WB (R_BS l_C + p_BS) + p_WB`; keeping the direction explicit avoids
  an accidental inverse when augmentation is added.
- **One extrinsics validator.** `stereo_geometry.cpp` owns the only rigid-transform
  check; the synthetic harness and the pinhole measurement model call it rather
  than carrying their own. Separate copies would let the harness accept a
  calibration the filter rejects, and a resulting mismatch would surface much
  later as a Jacobian or triangulation failure rather than as a calibration
  error. Errors carry a caller-supplied field name and the offending values so
  a shared validator does not cost per-camera diagnostics.
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
  storage and active-dimension implementation of this decision. Covariance
  storage is private so callers cannot resize it, the initial robot state and
  covariance are supplied explicitly, and landmark insertion uses preallocated
  storage without changing the allocation.
- **NaN poison for inactive covariance storage.** The covariance region beyond
  the active robot/landmark block is filled with `NaN` at construction. This is
  intentionally unconditional, including the default `RelWithDebInfo` build:
  zero-fill would silently represent an uninitialized landmark as infinite
  confidence, while NaN makes an early read fail at the point of misuse. The
  one-time initialization cost is paid at container construction, not in the
  200 Hz propagation loop. Landmark augmentation must overwrite a block before
  it becomes active.
- **ID-based landmark registry.** Landmark IDs are external identities, not
  state indices. All landmark state and covariance access goes through the
  `id -> state offset` registry, with a parallel slot-to-ID vector for dense
  compaction, so data association and removal cannot silently corrupt covariance
  blocks or require a reverse linear scan.
- **Explicit covariance on landmark insertion.** `add_landmark` requires the
  complete new covariance column, including robot/landmark and existing
  landmark cross-covariances. The container does not invent zero correlations;
  this keeps the active covariance finite without weakening the NaN guard.
- **Anisotropic rectified-stereo uncertainty.** Triangulation uses the analytic
  pixel Jacobian `R_tri = J_tri Sigma_pixels J_tri^T` rather than an isotropic
  placeholder. The implementation rejects near-zero disparity and non-rectified
  rigs because depth uncertainty grows quadratically with range. This is a
  rectified-input contract, not raw EuRoC camera support; the parsed EuRoC
  intrinsics and extrinsics still need an undistortion/rectification adapter.
- **Augmentation exploits Jacobian sparsity.** The new covariance column uses
  only the position and orientation columns of `P_rr` implied by `J_r`; it does
  not form a dense augmented transition for each landmark birth.
- **Batch compaction for removal.** Landmark removal compacts all survivors in
  one in-place compaction operation and rebuilds the registry, using fixed-size block temporaries
  rather than full covariance matrices. A free-list design would keep offsets
  stable, but it pushes holes into every propagation, update, and debugging path.
  Dense active storage keeps `active_dim` meaningful and is the simpler choice
  unless profiling later proves compaction is a bottleneck. Capacity exhaustion
  rejects insertion; it never reallocates or silently removes another landmark.
- **Structural view invalidation.** Adding or removing a landmark changes the
  active block layout. Callers must not retain landmark offsets or covariance
  block views across either operation; they must resolve them again afterward.
- **Shared robot propagation math, separate public paths.** The current
  `propagate(...)` API can remain as the IMU-only test/development path while
  SLAM propagation grows a joint-state API. Both should call the same internal
  robot transition/noise builder so `F`, `Phi`, and `Q_d` are not duplicated.
  Putting every mode into one public function would work mechanically, but it
  would blur two different contracts: fixed 15x15 IMU-only propagation versus
  joint covariance propagation with landmark cross-block invariants.

### Camera update decisions

- **Sequential per-landmark updates, not batch.** Under block-diagonal `R` and a
  shared linearization point the two are algebraically identical, both reducing
  to `P^-1 + sum_i H_i^T R_i^-1 H_i`. Sequential was chosen for per-landmark
  chi-square gating, `4x4` innovation factorizations instead of one `4m x 4m`,
  and a small fixed working set. The dominant `4 m n^2` covariance term is the
  same either way; at `N = 50, m = 20` sequential costs about 2.35 MFLOP against
  batch's 3.58. Batch exists only as a test oracle. The condition to revisit is
  cache, not flops: sequential streams `P` once per landmark, which is free
  while `P` fits in L2 (`n = 165` is 218 KB) and expensive when it does not
  (`N = 200` is 3.0 MB and about 120 MB of traffic per frame).
- **One linearization point per frame, injection deferred to the end.** The
  nominal state is frozen for the sweep and the error state accumulates, which
  is what makes the sweep equal the batch answer and keeps it independent of
  observation order. It also makes the SO(3) injection and the covariance reset
  happen once per frame instead of once per landmark. Relinearizing mid-sweep is
  the natural code shape and is the wrong one: it is order-dependent and
  manufactures information in the unobservable directions, the opposite of the
  direction the OC-EKF/FEJ open decision needs to move.
- **Gate against the prior covariance.** `P` shrinks monotonically through a
  sweep, so gating against the running covariance is tighter for later landmarks
  and can reject a good measurement because earlier ones already shrank `P`.
  Prior gating is order-independent and matches the diagonal blocks of the batch
  `S`. It errs loose rather than tight, which is the safer direction. It is also
  nearly free: `H_i P H_i^T` needs only the `9x9` sub-block at the observation's
  own columns, so no covariance snapshot is required.
- **Factored Joseph form.** `P = M - (M H^T) K^T + K R K^T` with
  `M = P - K (H P)`, three rank-4 `n^2` updates reusing the same 9-column
  gather. Under the optimal gain this is algebraically equal to `P - K (H P)`,
  so the only reason to write it this way is roundoff behaviour across `m`
  successive updates; it must not be "simplified". Square-root or UD
  factorization remains the real upgrade if PSD ever has to be guaranteed rather
  than asserted. A `use_joseph_form` flag exists to compare the two, not to
  switch off in production.
- **Injection and the rotation reset live on `SlamState`.** `inject_error_state`
  is a container method rather than update-module code because landmark position
  storage is private and the SO(3) manifold operation should exist in exactly one
  place. The reset Jacobian is the SO(3) right Jacobian `J_r(delta theta_hat)`,
  applied to the orientation rows and columns only. Skipping it is a common
  shortcut; it is implemented here because it costs `18n` flops once per frame.
  Note it is a similarity transform, not an information change, so `G P G^T` is
  not Loewner-ordered against the prior.
- **The update never augments.** Observations of unknown landmarks are reported
  as skipped and the caller decides whether to augment. A landmark must never be
  updated in the frame that created it: its triangulated position is a
  deterministic function of exactly those pixels, so prior and measurement are
  perfectly correlated. Augmenting after the update also keeps landmark-offset
  invalidation outside a sweep that holds offsets.
- **Batch as a test oracle.** The equivalence proof is only worth as much as its
  verification, so `tests/measurement_update_test.cpp` implements a dense
  textbook batch update and asserts agreement to `1e-12`, plus invariance across
  all permutations of a 3-landmark frame. Those two assertions catch every
  failure mode this design has: dropping the accumulated-error term,
  relinearizing mid-sweep, and correlated `R`.

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

### Camera measurement model

- **`h(.)` is a pose-and-one-landmark function, not a filter function.**
  `predict_pinhole_pixel(...)` takes `R_WB`, `p_WB`, one `l_W`, and one camera
  calibration directly. It deliberately does not accept `NominalState`,
  `SlamState`, covariance, velocity, or biases. This keeps projection geometry
  independent from propagation and leaves landmark iteration, state-vector
  slicing, residual stacking, and Kalman algebra to the future update step.
- **Visibility gating is outside `h(.)`.** Points with `Z <= 0` or projected
  pixels outside the image are frontend/association decisions, not measurement
  geometry. Keeping those checks out makes the model a predictable map from a
  supplied state hypothesis to a pixel prediction. The cost is that callers must
  only use valid correspondences when forming an EKF innovation: at `Z = 0` the
  model returns success with a non-finite pixel, while `Z < 0` can still return a
  finite pixel. A test pins both cases so the update step treats visibility
  gating as its own responsibility instead of relying on `allFinite()`.
- **The extrinsic transform and its validation come from `stereo_geometry`.**
  `predict_pinhole_pixel(...)` calls `camera_from_body_transform(...)` rather
  than re-deriving `R_BS^T` and re-checking `T_BS`, so a calibration accepted by
  one frame path cannot be rejected by the other, and the `SE(3)` inverse exists
  in one place.
- **`h(.)` is cross-checked against the synthetic harness.** The harness projects
  landmarks through its own code path, so a test asserts the two agree on
  noiseless pixels. Without it the two implementations could drift apart and
  every synthetic update test would still pass. The comparison uses exact pixels
  only; noisy measurements are reserved for update-step tests, where a mismatch
  would otherwise be ambiguous between a projection bug and expected noise.
- **Distortion is excluded from the state measurement model.** Measurements are
  expected to be undistorted before innovation computation. The EuRoC distortion
  coefficients are fixed calibration constants, so carrying them through
  `d h / d x` would add complexity without adding estimated state information.
  The current implementation therefore models pinhole projection only.
- **Jacobians remain sparse and explicit about layout.** The `2x6` compact pose
  block is ordered `[delta p_W, delta theta_B]`; the future update places its
  halves at non-contiguous state columns `0..2` and `6..8`. A resolved landmark
  offset identifies the sole active `2x3` landmark block, leaving every other
  robot and landmark block structurally zero without allocating a dense matrix.

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
- **Visualization traces are exported at the benchmark boundary, not from
  estimator code.** `mh01_benchmark --trace-dir` records IMU propagation,
  camera-frame prior/posterior, and per-observation innovation rows without
  changing the filter API or normal benchmark cost. Each row carries only the
  fixed-size 15-state robot covariance; exporting the full joint covariance at
  every IMU sample would grow with the map and make presentation data needlessly
  large. Metadata pins the sequence, noise setting, landmark budget, timestamp
  unit, covariance layout, and compiled Git revision for reproducibility.

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

- **Update linearization.** Measured, not predicted. Over 50 Monte Carlo runs
  the propagation-only filter sits at NEES `13.56` inside the `[13.52, 16.56]`
  bound; with camera updates the same scenario reaches `24.23`. The sweep is
  proven equal to a batch update and the Jacobians are verified against central
  differences, so this is linearization rather than a covariance bug.

  The obvious hypothesis -- spurious information in the unobservable yaw
  direction, the effect FEJ and OC-EKF exist to fix -- is *not* what the data
  shows. Splitting the orientation error along gravity gives yaw `1.41` against
  an expected `1.0` while tilt is `4.81` against `2.0`. The inconsistency is in
  the *observable* part of the orientation. Shrinking the initial tilt error
  drives the whole filter back to consistency (`24.23 -> 17.24 -> 15.92` for
  initial tilt sigma `0.01 -> 0.003 -> 0.001 rad`, the last inside the bound),
  which is the signature of the EKF's first-order approximation. It concentrates
  in tilt because rotation is the only state entering the measurement
  nonlinearly.

  That makes an **iterated EKF the targeted fix**, not FEJ. Relinearizing toward
  the posterior attacks a second-order error; FEJ attacks a different problem
  this filter does not currently have. Freezing the linearization point per frame
  is already in place and remains a prerequisite if FEJ is wanted later.
  See `BENCHMARKS.md` for the full experiment set;
  `SlamClosedLoopTest.DISABLED_MonteCarloRobotNeesMeetsTheConsistencyTarget` is
  the acceptance test.
- **Feature frontend.** The implementation uses hand-written pyramidal,
  inverse-compositional translation KLT, Shi-Tomasi corners, and row-constrained
  stereo matching. This suits EuRoC's 20 Hz cadence and gives sub-pixel pixels
  without putting OpenCV types at the estimator boundary. Tracks lost to blur or
  occlusion are deliberately reborn under a fresh ID; descriptor re-detection,
  RANSAC, and robust costs remain separate future work.
- **Layered rejection and lifecycle.** Forward/backward KLT validation and
  disparity bounds reject visual failures before the EKF. The existing prior
  innovation gate is independent filter evidence; repeated gated observations
  retire a mapped track at the following frame boundary so offsets cannot change
  during a sweep.
- **Initialization.** The EuRoC smoke test starts from ground truth, which a
  real system does not have. Static-start bias and gravity-direction estimation,
  or a short visual-inertial alignment, are the standard options and neither
  exists. This is now coupled to the NEES finding above: the filter is consistent
  at an initial tilt error of `0.001 rad` and inconsistent at `0.01 rad`, and
  static gravity alignment from a stationary accelerometer reaches the former.
  Initialization quality is therefore a consistency lever, not just a
  convenience.
- **Raw-frame MH_01 metrics rather than aligned scores.** `mh01_benchmark`
  associates posterior camera-time estimates to interpolated EuRoC truth and
  reports raw-world ATE, one-second RPE, and 15-dof NEES. A global SE(3)
  alignment would make a conventional leaderboard score, but it would remove
  precisely the drift this truth-initialized VIO run must expose. Frames past
  EuRoC ground-truth coverage still run through the filter and are excluded only
  from metric aggregation.
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

127 GoogleTest cases across thirteen binaries, run through CTest (125 active;
two NEES diagnostics intentionally disabled):

- `tests/parser_test.cpp` — inline YAML/CSV fixtures plus a smoke test against
  `datasets/machine_hall/MH_01_easy`.
- `tests/state_test.cpp` — public state-header declarations.
- `tests/slam_state_test.cpp` — SLAM state initialization, bounded covariance
  blocks, cross-covariance views, NaN-tail poisoning, explicit landmark
  covariance insertion, metric XYZ registry operations, and batch compaction.
- `tests/propagation_test.cpp` — nominal and joint integration, covariance
  transition and process-noise blocks, timestep validation, a Monte Carlo
  covariance consistency check, and a 1 s EuRoC IMU-only smoke test.
- `tests/stereo_geometry_test.cpp` — metric XYZ frame-map identity,
  round-trip, nonzero-extrinsic, and rigid-transform validation cases.
- `tests/measurement_model_test.cpp` — pure pinhole projection, inverse-pose
  convention, per-camera baseline behavior, analytical Jacobians checked with
  central finite differences across five pose/extrinsic cases, the
  no-visibility-gating contract for `h(.)`, and noiseless synthetic agreement.
- `tests/augmentation_jacobians_test.cpp` — analytical metric XYZ Jacobians,
  finite-difference agreement, sparsity, and validation.
- `tests/triangulation_test.cpp` — rectified stereo reconstruction, covariance
  scaling, principal-axis orientation, Monte Carlo agreement, and degeneracy.
- `tests/landmark_augmentation_test.cpp` — covariance augmentation, dense
  robot cross-correlation, zero-pose uncertainty, and inherited landmark
  correlation.
- `tests/slam_integration_test.cpp` — fully excited propagation, landmark
  insertion, stable allocation, PSD checks, timing capture, and compaction.
- `tests/synthetic_test.cpp` — the synthetic harness, and propagation against
  analytic truth up to the headline fully excited case.
- `tests/euroc_frontend_test.cpp` — real MH_01 rectification/tracking and a
  short closed-loop image measurement smoke run.
- `tests/evaluation_test.cpp` — ground-truth interpolation plus analytical
  ATE, RPE, and NEES metric cases.

Metric values and the reasoning behind each tolerance live in
[BENCHMARKS.md](BENCHMARKS.md).

## Keeping this document current

This file must reflect only what is actually implemented. Update it
whenever a change adds, removes, or restructures modules, public types, or
entry points — see the instruction in [CLAUDE.md](CLAUDE.md).
