# Camera Frontend Implementation Plan

This plan covers the visual frontend that feeds the camera measurement update:
raw EuRoC stereo PNGs in, `StereoObservation` records with stable `LandmarkId`s
out. It picks up where
[camera_update_implementation_plan.md](camera_update_implementation_plan.md)
stops — the update, triangulation, and augmentation all exist and are verified
against synthetic rectified stereo, but nothing produces real correspondences,
and `triangulate_stereo(...)`'s rectified-rig contract is currently satisfied
only by the synthetic harness.

It closes three unchecked `scope.md` items in Week 6–8:

- Feature detection and tracking (decide: KLT vs descriptor-based)
- Raw EuRoC stereo undistortion/rectification before landmark initialization
- Landmark marginalization — the *policy* half of it; the covariance mechanics
  already exist in `remove_landmarks(...)`

Each step has an explicit gate. A dependent step must not begin until its gate
passes.

**Status: implemented through the MH_01 closed-loop smoke test.** The current
smoke run establishes frontend plumbing and measurement flow; its high gate rate
means tracker-noise characterization and longer-sequence accuracy evaluation
remain benchmark work, not completion evidence for ATE/RPE/NEES.

## What the frontend has to produce

The update API already defines the handshake, and nothing in
`measurement_update.hpp` needs to change:

```text
frontend -> std::vector<StereoObservation>{ id, pixel_cam0, pixel_cam1 }
         -> rectified pseudo-CameraCalibration for cam0 and cam1
         -> a per-frame list of dead track ids for remove_landmarks(...)
```

Observations of landmarks not yet in the state come back from
`update_stereo_frame(...)` as `ObservationOutcome::kUnknownLandmark` and are
skipped, which is exactly the "augment after the update" ordering the update was
designed around. The frontend allocates the ID; the caller decides whether that
ID becomes a landmark.

Pixels must be in the **rectified** image, because that is the only domain in
which `triangulate_stereo(...)` and `predict_pinhole_pixel(...)` are valid
(`CONVENTIONS.md` §8, §10).

## Architectural Decisions

- **Pyramidal inverse-compositional KLT, not descriptor matching.** EuRoC stereo
  is 20 Hz with modest inter-frame motion, so the small-baseline assumption KLT
  needs actually holds, and KLT returns sub-pixel positions directly — which
  matters because the tracker's pixel noise *is* the `R` fed to the update, and a
  worse `R` shows up immediately in NEES. Descriptors (ORB/BRIEF) survive larger
  baselines and can re-acquire a landmark after occlusion; neither capability is
  needed here, since loop closure and relocalization are explicitly out of scope
  in `scope.md`. The cost accepted is that a track lost to motion blur or
  occlusion is lost permanently and comes back as a new ID with a new landmark.
  Revisit if the target moves to the fast V1_03/V2_03 sequences, where blur
  breaks KLT well before it breaks a descriptor.
- **Inverse compositional, not forward additive.** The `2x2` Hessian is built
  from the template gradients, so it is constant across iterations and is
  precomputed once per feature per pyramid level. Forward-additive recomputes
  gradients and re-inverts the Hessian every iteration for the same answer. The
  restriction is that the warp must form a group that composes — true for the
  translation-only warp used here, and the reason a full affine warp would need
  revisiting rather than a parameter change.
- **Translation-only warp with zero-mean patch normalization.** Affine KLT is the
  standard upgrade for long tracks under rotation, at 6 parameters instead of 2.
  At 20 Hz the inter-frame warp is close to a translation, and the affine
  parameters are poorly conditioned on a 21x21 window. Patch mean subtraction is
  kept because EuRoC MH runs auto-exposure, and a plain SSD cost treats a global
  gain change as motion. This is a real, cheap failure mode; the affine warp is a
  speculative one.
- **Rectify the images, don't undistort the points.** Both are valid. Rectifying
  the image once per frame makes the epipolar line an exact image row, which
  turns stereo matching into a 1-D search and makes `u0 - u1` a disparity by
  construction. Tracking on raw imagery and undistorting the resulting points
  preserves the original sampling, but leaves the epipolar constraint a curve and
  produces spatially varying, anisotropic pixel noise after the undistortion
  Jacobian — which the update's single `Eigen::Matrix4d pixel_covariance` cannot
  express. The cost is one bilinear resample per image, which correlates
  neighboring pixel noise slightly and blurs high-frequency corner structure.
- **OpenCV for PNG decode and the rectification maps; the tracker is
  hand-written.** `cv::stereoRectify` and `cv::initUndistortRectifyMap` implement
  a well-specified, uninteresting transform that is easy to get wrong by hand and
  proves nothing about the estimator. The pyramid, the corner detector, and the
  KLT solver stay in-repo because they are the part with a defensible design
  story, and because their numerical behavior has to be characterized to set `R`.
  OpenCV is confined to two translation-unit-local call sites
  (`image_io.cpp`, `rectification.cpp`) and must not appear in any public header,
  so the Jetson build can swap the codec without touching the frontend.
- **Rectification produces pseudo-`CameraCalibration` values, including a
  recomputed `t_bs`.** Rectification rotates the camera frame, so a rectified
  pixel's ray lives in `C'`, not `C`. Reusing the raw `T_BS` with rectified pixels
  is the single highest-value bug this plan can prevent: it survives every
  triangulation test (the rig is still self-consistently rectified), it produces
  visually plausible landmarks, and it shows up only as a slow bias in the filter.
  The composition is `T_BC' = T_BC * blkdiag(R_C'C^T, 1)`, and Gate 2 tests it
  directly rather than by inspection.
- **The frontend owns landmark identity; IDs are never reused.** `SlamState`'s
  registry keys on `LandmarkId`, so recycling an ID would silently graft a new
  track's measurements onto a dead landmark's covariance block. A monotonically
  increasing `std::int64_t` counter costs nothing and makes that impossible.
- **The track table stores IDs, never state offsets.** Adding or removing a
  landmark invalidates every offset (`CONVENTIONS.md` §16). The frontend resolves
  through `landmark_offset(id)` at the moment of use, and all removals happen at a
  frame boundary, never inside a sweep.
- **Spatial distribution is enforced by grid bucketing, not by a global corner
  score.** A pure "top-N by Shi-Tomasi score" detector piles features onto the
  highest-contrast region of the image, which is a geometrically degenerate
  configuration for pose observability even though every individual measurement is
  good. Bucketing into cells with a per-cell cap keeps the constraint spread
  across the field of view. It costs some measurement quality per feature, and
  that is the intended trade.
- **The innovation gate is not the outlier detector.** Chi-square gating in
  `update_stereo_frame(...)` catches a match once it disagrees with the filter. A
  consistent-but-wrong track — the classic repetitive-texture drift, one pixel per
  frame along a rail — never triggers it. The frontend's forward-backward check
  and epipolar residual check are the defenses that operate before the filter
  sees the measurement, and they are independent evidence rather than a second
  copy of the same test. This is the `ARCHITECTURE.md` "Outlier rejection beyond
  the innovation gate" open decision being partially closed; RANSAC on a
  two-view model stays open.
- **`R` is measured from the tracker, not assumed.** The per-pixel noise fed to
  the update is estimated from the epipolar row residual on real imagery (Gate
  6), not set to a round number. A frontend that reports optimistic pixel noise
  produces exactly the same NEES signature as a covariance bug, and the filter
  already has one documented consistency problem that must not get a second,
  confounded, source.
- **Image-based tracking first; IMU-predicted feature locations deferred.**
  Seeding the KLT search from the propagated pose is the standard VIO trick and
  is the difference between surviving and not surviving fast rotation. It is
  deliberately not in this plan, because it couples frontend correctness to
  filter correctness and makes a tracking failure ambiguous between the two. It
  is the first follow-up, with a defined metric (track survival rate under high
  angular rate) already available from Step 7.

## Step 0: Prerequisites and Baseline

No new frontend code. Establish what the frontend will be measured against, and
get OpenCV into the build behind a boundary.

### Work

- Add `find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)`
  to `CMakeLists.txt`, then link the OpenCV 4 `calib3d` target or the OpenCV 5
  `calib`/`geometry` split privately to `ekf_slam_parser`. This keeps the image
  dependency out of public headers and works on both supported major versions.
- Confirm OpenCV is actually installed (`brew install opencv`); it is not present
  on the current dev machine.
- Record the EuRoC MH_01 propagation-only trajectory error over 10 s, 30 s, and
  the full sequence, initialized from the first ground-truth state. The
  drift-growth row in `BENCHMARKS.md` covers up to 30 s; the full-sequence number
  is the baseline Step 8 has to beat.

### Gate 0

- The existing 121-test suite passes unchanged after the OpenCV link is added.
- No public header includes an OpenCV header. Enforce with a grep-based check in
  the test suite or a comment-documented convention, not by hoping.
- Full-sequence MH_01 propagation-only position error is recorded in
  `BENCHMARKS.md`. Without it, Gate 8 has nothing to compare against.

## Step 1: Image Container and Decode

New files `image.hpp/cpp`, `image_io.hpp/cpp`. Pure data plumbing; no geometry.

### Work

```cpp
// Row-major 8-bit grayscale. Owns its buffer; stride is width (no padding),
// because every consumer here is dense and padding buys nothing.
struct GrayImage {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> data;
};

ParseResult<GrayImage> load_grayscale_png(const std::filesystem::path& path);

// Bilinear sample at continuous pixel coordinates. Pixel (0,0) is the CENTER of
// the top-left pixel, matching the EuRoC cu/cv convention.
double sample_bilinear(const GrayImage& image, double u, double v);
bool in_bounds(const GrayImage& image, double u, double v, double margin);
```

`load_grayscale_png` wraps `cv::imread(..., cv::IMREAD_GRAYSCALE)` and copies
into the owned buffer. EuRoC images are already 8-bit mono, so no conversion path
is needed; a non-8-bit or non-single-channel input is an error, not a silent
convert.

The pixel-center convention is the load-bearing detail. EuRoC's `cu, cv` and
OpenCV both place `(0,0)` at the center of the top-left pixel. A sampler that
assumes `(0,0)` is the top-left *corner* introduces a uniform half-pixel bias
that survives every self-consistency test in the repo and lands as a small
constant position offset in the filter.

### Gate 1

- Round-trip: load a checked-in MH_01 frame, assert dimensions `752x480`.
- `sample_bilinear` at integer coordinates reproduces the stored pixel exactly.
- `sample_bilinear` at a half-integer coordinate equals the mean of the two
  neighbors, which pins the pixel-center convention numerically rather than by
  comment.
- Out-of-bounds and near-edge sampling is rejected by `in_bounds` rather than
  reading past the buffer; verify under `-fsanitize=address` at least once.
- A missing file and a non-grayscale image both return `ParseResult` errors with
  the path named, per the no-panics policy.

## Step 2: Stereo Rectification and Pseudo-Calibration

New files `rectification.hpp/cpp`. This is where the raw EuRoC calibration is
converted into something `triangulate_stereo(...)` will accept.

### Work

```cpp
struct StereoRectification {
    // Pseudo-cameras: zero distortion, shared intrinsics, shared orientation,
    // cam1 at positive cam0 x. Satisfies the CONVENTIONS.md section 10 contract.
    CameraCalibration cam0_rectified;
    CameraCalibration cam1_rectified;
    double baseline_meters;
    // Opaque remap tables; OpenCV types stay out of this header.
    struct Maps;
    std::shared_ptr<const Maps> maps;
};

ParseResult<StereoRectification> make_stereo_rectification(
    const CameraCalibration& cam0_raw,
    const CameraCalibration& cam1_raw);

ParseResult<GrayImage> rectify_cam0(const StereoRectification&, const GrayImage&);
ParseResult<GrayImage> rectify_cam1(const StereoRectification&, const GrayImage&);
```

Construction:

```text
T_C0C1     = T_BS_0^-1 * T_BS_1                    (relative stereo extrinsic)
R0, R1, P0, P1 = cv::stereoRectify(K0, D0, K1, D1, size, R_C0C1, p_C0C1, alpha=0)

  R0 maps C0 -> C0', R1 maps C1 -> C1'

rectified intrinsics = [P0(0,0), P0(1,1), P0(0,2), P0(1,2)]
baseline             = -P1(0,3) / P1(0,0)
t_bs_rect0           = T_BS_0 * blkdiag(R0^T, 1)
t_bs_rect1           = T_BS_1 * blkdiag(R1^T, 1)
distortion           = 0
```

Then **validate rather than assume**, because everything downstream trusts this:

```text
T_C0'C1' = t_bs_rect0^-1 * t_bs_rect1
  rotation    ~= I           (shared orientation)
  translation ~= [b, 0, 0]   with b > 0
  P0 and P1 intrinsics identical
```

`alpha = 0` crops to the all-valid region, which loses field of view but
guarantees every rectified pixel has real source data. `alpha = 1` keeps the full
frame and fills the invalid border with zeros, which the corner detector will
happily detect features on. Crop is the safer default; record the FOV loss.

If the baseline comes out negative, the camera roles are swapped relative to the
`triangulate_stereo` contract. Swap the roles explicitly and re-derive; do not
negate the disparity, which would leave `u0 - u1` meaning something different
from what `CONVENTIONS.md` §10 says it means.

Measured from the checked-in MH_01 calibration, so the gate has real numbers to
assert against rather than a shape:

```text
T_C0C1 translation      = [0.110074, -0.000157, 0.000889] m
baseline                = 0.110078 m
relative rotation angle = 0.014284 rad  (0.82 deg)
```

The raw rig is therefore already close to rectified — sub-millimetre y and z
baseline, under a degree of relative rotation. That is useful and also a trap:
rectification will barely move the pixels, so a broken rectification still
produces images that look correct and matches that mostly succeed. The rectified
baseline must stay within a few micrometres of `0.110078 m`, and the rectified
relative rotation must drop to zero, not merely get smaller.

### Gate 2

- **The extrinsic-composition test.** Take a point `X_C0'` in the rectified cam0
  frame and its raw-frame equivalent `X_C0 = R0^T * X_C0'`. Assert
  `camera_point_to_world(robot, cam0_rectified, X_C0')` equals
  `camera_point_to_world(robot, cam0_raw, X_C0)` to `1e-12`, at a nontrivial
  `robot` pose. This is the test that fails if `t_bs` is passed through
  unrecomposed, and it is the reason this step has its own gate.
- The rectified pair passes `triangulate_stereo(...)`'s own rig validation — call
  it, don't re-implement the check.
- `body_from_camera_transform(...)` accepts both pseudo-calibrations, so the
  rigid-transform validator agrees the composition stayed rigid.
- Distortion round-trip: for a grid of raw pixels, `cv::undistortPoints` then
  re-project through the raw distortion model returns the original within
  `1e-3 px`. This checks the parsed EuRoC coefficients are being handed to
  OpenCV in the right order (`k1, k2, p1, p2` for radial-tangential — the parser
  stores four coefficients and the ordering must be confirmed against the EuRoC
  `distortion_model` field, not assumed).
- A synthetic end-to-end check: place a known world point, project it through the
  *raw* model with distortion applied by hand, rectify that pixel through the
  maps, and assert the rectified pixel matches `predict_pinhole_pixel(...)` with
  the rectified pseudo-camera to `0.1 px`. Sub-pixel rather than exact because
  the map is resampled.
- Rectified MH_01 images are visually straight-lined: assert the epipolar
  property numerically in Gate 6 rather than by eye here.
- Missing or non-rectifiable calibration returns a `ParseResult` error naming the
  offending field.

## Step 3: Image Pyramid and Gradients

New file `image_pyramid.hpp/cpp`. Hand-written, no OpenCV.

### Work

```cpp
inline constexpr int kPyramidLevels = 4;   // 752x480 down to 94x60

struct ImagePyramid {
    std::vector<GrayImage> levels;         // levels[0] is full resolution
};

ParseResult<ImagePyramid> build_pyramid(const GrayImage& image, int levels);
```

Each level is a 5-tap binomial blur `[1 4 6 4 1] / 16` applied separably,
followed by 2x decimation. The blur is not optional: decimating without it
aliases high-frequency texture into the coarse levels, and the coarse levels are
exactly where large motions are resolved, so aliasing there produces confident
wrong tracks rather than failed ones.

Four levels puts the coarsest at `94x60`, which resolves an inter-frame motion of
roughly `2^3 * (window/2) ≈ 80 px` at full resolution — comfortably above what
20 Hz EuRoC MH produces, and the number to revisit for the fast sequences.

Gradients are computed on the template patch during KLT setup, using central
differences on the bilinear-sampled patch rather than a precomputed Sobel image,
so the gradient is evaluated at the same sub-pixel locations as the intensities.

### Gate 3

- Level dimensions follow `ceil(n / 2)` at every level, and odd input widths do
  not lose the last column.
- A constant image stays constant at every level (the blur normalizes to 1).
- A linear ramp stays linear away from the borders, which catches an unnormalized
  or asymmetric kernel.
- Separable and direct 2-D convolution agree to `1e-12` on a random image.
- Border handling is clamp-to-edge and is tested explicitly; a wrap or a zero
  border creates artificial high-contrast corners at the image edge.
- Determinism: same input, same bytes out.

## Step 4: Shi-Tomasi Detection with Grid Bucketing

New file `corner_detector.hpp/cpp`.

### Work

```cpp
struct DetectorOptions {
    int    grid_cols = 12;
    int    grid_rows = 8;
    int    max_features_per_cell = 3;
    double min_eigenvalue = 1e-3;     // absolute, on normalized intensities
    double min_separation_px = 15.0;
    int    structure_tensor_window = 3;
};

struct Corner {
    Eigen::Vector2d pixel;
    double score;
};

// Detects only where existing tracks are absent: cells already holding
// max_features_per_cell live tracks are skipped entirely.
std::vector<Corner> detect_corners(
    const GrayImage& image,
    std::span<const Eigen::Vector2d> existing_features,
    const DetectorOptions& options);
```

Score is the smaller eigenvalue of the `2x2` structure tensor
`sum_W [Ix^2, IxIy; IxIy, Iy^2]`, which for a `2x2` symmetric matrix has a closed
form and needs no eigensolver. Shi-Tomasi over Harris because the min-eigenvalue
is directly the quantity that conditions the KLT Hessian — a feature that scores
well here is by construction one KLT can solve for, and the Harris corner
response `det - k*trace^2` is a cheaper proxy for the same thing with a free
parameter to tune.

Detection is incremental: cells that already hold enough live tracks are not
searched at all. This is both the cost saver and the mechanism that keeps
long-lived tracks from being displaced by fresher, higher-scoring corners.

### Gate 4

- A synthetic checkerboard yields corners at the known intersections within
  `1 px`.
- A pure gradient ramp yields no corners (one eigenvalue is zero), and a uniform
  region yields none.
- The per-cell cap is never exceeded, and no two returned corners are closer than
  `min_separation_px`.
- Passing existing features suppresses detection in their cells: with a full grid
  of existing features, the detector returns empty.
- Determinism across runs, including tie-breaking between equal scores — an
  unstable sort here makes every downstream test flaky for no reason.
- On a real MH_01 frame, the returned corners cover at least 70% of the grid
  cells, which is the actual property being bought and is not implied by any of
  the synthetic cases.

## Step 5: Pyramidal Inverse-Compositional KLT

New file `klt_tracker.hpp/cpp`. The numerical core of the frontend.

### Work

```cpp
struct KltOptions {
    int    window_half_size = 10;      // 21x21
    int    max_iterations = 30;
    double convergence_px = 0.01;
    double max_residual_per_pixel = 20.0;   // mean |I - T| after mean removal
    double min_hessian_eigenvalue = 1e-4;
    double forward_backward_px = 0.5;
    bool   constrain_to_row = false;        // true for the stereo pass
};

enum class KltStatus { kTracked, kDiverged, kOutOfBounds, kIllConditioned, kHighResidual };

struct KltResult {
    Eigen::Vector2d pixel;
    KltStatus status;
    double residual;
};

KltResult track_feature(
    const ImagePyramid& from,
    const ImagePyramid& to,
    const Eigen::Vector2d& from_pixel,
    const Eigen::Vector2d& initial_guess,
    const KltOptions& options);
```

Per level, coarse to fine, with the estimate scaled by 2 between levels:

```text
template T   = patch sampled from `from` at from_pixel / 2^level
grad T       = central differences on the sampled patch
H            = sum_x grad_T^T grad_T                   (2x2, precomputed)
reject if min eigenvalue of H < min_hessian_eigenvalue

iterate:
  I_w  = patch sampled from `to` at current estimate
  e    = sum_x grad_T^T ((I_w - mean(I_w)) - (T - mean(T)))
  dp   = H^-1 e
  p   <- p - dp                                        (inverse composition)
  stop when ||dp|| < convergence_px
```

Mean removal on both patches is the illumination model. It makes the cost
invariant to an additive intensity offset; a full gain-and-bias model would
normalize by the patch standard deviation as well, at the cost of a second
statistic per iteration and a division that is unstable on low-texture patches.
Start with mean removal and let Gate 5's residual distribution on MH_01 say
whether gain invariance is needed.

`constrain_to_row` zeroes the `v` component of `dp`, reducing the solve to the
scalar `dp_u = e_u / H_uu`. This is the stereo pass: after rectification the
match is on the same row by construction, and letting `v` float would fit
rectification error into the disparity.

Forward-backward validation is a second `track_feature` call in the reverse
direction, seeded at the forward result. It roughly doubles tracking cost and is
the main defense against a track sliding along an edge — the aperture-problem
failure that produces a confident, low-residual, wrong answer. Keep it on; it is
the cheapest independent evidence available.

### Gate 5

The gate that decides whether the tracker is trustworthy.

- **Known-warp recovery.** Take a real MH_01 frame, shift it by a known
  sub-pixel translation via bilinear resampling, and track 200 detected corners.
  Median recovery error `< 0.05 px`, 95th percentile `< 0.2 px`, over shifts of
  `0.3`, `1.7`, and `7.5 px`. The sub-pixel shift is the case that catches a
  half-pixel convention error between the sampler and the gradient.
- **Large-motion recovery through the pyramid.** A `40 px` shift is recovered at
  4 levels and *fails cleanly* (`kDiverged`, not a wrong answer) at 1 level. A
  test that only shows success proves the pyramid runs, not that it is needed.
- **Illumination invariance.** A global `+20` intensity offset leaves the
  recovered position unchanged to `1e-6`. A `1.3x` gain change is allowed to
  degrade, and the measured degradation is what decides whether to add the gain
  term.
- **Aperture-problem rejection.** On a synthetic straight edge, the along-edge
  component is unconstrained; the tracker must return `kIllConditioned` or be
  caught by forward-backward, not return a confident wrong displacement.
- **Forward-backward has power.** Construct a case that passes the residual
  check and fails forward-backward. If no such case exists in the test suite, the
  check is not demonstrably doing anything.
- **Row constraint.** With `constrain_to_row`, the recovered `v` is bit-identical
  to the initial guess.
- Determinism, and no heap allocation inside `track_feature` — the patch, its
  gradients, and the Hessian are fixed-size for a fixed window.

## Step 6: Stereo Matching and Noise Characterization

New file `stereo_matcher.hpp/cpp`, plus the measurement that sets `R`.

### Work

```cpp
struct StereoMatchOptions {
    double min_disparity_px = 1.0;      // ~ 50 m at EuRoC's 0.11 m baseline
    double max_disparity_px = 200.0;
    double max_row_residual_px = 1.0;   // unconstrained-pass validation only
    KltOptions klt{};                   // with constrain_to_row = true
};

struct StereoMatch {
    Eigen::Vector2d pixel_cam0;
    Eigen::Vector2d pixel_cam1;
    double disparity;
    bool valid;
};

StereoMatch match_stereo(
    const ImagePyramid& cam0_rectified,
    const ImagePyramid& cam1_rectified,
    const Eigen::Vector2d& pixel_cam0,
    double disparity_prior_px,          // previous frame's, or 0 for a new track
    const StereoMatchOptions& options);
```

The prior matters. An existing track's disparity changes slowly, so seeding from
the previous frame usually converges in a couple of iterations at the finest
level only. A new track has no prior and gets the full pyramid descent from a
zero-disparity guess.

`min_disparity_px` is where the frontend and the estimator meet: below it, the
triangulated depth uncertainty is enormous and `augment_landmark` inserts a
landmark whose covariance dominates the map. `triangulate_stereo` already rejects
near-zero disparity, but its default `1e-9` threshold is a degeneracy guard, not
a quality policy. Setting the policy here, in pixels, with the implied maximum
range written down, keeps the two concerns separate.

**Noise characterization.** Run the stereo pass on MH_01 a second time with
`constrain_to_row = false` and record the distribution of `v0 - v1`. Under a
correct rectification the true row difference is zero, so

```text
sigma_px ~= stddev(v0 - v1) / sqrt(2)
```

is an estimate of per-pixel matching noise that is completely independent of the
filter. Cross-check it against the forward-backward residual distribution from
Step 5. The resulting `sigma_px^2 * I4` is what gets passed as
`pixel_covariance`.

This also doubles as the empirical rectification check: a systematic nonzero mean
in `v0 - v1` is a rectification error, and a large variance is a tracking
problem. The two are distinguishable by whether the mean or the spread is wrong,
which is worth more than either number alone.

### Gate 6

- On a synthetic rectified pair with known depths, recovered disparity matches
  truth within `0.1 px` and triangulated depth within `1%` at 5 m.
- On MH_01, the mean of `v0 - v1` is `< 0.1 px` in magnitude — a bias larger than
  this is a Step 2 failure surfacing here, and should be fixed there.
- The `v0 - v1` standard deviation is recorded in `BENCHMARKS.md` along with the
  derived `sigma_px`. This is a recorded measurement, not a gate with a bound,
  because there is no prior value to assert against.
- Disparity outside `[min, max]` is rejected and produces no observation.
- A match on a repetitive-texture region (checkerboard) either fails
  forward-backward or is caught by the disparity bounds; document which, because
  it is the failure mode most likely to reach the filter.
- The constrained and unconstrained passes agree in `u` within `0.2 px` on
  MH_01, confirming the row constraint is not distorting disparity.

## Step 7: Track Manager and Frontend Orchestration

New file `feature_frontend.hpp/cpp`. The stateful piece that everything else
feeds.

### Work

```cpp
enum class TrackState { kCandidate, kMapped };

struct FrontendOptions {
    DetectorOptions detector{};
    KltOptions temporal{};
    StereoMatchOptions stereo{};
    std::size_t max_mapped_landmarks = 100;
    int min_track_age_for_mapping = 2;      // frames survived before augmentation
    int max_consecutive_gated = 3;          // per-landmark health
};

struct FrontendFrame {
    TimestampNs timestamp;
    // Observations of tracks already in the filter. Ready for update_stereo_frame.
    std::vector<StereoObservation> mapped_observations;
    // Tracks eligible for augmentation after the update, with their pixels.
    std::vector<StereoObservation> birth_candidates;
    // Ids to hand to remove_landmarks at the frame boundary.
    std::vector<LandmarkId> dead_landmarks;
    int active_track_count = 0;
};

class FeatureFrontend {
public:
    static ParseResult<FeatureFrontend> create(
        const StereoRectification&, const FrontendOptions&);

    ParseResult<FrontendFrame> process(
        const GrayImage& cam0_raw, const GrayImage& cam1_raw, TimestampNs);

    // Fed back after update_stereo_frame so gating history informs track death.
    void report_outcomes(std::span<const LandmarkUpdateDiagnostics>);
};
```

Per-frame sequence inside `process`:

```text
1. rectify cam0 and cam1; build both pyramids
2. temporal KLT: previous cam0 pyramid -> current, for every live track,
   with forward-backward validation
3. stereo match every surviving track in the current frame
4. mark tracks that failed either step dead
5. detect new corners only in under-populated cells; stereo match them;
   admit as candidates
6. partition into mapped_observations and birth_candidates
7. capacity: if mapped + candidates would exceed max_mapped_landmarks,
   drop the youngest candidates first
```

And the caller's loop, which is what Step 8 implements:

```text
propagate_slam(...) over IMU up to the frame timestamp
update_stereo_frame(..., frame.mapped_observations, ...)
frontend.report_outcomes(result.diagnostics)
augment_landmark(...) for each birth_candidate, up to capacity
remove_landmarks(frame.dead_landmarks)
```

Removal last, and never inside the sweep, because it invalidates every offset
(`CONVENTIONS.md` §16).

**Track death** has four causes, and each maps to a different diagnostic: KLT
failure, forward-backward failure, stereo match loss, and repeated innovation
gating. The last one is the per-landmark health tracking that `ARCHITECTURE.md`
lists as open; `report_outcomes` is the whole mechanism. Count consecutive
`kGated` outcomes and kill at `max_consecutive_gated` — a landmark the filter
keeps rejecting is either badly triangulated or attached to a moving object, and
neither improves with time.

**`min_track_age_for_mapping`** exists because a track observed once has no
evidence of stability. Requiring it to survive two frames costs the first two
frames of information and filters out the single-frame blur artifacts that
otherwise become permanent, badly-placed landmarks. The right value is an
empirical question Step 8 can answer.

### Gate 7

- **IDs are never reused**, across any sequence of births and deaths. Assert
  directly on a long synthetic run: collect every ID ever issued and check for
  duplicates.
- A dead track is reported in `dead_landmarks` exactly once, and never appears in
  `mapped_observations` on the same frame.
- A candidate never appears in `mapped_observations` before it has been
  augmented; feeding candidates to the update would show up as
  `kUnknownLandmark` skips, which is harmless but hides real skips in the noise.
- `max_mapped_landmarks` is never exceeded, including in a frame where many
  births and many deaths coincide.
- `report_outcomes` with `max_consecutive_gated` consecutive `kGated` results
  kills the track, and a single interleaved `kApplied` resets the counter.
- The frontend holds no state offsets: verify by adding and removing landmarks
  between `process` calls and confirming the output is unaffected.
- On MH_01, report track length distribution (median, 90th percentile) and mean
  active track count. These are the numbers that say whether the tracker is
  actually working, and no synthetic test produces them.

## Step 8: EuRoC Closed-Loop Integration

New file `tests/euroc_frontend_test.cpp`, new CMake target
`euroc_frontend_tests`.

### Work

The full pipeline on real data: `parse_dataset` → rectify → track → propagate →
update → augment → remove, over MH_01, compared against ground truth.

Initialization is the known gap. `ARCHITECTURE.md` records that the filter is
consistent at an initial tilt error of `0.001 rad` and inconsistent at `0.01`,
and that static gravity alignment reaches the former. Initialize from the first
ground-truth state as the existing smoke test does, and record the fact that this
is not a real initialization — the frontend does not change that, and conflating
the two would obscure both.

Report, per run: ATE against ground truth, final position error, active landmark
count over time, observations applied/gated/skipped per frame, and wall-clock per
frame split across rectify / detect / track / match / filter.

### Gate 8

- **The headline.** Full-sequence MH_01 position error with the frontend and
  updates enabled is materially below the propagation-only baseline from Gate 0.
  Given that propagation-only diverges past 30 s, this should be a large factor,
  and if it is not, something upstream is wrong rather than marginal.
- The filter survives the full sequence without a non-finite state, a
  non-positive-definite `P`, or a capacity error.
- The gated fraction is reported per frame and stays below a recorded ceiling.
  A gated fraction that climbs over the sequence means the filter and the
  frontend are diverging from each other, which is the diagnostic that
  distinguishes a tracking failure from an estimator failure.
- Per-frame wall clock is inside the `50 ms` budget at 20 Hz on the dev machine,
  with the split recorded. Not a Jetson number, and labeled as such in
  `BENCHMARKS.md`.
- Deterministic: two runs over the same sequence produce identical trajectories.
- **The confound is stated, not hidden.** The updated-filter NEES is already
  known to be inconsistent (`24.23` against a `16.56` bound), attributed to
  update linearization. Real data adds tracking noise and residual outliers on
  top. Do not report a EuRoC NEES as evidence about linearization, and do not
  tune `sigma_px` to make a NEES number look better — the Step 6 estimate is
  measured independently and stays measured. The synthetic Monte Carlo test
  remains the controlled experiment.

## Step 9: Documentation

### Work

- **`CONVENTIONS.md` §17, new.** Frontend conventions: the pixel-center
  convention, the rectified pixel domain, that all frontend output is in
  rectified coordinates, the ID lifecycle rule (monotonic, never reused), and the
  rule that offsets are resolved at point of use.
- **`CONVENTIONS.md` §10.** Amend the note that says raw EuRoC calibration does
  not satisfy the rectified contract "until an undistortion/rectification layer
  is added" — it now exists. Add the `T_BC' = T_BC * blkdiag(R_C'C^T, 1)`
  composition explicitly.
- **`ARCHITECTURE.md`.** Add the new modules and public types. Move "Feature
  frontend" out of Open decisions into Design Decisions with the KLT choice and
  its conditions for revisiting. Partially close "Outlier rejection beyond the
  innovation gate": forward-backward and epipolar checks are implemented, RANSAC
  and robust costs stay open. Leave "Update linearization", "Initialization", and
  "Threading and time handling" open. Add the OpenCV dependency boundary as a
  build decision.
- **`scope.md`.** Check feature detection and tracking, and raw EuRoC
  undistortion/rectification. Landmark marginalization is partially addressed —
  the removal policy exists, but a proper marginalization decision (what to do
  with the information in a removed landmark) does not; note that rather than
  checking it.
- **`BENCHMARKS.md`.** Rectification row residual, `sigma_px`, track length
  distribution, MH_01 ATE with and without updates, per-frame timing split.
- **`present.md`.** Three things worth presenting: the rectification extrinsic
  composition as a bug that no self-consistency test catches; the derivation of
  `R` from the epipolar residual as an independent measurement rather than a
  tuned parameter; and the layered outlier defense, with the argument for why the
  innovation gate alone is insufficient.
- **`CLAUDE.md`.** Current-state paragraph, test count, and the OpenCV
  dependency in the build command.

### Gate 9

- `ARCHITECTURE.md` describes only what exists.
- No claim in `CONVENTIONS.md` contradicts the implementation.
- Test counts and build commands in `CLAUDE.md` match reality, including the
  OpenCV prerequisite.

## Cost Reference

Per frame at 20 Hz the budget is `50 ms`. EuRoC is `752x480 = 361k` pixels;
a 4-level pyramid is about `481k` pixels.

| stage | work | notes |
|---|---|---|
| PNG decode x2 | — | I/O and libpng bound, not arithmetic |
| remap x2 | `~4 MFLOP` | memory bound; 2 bilinear taps per output pixel |
| pyramid x2 | `~10 MFLOP` | separable 5-tap, 2 passes per level |
| detect | `~5 MFLOP` | only in under-populated cells, so usually far less |
| temporal KLT | `~8–30 MFLOP` | 200 features x 4 levels x `441` px x iterations |
| forward-backward | same again | the reason it is a deliberate decision |
| stereo KLT | `~4–15 MFLOP` | prior-seeded, mostly finest level only |
| filter update | `~2.4 MFLOP` | at `N = 50, m = 20`, from the update plan |

The arithmetic total is order `100 MFLOP/frame`, or `2 GFLOP/s` at 20 Hz. That is
not the constraint on a desktop and is a real number on an Orin Nano core.

The KLT iteration count is the term that actually varies, by roughly 4x between
a well-seeded track and a hard one, which is why Step 8 records the timing split
rather than a single total. The inverse-compositional Hessian precomputation is
what keeps the per-iteration cost at roughly `441 * 3` flops instead of
recomputing gradients and a `2x2` inverse each time.

Memory is unremarkable: two pyramids at both timestamps is about `2 MB`, and the
patch working set is `441` doubles per feature.

## Risks

- **Rectification correctness is load-bearing and quiet.** A wrong `t_bs`
  composition produces a self-consistent, plausible, biased system. Gate 2's
  extrinsic test is the only thing standing between that and a week of debugging
  the estimator. Do not weaken it.
- **`R` and NEES are entangled.** The filter is already documented as
  over-confident. An optimistic `sigma_px` produces the same symptom. Keeping the
  Step 6 estimate independent of the filter is what makes the two separable.
- **KLT on MH_01 is the easy case.** MH_01 is the slowest EuRoC sequence.
  Anything built and validated only here should be expected to fail on V1_03 and
  V2_03, and IMU-seeded prediction is the known fix.
- **OpenCV on Jetson.** `find_package(OpenCV)` on the dev machine says nothing
  about the aarch64 cross-compile in Phase 2. Keeping OpenCV to two source files
  is the mitigation, and it only works if the boundary is actually enforced.

## Out of Scope

Deliberately not in this plan:

- IMU-predicted feature locations for KLT seeding. First follow-up, with the
  track-survival metric from Step 7 as the measure.
- Two-view RANSAC and robust cost functions. The innovation gate and the
  forward-backward check are what this plan implements.
- Affine or gain-and-bias KLT warps. Revisit only if Gate 5's illumination and
  long-track measurements say so.
- Descriptor extraction, re-detection of lost landmarks, and anything approaching
  loop closure or relocalization — out of scope in `scope.md`.
- True marginalization of removed landmarks. Removal is row/column deletion per
  `CONVENTIONS.md` §12; what to do with the discarded information is a separate
  decision.
- Iterated EKF, OC-EKF, and first-estimates Jacobians. Unchanged by this plan.
- ATE/RPE/NEES as a reusable evaluation module. Step 8 computes ATE inline; the
  evaluation phase in `scope.md` stays separate.
- Multithreading and the ROS 2 node. Phase 2.
