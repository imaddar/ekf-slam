#pragma once

#include "measurement_model.hpp"
#include "parser.hpp"
#include "slam_state.hpp"
#include "state.hpp"
#include "types.hpp"

#include <span>
#include <vector>

#include <Eigen/Core>

// Stereo pixel ordering is [u0, v0, u1, v1], matching triangulate_stereo.
inline constexpr int kStereoMeasurementDim = 4;
// A stereo row block has 9 nonzero state columns: position 0..2, orientation
// 6..8, and the observed landmark. See CONVENTIONS.md section 9.
inline constexpr int kMeasurementSparseColumns = 9;

// One landmark's stacked stereo Jacobian. The pose block keeps the compact
// [delta p_W | delta theta_B] ordering of MeasurementJacobianBlocks; scattering
// its halves to the non-contiguous state columns happens in the update.
struct StereoMeasurementBlocks {
    Eigen::Matrix<double, kStereoMeasurementDim, 6> pose;
    Eigen::Matrix<double, kStereoMeasurementDim, kLandmarkDim> landmark;
    Eigen::Vector4d prediction;
    double depth_cam0;
    double depth_cam1;
    int landmark_offset;
    // False when either depth is non-positive. Jacobians are zero in that case:
    // a negative depth still projects to a finite pixel, so the caller cannot
    // use isfinite() as a visibility test (CONVENTIONS.md section 8).
    bool visible;
};

ParseResult<StereoMeasurementBlocks> make_stereo_measurement_blocks(
    const NominalState& robot,
    const Eigen::Vector3d& landmark_world,
    const CameraCalibration& cam0,
    const CameraCalibration& cam1,
    int landmark_offset);

struct StereoObservation {
    LandmarkId id = 0;
    Eigen::Vector2d pixel_cam0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d pixel_cam1 = Eigen::Vector2d::Zero();
};

enum class ObservationOutcome {
    kApplied,
    kGated,
    kUnknownLandmark,
    kBehindCamera,
};

struct LandmarkUpdateDiagnostics {
    LandmarkId id;
    ObservationOutcome outcome;
    // Gate statistic against the prior covariance; NaN when the gate did not run.
    double mahalanobis_distance;
    // Residual against the frame-entry nominal state, before any update.
    Eigen::Vector4d innovation;
};

struct UpdateOptions {
    // 4 degrees of freedom: 9.4877 at p=0.95, 13.2767 at p=0.99.
    double chi_square_threshold = 9.4877;
    // Joseph form is algebraically identical to the simple form under the
    // optimal gain; it is retained because it degrades more gracefully across a
    // sweep of successive updates. Disable it only to compare the two.
    bool use_joseph_form = true;
};

struct StereoUpdateResult {
    int applied_count = 0;
    int gated_count = 0;
    int skipped_count = 0;
    // The accumulated correction injected at the end of the frame, in the
    // error-state ordering of CONVENTIONS.md section 1. Empty when nothing was
    // applied. The covariance reset that follows injection is a function of its
    // rotation block, so this is also what a caller needs to reason about the
    // reset after the fact.
    Eigen::VectorXd error_state;
    std::vector<LandmarkUpdateDiagnostics> diagnostics;
};

// Sequential per-landmark EKF update over one stereo frame.
//
// Every Jacobian and residual is built from the nominal state as it stands at
// entry, the error state accumulates across the sweep, and injection happens
// once at the end. Freezing the linearization point is what makes the sweep
// equal a batch update over the same observations, and it keeps the result
// independent of observation order.
//
// pixel_covariance is per-pixel detector noise in [u0, v0, u1, v1] ordering.
// It is not StereoTriangulation::covariance, which is the propagated 3D
// position covariance already reflected in P_ll from the landmark's birth.
//
// Observations whose landmark is not in the state are reported as skipped, not
// treated as errors; the caller decides whether to augment them afterwards. A
// landmark must never be updated in the frame that created it.
ParseResult<StereoUpdateResult> update_stereo_frame(
    SlamState& state,
    std::span<const StereoObservation> observations,
    const CameraCalibration& cam0,
    const CameraCalibration& cam1,
    const Eigen::Matrix4d& pixel_covariance,
    const UpdateOptions& options = {});
