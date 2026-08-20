#include "landmark_augmentation.hpp"
#include "measurement_update.hpp"
#include "propagation.hpp"
#include "synthetic.hpp"

#include <algorithm>
#include <array>
#include <numeric>
#include <vector>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

namespace {

constexpr int kPositionIndex = 0;
constexpr int kOrientationIndex = 6;
constexpr double kUngated = 1e12;

CameraCalibration make_cam0() {
    return make_synthetic_pinhole_camera_calibration(Eigen::Matrix4d::Identity());
}

CameraCalibration make_cam1() {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform(0, 3) = 0.2;
    return make_synthetic_pinhole_camera_calibration(transform);
}

SyntheticTrajectory make_trajectory() {
    return SyntheticTrajectory{
        .initial_position = Eigen::Vector3d{0.1, -0.2, 0.0},
        .initial_velocity = Eigen::Vector3d{0.3, -0.1, 0.0},
        .world_acceleration = Eigen::Vector3d{0.2, 0.1, 0.0},
        .sinusoidal_world_acceleration = Eigen::Vector3d{0.1, -0.15, 0.05},
        .sinusoidal_frequency_rad_per_sec = 1.3,
        .body_angular_velocity = Eigen::Vector3d{0.03, -0.02, 0.12},
    };
}

NominalState nominal_from(const GroundTruthState& truth) {
    return NominalState{
        .position = truth.position,
        .velocity = truth.velocity,
        .orientation = Sophus::SO3d{truth.orientation},
        .accelerometer_bias = truth.accelerometer_bias,
        .gyroscope_bias = truth.gyroscope_bias,
    };
}

// A state with landmarks in it and non-trivial robot/landmark correlation, which
// is what makes the batch oracle exercise the cross-covariance paths rather than
// just the block diagonal.
struct Scenario {
    SlamState state;
    std::vector<StereoObservation> observations;
    Eigen::Matrix4d pixel_covariance = 0.25 * Eigen::Matrix4d::Identity();
};

Scenario make_scenario(std::size_t landmark_count, double position_perturbation) {
    const SyntheticTrajectory trajectory = make_trajectory();
    const CameraCalibration cam0 = make_cam0();
    const CameraCalibration cam1 = make_cam1();
    auto landmarks = make_synthetic_landmarks();
    landmarks.resize(landmark_count);

    const auto observations = synthesize_stereo_observations(
        trajectory,
        landmarks,
        cam0,
        cam1,
        SyntheticCameraConfig{.rate_hz = 20.0, .duration_seconds = 0.2});
    EXPECT_TRUE(observations) << observations.error();

    // The observed frame and the propagated state must sit at the same instant,
    // otherwise every innovation carries the time offset and the gate fires on a
    // clean frame.
    const TimestampNs frame_timestamp = observations->back().timestamp;
    const double frame_time = trajectory.time_at(frame_timestamp);

    auto state_result = SlamState::create(
        landmarks.size(), nominal_from(trajectory.state_at(0.0)), 1e-4 * ImuStateCovariance::Identity());
    EXPECT_TRUE(state_result) << state_result.error();
    SlamState state = std::move(*state_result);

    const Eigen::Matrix4d pixel_covariance = 0.25 * Eigen::Matrix4d::Identity();
    for (const auto& observation : *observations) {
        if (observation.timestamp != trajectory.start_timestamp) {
            continue;
        }
        EXPECT_TRUE(augment_landmark(
            state,
            observation.landmark_id,
            observation.cam0_pixel,
            observation.cam1_pixel,
            cam0,
            cam1,
            pixel_covariance)) << observation.landmark_id;
    }

    // Propagation builds up the robot/landmark cross-covariance the update has to
    // handle correctly; without it P_rl stays zero and half the math is untested.
    const auto imu = synthesize_imu(
        trajectory, SyntheticImuConfig{.rate_hz = 200.0, .duration_seconds = frame_time});
    EXPECT_TRUE(imu) << imu.error();
    for (const auto& measurement : *imu) {
        EXPECT_TRUE(propagate_slam(
            state, measurement, make_synthetic_imu_calibration(200.0), 1.0 / 200.0));
    }

    // Offset the nominal state so predictions differ from the observations and
    // there is a real innovation to apply.
    state.robot.position.x() += position_perturbation;

    std::vector<StereoObservation> frame;
    for (const auto& observation : *observations) {
        if (observation.timestamp != frame_timestamp) {
            continue;
        }
        frame.push_back(StereoObservation{
            .id = observation.landmark_id,
            .pixel_cam0 = observation.cam0_pixel,
            .pixel_cam1 = observation.cam1_pixel,
        });
    }
    EXPECT_FALSE(frame.empty());

    return Scenario{.state = std::move(state), .observations = std::move(frame)};
}

Eigen::MatrixXd dense_jacobian(const StereoMeasurementBlocks& blocks, int dimension) {
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(kStereoMeasurementDim, dimension);
    jacobian.middleCols<3>(kPositionIndex) = blocks.pose.leftCols<3>();
    jacobian.middleCols<3>(kOrientationIndex) = blocks.pose.rightCols<3>();
    jacobian.middleCols<3>(blocks.landmark_offset) = blocks.landmark;
    return jacobian;
}

// Textbook batch update, written independently of the sequential implementation.
// This is the oracle, never the hot path.
void apply_batch_update(
    SlamState& state,
    const std::vector<StereoObservation>& observations,
    const Eigen::Matrix4d& pixel_covariance) {
    const CameraCalibration cam0 = make_cam0();
    const CameraCalibration cam1 = make_cam1();
    const int dimension = state.active_dim();
    const int rows = kStereoMeasurementDim * static_cast<int>(observations.size());

    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(rows, dimension);
    Eigen::MatrixXd noise = Eigen::MatrixXd::Zero(rows, rows);
    Eigen::VectorXd residual = Eigen::VectorXd::Zero(rows);

    for (std::size_t index = 0; index < observations.size(); ++index) {
        const auto offset = state.landmark_offset(observations[index].id);
        ASSERT_TRUE(offset) << offset.error();
        const auto position = state.landmark_position(observations[index].id);
        ASSERT_TRUE(position) << position.error();
        const auto blocks =
            make_stereo_measurement_blocks(state.robot, *position, cam0, cam1, *offset);
        ASSERT_TRUE(blocks) << blocks.error();
        ASSERT_TRUE(blocks->visible);

        const int row = kStereoMeasurementDim * static_cast<int>(index);
        jacobian.middleRows<kStereoMeasurementDim>(row) = dense_jacobian(*blocks, dimension);
        noise.block<kStereoMeasurementDim, kStereoMeasurementDim>(row, row) = pixel_covariance;
        Eigen::Vector4d measurement;
        measurement << observations[index].pixel_cam0, observations[index].pixel_cam1;
        residual.segment<kStereoMeasurementDim>(row) = measurement - blocks->prediction;
    }

    const Eigen::MatrixXd covariance = state.active_covariance();
    const Eigen::MatrixXd innovation_covariance =
        jacobian * covariance * jacobian.transpose() + noise;
    const Eigen::MatrixXd gain =
        covariance * jacobian.transpose() * innovation_covariance.inverse();
    const Eigen::VectorXd error_state = gain * residual;
    const Eigen::MatrixXd updated =
        (Eigen::MatrixXd::Identity(dimension, dimension) - gain * jacobian) * covariance;

    state.active_covariance() = updated;
    ASSERT_TRUE(state.inject_error_state(error_state));
}

void expect_states_match(const SlamState& left, const SlamState& right, double tolerance) {
    EXPECT_LT((left.robot.position - right.robot.position).cwiseAbs().maxCoeff(), tolerance);
    EXPECT_LT((left.robot.velocity - right.robot.velocity).cwiseAbs().maxCoeff(), tolerance);
    EXPECT_LT(
        (left.robot.orientation.inverse() * right.robot.orientation).log().cwiseAbs().maxCoeff(),
        tolerance);
    EXPECT_LT(
        (left.robot.accelerometer_bias - right.robot.accelerometer_bias).cwiseAbs().maxCoeff(),
        tolerance);
    EXPECT_LT(
        (left.robot.gyroscope_bias - right.robot.gyroscope_bias).cwiseAbs().maxCoeff(), tolerance);
    ASSERT_EQ(left.active_dim(), right.active_dim());
    EXPECT_LT(
        (Eigen::MatrixXd{left.active_covariance()} - Eigen::MatrixXd{right.active_covariance()})
            .cwiseAbs()
            .maxCoeff(),
        tolerance);
}

double smallest_eigenvalue(const SlamState& state) {
    const Eigen::MatrixXd covariance = state.active_covariance();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance);
    EXPECT_EQ(solver.info(), Eigen::Success);
    return solver.eigenvalues().minCoeff();
}

}  // namespace

// ---------------------------------------------------------------------------
// Gate 1: stereo measurement blocks
// ---------------------------------------------------------------------------

TEST(StereoMeasurementBlocksTest, MatchesCentralDifferencesOnTheManifold) {
    const CameraCalibration cam0 = make_cam0();
    const CameraCalibration cam1 = make_cam1();
    const NominalState robot{
        .position = Eigen::Vector3d{0.15, -0.3, 0.05},
        .velocity = Eigen::Vector3d::Zero(),
        .orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.05, -0.09, 0.12}),
        .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = Eigen::Vector3d::Zero(),
    };
    const Eigen::Vector3d landmark{0.7, 0.35, 6.0};
    const auto blocks = make_stereo_measurement_blocks(robot, landmark, cam0, cam1, 15);
    ASSERT_TRUE(blocks) << blocks.error();
    ASSERT_TRUE(blocks->visible);

    const double step = 1e-6;
    const auto predict = [&](const NominalState& state, const Eigen::Vector3d& point) {
        const auto perturbed = make_stereo_measurement_blocks(state, point, cam0, cam1, 15);
        EXPECT_TRUE(perturbed) << perturbed.error();
        return perturbed->prediction;
    };

    for (int axis = 0; axis < 3; ++axis) {
        const Eigen::Vector3d basis = Eigen::Vector3d::Unit(axis);

        NominalState forward_position = robot;
        NominalState backward_position = robot;
        forward_position.position += step * basis;
        backward_position.position -= step * basis;
        const Eigen::Vector4d position_derivative =
            (predict(forward_position, landmark) - predict(backward_position, landmark))
            / (2.0 * step);
        EXPECT_LT(
            (position_derivative - blocks->pose.leftCols<3>().col(axis)).cwiseAbs().maxCoeff(),
            1e-5)
            << "position axis " << axis;

        // Right/local perturbation, per CONVENTIONS.md section 2.
        NominalState forward_rotation = robot;
        NominalState backward_rotation = robot;
        forward_rotation.orientation = robot.orientation * Sophus::SO3d::exp(step * basis);
        backward_rotation.orientation = robot.orientation * Sophus::SO3d::exp(-step * basis);
        const Eigen::Vector4d rotation_derivative =
            (predict(forward_rotation, landmark) - predict(backward_rotation, landmark))
            / (2.0 * step);
        EXPECT_LT(
            (rotation_derivative - blocks->pose.rightCols<3>().col(axis)).cwiseAbs().maxCoeff(),
            1e-5)
            << "rotation axis " << axis;

        const Eigen::Vector4d landmark_derivative =
            (predict(robot, landmark + step * basis) - predict(robot, landmark - step * basis))
            / (2.0 * step);
        EXPECT_LT((landmark_derivative - blocks->landmark.col(axis)).cwiseAbs().maxCoeff(), 1e-5)
            << "landmark axis " << axis;
    }
}

// The convention check above only has power if the wrong convention would fail it.
TEST(StereoMeasurementBlocksTest, RejectsALeftGlobalRotationPerturbation) {
    const CameraCalibration cam0 = make_cam0();
    const CameraCalibration cam1 = make_cam1();
    const NominalState robot{
        .position = Eigen::Vector3d{0.15, -0.3, 0.05},
        .velocity = Eigen::Vector3d::Zero(),
        .orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.05, -0.09, 0.12}),
        .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = Eigen::Vector3d::Zero(),
    };
    const Eigen::Vector3d landmark{0.7, 0.35, 6.0};
    const auto blocks = make_stereo_measurement_blocks(robot, landmark, cam0, cam1, 15);
    ASSERT_TRUE(blocks) << blocks.error();

    const double step = 1e-6;
    double largest_mismatch = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        const Eigen::Vector3d basis = Eigen::Vector3d::Unit(axis);
        NominalState forward = robot;
        NominalState backward = robot;
        forward.orientation = Sophus::SO3d::exp(step * basis) * robot.orientation;
        backward.orientation = Sophus::SO3d::exp(-step * basis) * robot.orientation;
        const auto forward_blocks = make_stereo_measurement_blocks(forward, landmark, cam0, cam1, 15);
        const auto backward_blocks =
            make_stereo_measurement_blocks(backward, landmark, cam0, cam1, 15);
        ASSERT_TRUE(forward_blocks);
        ASSERT_TRUE(backward_blocks);
        const Eigen::Vector4d derivative =
            (forward_blocks->prediction - backward_blocks->prediction) / (2.0 * step);
        largest_mismatch = std::max(
            largest_mismatch,
            (derivative - blocks->pose.rightCols<3>().col(axis)).cwiseAbs().maxCoeff());
    }
    EXPECT_GT(largest_mismatch, 1e-3);
}

TEST(StereoMeasurementBlocksTest, StacksBothCamerasAndSurfacesDepth) {
    const CameraCalibration cam0 = make_cam0();
    const CameraCalibration cam1 = make_cam1();
    const NominalState robot{
        .position = Eigen::Vector3d::Zero(),
        .velocity = Eigen::Vector3d::Zero(),
        .orientation = Sophus::SO3d{},
        .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = Eigen::Vector3d::Zero(),
    };
    const Eigen::Vector3d landmark{0.0, 0.0, 5.0};
    const auto blocks = make_stereo_measurement_blocks(robot, landmark, cam0, cam1, 15);
    ASSERT_TRUE(blocks) << blocks.error();

    // Identity pose, landmark on the optical axis: cam0 sees the principal point
    // and cam1 sees it shifted by the baseline.
    EXPECT_NEAR(blocks->prediction(0), 320.0, 1e-9);
    EXPECT_NEAR(blocks->prediction(1), 240.0, 1e-9);
    EXPECT_NEAR(blocks->prediction(2), 320.0 - 500.0 * 0.2 / 5.0, 1e-9);
    EXPECT_NEAR(blocks->prediction(3), 240.0, 1e-9);
    EXPECT_NEAR(blocks->depth_cam0, 5.0, 1e-9);
    EXPECT_NEAR(blocks->depth_cam1, 5.0, 1e-9);
    EXPECT_TRUE(blocks->visible);
    EXPECT_EQ(blocks->landmark_offset, 15);
}

TEST(StereoMeasurementBlocksTest, ReportsLandmarksBehindTheCameraAsInvisible) {
    const NominalState robot{
        .position = Eigen::Vector3d::Zero(),
        .velocity = Eigen::Vector3d::Zero(),
        .orientation = Sophus::SO3d{},
        .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = Eigen::Vector3d::Zero(),
    };
    const auto blocks =
        make_stereo_measurement_blocks(robot, Eigen::Vector3d{0.3, 0.1, -5.0}, make_cam0(), make_cam1(), 15);
    ASSERT_TRUE(blocks) << blocks.error();
    EXPECT_FALSE(blocks->visible);
    // Finite pixels despite an invalid depth: isfinite() is not a visibility test.
    EXPECT_TRUE(blocks->prediction.allFinite());
    EXPECT_TRUE(blocks->pose.isZero());
    EXPECT_TRUE(blocks->landmark.isZero());
}

// ---------------------------------------------------------------------------
// Gate 3: single-landmark update against a dense reference
// ---------------------------------------------------------------------------

TEST(SequentialUpdateTest, SingleLandmarkMatchesTheDenseReference) {
    Scenario scenario = make_scenario(5, 0.03);
    SlamState sequential = scenario.state;
    SlamState reference = scenario.state;

    const std::vector<StereoObservation> single{scenario.observations.front()};
    const auto result = update_stereo_frame(
        sequential,
        single,
        make_cam0(),
        make_cam1(),
        scenario.pixel_covariance,
        UpdateOptions{.chi_square_threshold = kUngated});
    ASSERT_TRUE(result) << result.error();
    ASSERT_EQ(result->applied_count, 1);

    apply_batch_update(reference, single, scenario.pixel_covariance);
    expect_states_match(sequential, reference, 1e-12);
}

TEST(SequentialUpdateTest, CovarianceStaysSymmetricPositiveDefiniteAndDecreases) {
    Scenario scenario = make_scenario(5, 0.03);
    const Eigen::MatrixXd prior = scenario.state.active_covariance();
    SlamState updated = scenario.state;

    const auto result = update_stereo_frame(
        updated,
        scenario.observations,
        make_cam0(),
        make_cam1(),
        scenario.pixel_covariance,
        UpdateOptions{.chi_square_threshold = kUngated});
    ASSERT_TRUE(result) << result.error();
    ASSERT_GT(result->applied_count, 0);

    const Eigen::MatrixXd posterior = updated.active_covariance();
    EXPECT_LT((posterior - posterior.transpose()).cwiseAbs().maxCoeff(), 1e-14);
    EXPECT_GT(smallest_eigenvalue(updated), 0.0);

    // The update adds information, so the prior dominates it in the Loewner
    // order. The reset that follows is a similarity transform on the orientation
    // block, not an information change, and it is not Loewner-ordered against the
    // prior; undo it before comparing rather than absorbing it into a tolerance.
    const int dimension = updated.active_dim();
    const Eigen::Matrix3d reset =
        Sophus::SO3d::leftJacobian(-Eigen::Vector3d{result->error_state.segment<3>(kOrientationIndex)});
    const Eigen::Matrix3d undo = reset.inverse();
    Eigen::MatrixXd before_reset = posterior;
    before_reset.block(kOrientationIndex, 0, 3, dimension) =
        undo * before_reset.block(kOrientationIndex, 0, 3, dimension).eval();
    before_reset.block(0, kOrientationIndex, dimension, 3) =
        before_reset.block(0, kOrientationIndex, dimension, 3).eval() * undo.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(prior - before_reset);
    ASSERT_EQ(solver.info(), Eigen::Success);
    EXPECT_GT(solver.eigenvalues().minCoeff(), -1e-12);
}

TEST(SequentialUpdateTest, VanishinglyInformativeMeasurementsLeaveTheStateAlone) {
    Scenario scenario = make_scenario(5, 0.03);
    const Eigen::MatrixXd prior = scenario.state.active_covariance();
    const Eigen::Vector3d prior_position = scenario.state.robot.position;
    SlamState updated = scenario.state;

    const auto result = update_stereo_frame(
        updated,
        scenario.observations,
        make_cam0(),
        make_cam1(),
        1e12 * Eigen::Matrix4d::Identity(),
        UpdateOptions{.chi_square_threshold = kUngated});
    ASSERT_TRUE(result) << result.error();
    EXPECT_LT((Eigen::MatrixXd{updated.active_covariance()} - prior).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT((updated.robot.position - prior_position).cwiseAbs().maxCoeff(), 1e-9);
}

TEST(SequentialUpdateTest, JosephAndSimpleFormsAgree) {
    Scenario scenario = make_scenario(5, 0.03);
    SlamState joseph = scenario.state;
    SlamState simple = scenario.state;

    ASSERT_TRUE(update_stereo_frame(
        joseph, scenario.observations, make_cam0(), make_cam1(), scenario.pixel_covariance,
        UpdateOptions{.chi_square_threshold = kUngated, .use_joseph_form = true}));
    ASSERT_TRUE(update_stereo_frame(
        simple, scenario.observations, make_cam0(), make_cam1(), scenario.pixel_covariance,
        UpdateOptions{.chi_square_threshold = kUngated, .use_joseph_form = false}));

    // Equal in exact arithmetic under the optimal gain; they may differ only at
    // roundoff. A large gap means one of the two forms is wrong.
    expect_states_match(joseph, simple, 1e-10);
}

// ---------------------------------------------------------------------------
// Gate 4: batch equivalence and order invariance
// ---------------------------------------------------------------------------

TEST(SequentialUpdateTest, ThreeLandmarkSweepMatchesABatchUpdate) {
    Scenario scenario = make_scenario(3, 0.03);
    ASSERT_EQ(scenario.observations.size(), 3u);
    SlamState sequential = scenario.state;
    SlamState reference = scenario.state;

    const auto result = update_stereo_frame(
        sequential,
        scenario.observations,
        make_cam0(),
        make_cam1(),
        scenario.pixel_covariance,
        UpdateOptions{.chi_square_threshold = kUngated});
    ASSERT_TRUE(result) << result.error();
    ASSERT_EQ(result->applied_count, 3);

    apply_batch_update(reference, scenario.observations, scenario.pixel_covariance);
    expect_states_match(sequential, reference, 1e-12);
}

TEST(SequentialUpdateTest, FiveLandmarkSweepMatchesABatchUpdate) {
    Scenario scenario = make_scenario(5, 0.05);
    SlamState sequential = scenario.state;
    SlamState reference = scenario.state;

    const auto result = update_stereo_frame(
        sequential,
        scenario.observations,
        make_cam0(),
        make_cam1(),
        scenario.pixel_covariance,
        UpdateOptions{.chi_square_threshold = kUngated});
    ASSERT_TRUE(result) << result.error();
    ASSERT_EQ(result->applied_count, static_cast<int>(scenario.observations.size()));

    apply_batch_update(reference, scenario.observations, scenario.pixel_covariance);
    expect_states_match(sequential, reference, 1e-12);
}

TEST(SequentialUpdateTest, ResultIsInvariantToObservationOrder) {
    Scenario scenario = make_scenario(3, 0.03);
    ASSERT_EQ(scenario.observations.size(), 3u);

    std::vector<std::size_t> order(scenario.observations.size());
    std::iota(order.begin(), order.end(), 0u);
    std::sort(order.begin(), order.end());

    SlamState canonical = scenario.state;
    ASSERT_TRUE(update_stereo_frame(
        canonical, scenario.observations, make_cam0(), make_cam1(), scenario.pixel_covariance,
        UpdateOptions{.chi_square_threshold = kUngated}));

    int permutations = 0;
    do {
        std::vector<StereoObservation> permuted;
        permuted.reserve(order.size());
        for (const std::size_t index : order) {
            permuted.push_back(scenario.observations[index]);
        }
        SlamState candidate = scenario.state;
        ASSERT_TRUE(update_stereo_frame(
            candidate, permuted, make_cam0(), make_cam1(), scenario.pixel_covariance,
            UpdateOptions{.chi_square_threshold = kUngated}));
        expect_states_match(canonical, candidate, 1e-10);
        ++permutations;
    } while (std::next_permutation(order.begin(), order.end()));
    EXPECT_EQ(permutations, 6);
}

// ---------------------------------------------------------------------------
// Gate 4: skipped observations
// ---------------------------------------------------------------------------

TEST(SequentialUpdateTest, UnknownLandmarksAreSkippedNotErrors) {
    Scenario scenario = make_scenario(3, 0.03);
    SlamState state = scenario.state;
    const Eigen::MatrixXd prior = state.active_covariance();

    const std::array<StereoObservation, 1> unknown{StereoObservation{
        .id = 9999,
        .pixel_cam0 = Eigen::Vector2d{300.0, 200.0},
        .pixel_cam1 = Eigen::Vector2d{280.0, 200.0},
    }};
    const auto result = update_stereo_frame(
        state, unknown, make_cam0(), make_cam1(), scenario.pixel_covariance);
    ASSERT_TRUE(result) << result.error();
    EXPECT_EQ(result->applied_count, 0);
    EXPECT_EQ(result->skipped_count, 1);
    ASSERT_EQ(result->diagnostics.size(), 1u);
    EXPECT_EQ(result->diagnostics.front().outcome, ObservationOutcome::kUnknownLandmark);
    EXPECT_EQ(Eigen::MatrixXd{state.active_covariance()}, prior);
}

TEST(SequentialUpdateTest, LandmarksBehindTheCameraAreSkipped) {
    Scenario scenario = make_scenario(3, 0.03);
    SlamState state = scenario.state;
    // Rotate the robot away so every landmark falls behind the camera.
    state.robot.orientation = state.robot.orientation * Sophus::SO3d::exp(Eigen::Vector3d{0.0, M_PI, 0.0});
    const Eigen::MatrixXd prior = state.active_covariance();

    const auto result = update_stereo_frame(
        state, scenario.observations, make_cam0(), make_cam1(), scenario.pixel_covariance);
    ASSERT_TRUE(result) << result.error();
    EXPECT_EQ(result->applied_count, 0);
    EXPECT_EQ(result->skipped_count, static_cast<int>(scenario.observations.size()));
    for (const auto& diagnostics : result->diagnostics) {
        EXPECT_EQ(diagnostics.outcome, ObservationOutcome::kBehindCamera);
    }
    EXPECT_EQ(Eigen::MatrixXd{state.active_covariance()}, prior);
}

TEST(SequentialUpdateTest, AnEmptyFrameIsANoOp) {
    Scenario scenario = make_scenario(3, 0.03);
    SlamState state = scenario.state;
    const Eigen::MatrixXd prior = state.active_covariance();
    const Eigen::Vector3d prior_position = state.robot.position;

    const auto result = update_stereo_frame(
        state, std::span<const StereoObservation>{}, make_cam0(), make_cam1(),
        scenario.pixel_covariance);
    ASSERT_TRUE(result) << result.error();
    EXPECT_EQ(result->applied_count, 0);
    EXPECT_EQ(Eigen::MatrixXd{state.active_covariance()}, prior);
    EXPECT_EQ(state.robot.position, prior_position);
}

// ---------------------------------------------------------------------------
// Gate 5: chi-square gating
// ---------------------------------------------------------------------------

TEST(GatingTest, CleanFrameAdmitsEveryObservation) {
    Scenario scenario = make_scenario(5, 0.0);
    SlamState state = scenario.state;
    const auto result = update_stereo_frame(
        state, scenario.observations, make_cam0(), make_cam1(), scenario.pixel_covariance);
    ASSERT_TRUE(result) << result.error();
    EXPECT_EQ(result->applied_count, static_cast<int>(scenario.observations.size()));
    EXPECT_EQ(result->gated_count, 0);
}

TEST(GatingTest, AnOutlierIsRejectedAndLeavesNoTrace) {
    Scenario scenario = make_scenario(5, 0.0);
    std::vector<StereoObservation> clean = scenario.observations;
    ASSERT_GE(clean.size(), 2u);

    SlamState without_outlier = scenario.state;
    const auto clean_result = update_stereo_frame(
        without_outlier, clean, make_cam0(), make_cam1(), scenario.pixel_covariance);
    ASSERT_TRUE(clean_result) << clean_result.error();
    ASSERT_EQ(clean_result->gated_count, 0);

    // Displace one observation far outside the gate.
    std::vector<StereoObservation> corrupted = clean;
    corrupted[1].pixel_cam0.x() += 50.0;

    SlamState with_outlier = scenario.state;
    const auto corrupted_result = update_stereo_frame(
        with_outlier, corrupted, make_cam0(), make_cam1(), scenario.pixel_covariance);
    ASSERT_TRUE(corrupted_result) << corrupted_result.error();
    EXPECT_EQ(corrupted_result->gated_count, 1);
    EXPECT_EQ(corrupted_result->applied_count, clean_result->applied_count - 1);
    EXPECT_EQ(corrupted_result->diagnostics[1].outcome, ObservationOutcome::kGated);

    // The rejected observation must leave the remaining result untouched, so
    // compare against a clean run with that observation removed entirely.
    std::vector<StereoObservation> without = clean;
    without.erase(without.begin() + 1);
    SlamState removed = scenario.state;
    ASSERT_TRUE(update_stereo_frame(
        removed, without, make_cam0(), make_cam1(), scenario.pixel_covariance));
    expect_states_match(with_outlier, removed, 1e-12);
}

TEST(GatingTest, GateDecisionsAreIndependentOfObservationOrder) {
    Scenario scenario = make_scenario(5, 0.0);
    std::vector<StereoObservation> corrupted = scenario.observations;
    corrupted[2].pixel_cam1.y() += 40.0;

    SlamState forward = scenario.state;
    const auto forward_result = update_stereo_frame(
        forward, corrupted, make_cam0(), make_cam1(), scenario.pixel_covariance);
    ASSERT_TRUE(forward_result) << forward_result.error();

    std::vector<StereoObservation> reversed = corrupted;
    std::reverse(reversed.begin(), reversed.end());
    SlamState backward = scenario.state;
    const auto backward_result = update_stereo_frame(
        backward, reversed, make_cam0(), make_cam1(), scenario.pixel_covariance);
    ASSERT_TRUE(backward_result) << backward_result.error();

    EXPECT_EQ(forward_result->gated_count, backward_result->gated_count);
    EXPECT_EQ(forward_result->applied_count, backward_result->applied_count);
    expect_states_match(forward, backward, 1e-10);
}

TEST(GatingTest, AZeroThresholdRejectsEverythingAndAHugeOneAdmitsEverything) {
    Scenario scenario = make_scenario(5, 0.03);

    SlamState closed = scenario.state;
    const Eigen::MatrixXd prior = closed.active_covariance();
    const auto closed_result = update_stereo_frame(
        closed, scenario.observations, make_cam0(), make_cam1(), scenario.pixel_covariance,
        UpdateOptions{.chi_square_threshold = 0.0});
    ASSERT_TRUE(closed_result) << closed_result.error();
    EXPECT_EQ(closed_result->applied_count, 0);
    EXPECT_EQ(closed_result->gated_count, static_cast<int>(scenario.observations.size()));
    EXPECT_EQ(Eigen::MatrixXd{closed.active_covariance()}, prior);

    SlamState open = scenario.state;
    const auto open_result = update_stereo_frame(
        open, scenario.observations, make_cam0(), make_cam1(), scenario.pixel_covariance,
        UpdateOptions{.chi_square_threshold = kUngated});
    ASSERT_TRUE(open_result) << open_result.error();
    EXPECT_EQ(open_result->applied_count, static_cast<int>(scenario.observations.size()));
}

TEST(GatingTest, DiagnosticsReportADistanceForRejectedObservations) {
    Scenario scenario = make_scenario(5, 0.0);
    std::vector<StereoObservation> corrupted = scenario.observations;
    corrupted[0].pixel_cam0.x() += 50.0;

    SlamState state = scenario.state;
    const auto result = update_stereo_frame(
        state, corrupted, make_cam0(), make_cam1(), scenario.pixel_covariance);
    ASSERT_TRUE(result) << result.error();
    ASSERT_EQ(result->diagnostics.size(), corrupted.size());
    EXPECT_EQ(result->diagnostics[0].outcome, ObservationOutcome::kGated);
    EXPECT_GT(result->diagnostics[0].mahalanobis_distance, 9.4877);
    EXPECT_TRUE(std::isfinite(result->diagnostics[0].mahalanobis_distance));
    EXPECT_GT(std::abs(result->diagnostics[0].innovation(0)), 40.0);
    for (std::size_t index = 1; index < result->diagnostics.size(); ++index) {
        EXPECT_LT(result->diagnostics[index].mahalanobis_distance, 9.4877);
    }
}

TEST(GatingTest, RejectsANonFiniteThreshold) {
    Scenario scenario = make_scenario(3, 0.0);
    SlamState state = scenario.state;
    EXPECT_FALSE(update_stereo_frame(
        state, scenario.observations, make_cam0(), make_cam1(), scenario.pixel_covariance,
        UpdateOptions{.chi_square_threshold = std::numeric_limits<double>::quiet_NaN()}));
    EXPECT_FALSE(update_stereo_frame(
        state, scenario.observations, make_cam0(), make_cam1(), scenario.pixel_covariance,
        UpdateOptions{.chi_square_threshold = -1.0}));
    EXPECT_FALSE(update_stereo_frame(
        state, scenario.observations, make_cam0(), make_cam1(), scenario.pixel_covariance,
        UpdateOptions{.max_iterations = 0}));
}

TEST(GatingTest, RejectsNonFinitePixelCovariance) {
    Scenario scenario = make_scenario(3, 0.0);
    SlamState state = scenario.state;
    Eigen::Matrix4d covariance = Eigen::Matrix4d::Identity();
    covariance(2, 2) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(update_stereo_frame(
        state, scenario.observations, make_cam0(), make_cam1(), covariance));
}
