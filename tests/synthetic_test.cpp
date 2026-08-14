#include "propagation.hpp"
#include "synthetic.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <ranges>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

namespace {

constexpr double kTightTolerance = 1e-9;
constexpr double kPropagationTolerance = 4e-3;
constexpr double kConvergenceSlack = 0.25;
// Sample standard deviation over ~20k draws lands well inside this band.
constexpr double kNoiseStddevTolerance = 0.05;

NominalState make_nominal_state(const GroundTruthState& truth) {
    return {
        .position = truth.position,
        .velocity = truth.velocity,
        .orientation = Sophus::SO3d{truth.orientation},
        .accelerometer_bias = truth.accelerometer_bias,
        .gyroscope_bias = truth.gyroscope_bias,
    };
}

ParseResult<PropagationResult> propagate_over_measurements(
    const SyntheticTrajectory& trajectory,
    const std::vector<ImuMeasurement>& measurements,
    const ImuCalibration& imu_calibration) {
    auto state = make_nominal_state(trajectory.state_at(0.0));
    StateCovariance covariance = StateCovariance::Zero();

    for (std::size_t i = 1; i < measurements.size(); ++i) {
        const double dt_seconds =
            static_cast<double>(measurements[i].timestamp - measurements[i - 1].timestamp) * 1e-9;
        const auto result = propagate(state, measurements[i - 1], imu_calibration, dt_seconds, covariance);
        if (!result) {
            return std::unexpected(result.error());
        }
        state = result->nominal_state;
        covariance = result->covariance;
    }

    return PropagationResult{
        .nominal_state = state,
        .covariance = covariance,
    };
}

// Truth at the instant the stream actually ends, which is not necessarily the
// requested duration when it is not a whole number of sample steps.
GroundTruthState truth_at_last_sample(
    const SyntheticTrajectory& trajectory,
    const std::vector<ImuMeasurement>& measurements) {
    return trajectory.state_at(trajectory.time_at(measurements.back().timestamp));
}

// Absolute error, unlike Eigen's isApprox, which is relative and so degenerates
// into exact equality whenever the reference vector is zero.
double vector_error(const Eigen::Vector3d& estimated, const Eigen::Vector3d& truth) {
    return (estimated - truth).norm();
}

double orientation_error_rad(const Sophus::SO3d& estimated, const Eigen::Quaterniond& truth) {
    return (Sophus::SO3d{truth}.inverse() * estimated).log().norm();
}

ParseResult<double> propagation_translation_velocity_error(
    const SyntheticTrajectory& trajectory,
    double rate_hz,
    double duration_seconds) {
    const SyntheticImuConfig config{
        .rate_hz = rate_hz,
        .duration_seconds = duration_seconds,
    };
    const auto measurements = synthesize_imu(trajectory, config);
    if (!measurements) {
        return std::unexpected(measurements.error());
    }

    const auto imu_calibration = make_synthetic_imu_calibration(config.rate_hz);
    const auto result = propagate_over_measurements(trajectory, *measurements, imu_calibration);
    if (!result) {
        return std::unexpected(result.error());
    }

    const auto truth = truth_at_last_sample(trajectory, *measurements);

    return vector_error(result->nominal_state.position, truth.position)
        + vector_error(result->nominal_state.velocity, truth.velocity);
}

bool imu_samples_match(const std::vector<ImuMeasurement>& lhs, const std::vector<ImuMeasurement>& rhs) {
    if (lhs.size() != rhs.size()) {
        return false;
    }

    for (std::size_t i = 0; i < lhs.size(); ++i) {
        if (vector_error(lhs[i].acceleration, rhs[i].acceleration) > kTightTolerance
            || vector_error(lhs[i].angular_velocity, rhs[i].angular_velocity) > kTightTolerance) {
            return false;
        }
    }

    return true;
}

bool stereo_observations_match(
    const std::vector<SyntheticStereoObservation>& lhs,
    const std::vector<SyntheticStereoObservation>& rhs) {
    if (lhs.size() != rhs.size()) {
        return false;
    }

    for (std::size_t i = 0; i < lhs.size(); ++i) {
        if (lhs[i].timestamp != rhs[i].timestamp || lhs[i].landmark_id != rhs[i].landmark_id
            || (lhs[i].cam0_pixel - rhs[i].cam0_pixel).norm() > kTightTolerance
            || (lhs[i].cam1_pixel - rhs[i].cam1_pixel).norm() > kTightTolerance
            || std::abs(lhs[i].disparity_pixels - rhs[i].disparity_pixels) > kTightTolerance) {
            return false;
        }
    }

    return true;
}

Eigen::Matrix4d make_transform_with_translation(const Eigen::Vector3d& translation) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 1>(0, 3) = translation;
    return transform;
}

CameraCalibration make_cam0() {
    return make_synthetic_pinhole_camera_calibration(Eigen::Matrix4d::Identity());
}

CameraCalibration make_cam1(const Eigen::Vector3d& baseline = Eigen::Vector3d{0.2, 0.0, 0.0}) {
    return make_synthetic_pinhole_camera_calibration(make_transform_with_translation(baseline));
}

double sample_stddev(const std::vector<double>& values) {
    const double mean = std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
    double sum_squares = 0.0;
    for (const double value : values) {
        sum_squares += (value - mean) * (value - mean);
    }

    return std::sqrt(sum_squares / static_cast<double>(values.size() - 1));
}

}  // namespace

TEST(SyntheticTest, GeneratesMonotonicImuTimestampsAtConfiguredRate) {
    SyntheticTrajectory trajectory;
    trajectory.start_timestamp = 10'000;
    const SyntheticImuConfig config{
        .rate_hz = 200.0,
        .duration_seconds = 0.02,
    };

    const auto measurements = synthesize_imu(trajectory, config);

    ASSERT_TRUE(measurements) << measurements.error();
    ASSERT_EQ(measurements->size(), 5);
    EXPECT_TRUE(std::ranges::is_sorted(*measurements, {}, &ImuMeasurement::timestamp));
    EXPECT_EQ(measurements->front().timestamp, 10'000);
    EXPECT_EQ((*measurements)[1].timestamp - measurements->front().timestamp, 5'000'000);
}

// 0.29 * 200 is 57.999999999999993 in double, so a plain floor loses the last sample
// and every propagation test would then compare against truth the stream never reached.
TEST(SyntheticTest, KeepsFinalSampleWhenTheStepProductIsNotRepresentable) {
    SyntheticTrajectory trajectory;
    const SyntheticImuConfig config{
        .rate_hz = 200.0,
        .duration_seconds = 0.29,
    };

    const auto measurements = synthesize_imu(trajectory, config);

    ASSERT_TRUE(measurements) << measurements.error();
    EXPECT_EQ(measurements->size(), 59);
    EXPECT_EQ(measurements->back().timestamp, 290'000'000);
}

// A duration that is not a whole number of steps truncates to the last grid point.
TEST(SyntheticTest, TruncatesDurationsThatDoNotLandOnTheSampleGrid) {
    SyntheticTrajectory trajectory;
    const SyntheticImuConfig config{
        .rate_hz = 200.0,
        .duration_seconds = 0.0071,
    };

    const auto measurements = synthesize_imu(trajectory, config);

    ASSERT_TRUE(measurements) << measurements.error();
    EXPECT_EQ(measurements->size(), 2);
    EXPECT_EQ(measurements->back().timestamp, 5'000'000);
    EXPECT_NEAR(trajectory.time_at(measurements->back().timestamp), 0.005, kTightTolerance);
}

TEST(SyntheticTest, SamplesGroundTruthOnTheTrajectoryGrid) {
    SyntheticTrajectory trajectory{
        .initial_position = Eigen::Vector3d{1.0, -2.0, 0.5},
        .initial_velocity = Eigen::Vector3d{0.2, -0.3, 0.4},
        .world_acceleration = Eigen::Vector3d{0.6, -0.4, 0.25},
        .body_angular_velocity = Eigen::Vector3d{0.1, -0.2, 0.3},
    };
    trajectory.start_timestamp = 1'000'000'000;

    const auto states = sample_ground_truth(trajectory, 200.0, 0.02);

    ASSERT_TRUE(states) << states.error();
    ASSERT_EQ(states->size(), 5);
    EXPECT_EQ(states->front().timestamp, 1'000'000'000);
    EXPECT_EQ(states->back().timestamp, 1'020'000'000);
    for (const auto& state : *states) {
        const auto expected = trajectory.state_at(trajectory.time_at(state.timestamp));
        EXPECT_LT(vector_error(state.position, expected.position), kTightTolerance);
        EXPECT_LT(vector_error(state.velocity, expected.velocity), kTightTolerance);
        EXPECT_LT(orientation_error_rad(Sophus::SO3d{state.orientation}, expected.orientation), kTightTolerance);
    }
}

// The trajectory owns the time origin, so every stream derived from it must agree.
TEST(SyntheticTest, GroundTruthAndImuShareTheTrajectoryTimeOrigin) {
    SyntheticTrajectory trajectory;
    trajectory.start_timestamp = 1'403'636'579'763'555'584;
    const SyntheticImuConfig config{
        .rate_hz = 200.0,
        .duration_seconds = 0.02,
    };

    const auto states = sample_ground_truth(trajectory, config.rate_hz, config.duration_seconds);
    const auto measurements = synthesize_imu(trajectory, config);

    ASSERT_TRUE(states) << states.error();
    ASSERT_TRUE(measurements) << measurements.error();
    ASSERT_EQ(states->size(), measurements->size());
    for (std::size_t i = 0; i < states->size(); ++i) {
        EXPECT_EQ((*states)[i].timestamp, (*measurements)[i].timestamp);
    }
}

// The trajectory also owns the true biases, so reported truth and the IMU stream
// cannot disagree about them.
TEST(SyntheticTest, TrajectoryBiasesReachBothTruthAndTheImuStream) {
    const SyntheticTrajectory trajectory{
        .accelerometer_bias = Eigen::Vector3d{0.5, -0.25, 0.125},
        .gyroscope_bias = Eigen::Vector3d{-0.04, 0.05, -0.06},
    };
    const SyntheticImuConfig config{
        .rate_hz = 200.0,
        .duration_seconds = 0.0,
    };

    const auto measurements = synthesize_imu(trajectory, config);

    ASSERT_TRUE(measurements) << measurements.error();
    ASSERT_EQ(measurements->size(), 1);
    const auto truth = trajectory.state_at(0.0);
    EXPECT_LT(vector_error(truth.accelerometer_bias, trajectory.accelerometer_bias), kTightTolerance);
    EXPECT_LT(vector_error(truth.gyroscope_bias, trajectory.gyroscope_bias), kTightTolerance);
    EXPECT_LT(
        vector_error(
            measurements->front().acceleration,
            Eigen::Vector3d{0.0, 0.0, 9.81} + trajectory.accelerometer_bias),
        kTightTolerance);
    EXPECT_LT(
        vector_error(measurements->front().angular_velocity, trajectory.gyroscope_bias), kTightTolerance);
}

TEST(SyntheticTest, RejectsInvalidSamplingConfiguration) {
    const SyntheticTrajectory trajectory;

    EXPECT_FALSE(sample_ground_truth(trajectory, 0.0, 1.0));
    EXPECT_FALSE(sample_ground_truth(trajectory, -200.0, 1.0));
    EXPECT_FALSE(sample_ground_truth(trajectory, 200.0, -1.0));
    EXPECT_FALSE(sample_ground_truth(trajectory, std::nan(""), 1.0));
    EXPECT_FALSE(synthesize_imu(trajectory, {.rate_hz = 0.0, .duration_seconds = 1.0}));
    EXPECT_FALSE(synthesize_imu(trajectory, {.rate_hz = 200.0, .duration_seconds = -1.0}));
    EXPECT_FALSE(synthesize_imu(
        trajectory,
        {.rate_hz = 200.0,
         .duration_seconds = 1.0,
         .noise = SyntheticImuNoiseConfig{.accelerometer_noise_density = -1.0}}));
}

TEST(SyntheticTest, StationarySyntheticImuMatchesPropagationGravityConvention) {
    SyntheticTrajectory trajectory;
    const SyntheticImuConfig config{
        .rate_hz = 200.0,
        .duration_seconds = 0.0,
    };

    const auto measurements = synthesize_imu(trajectory, config);

    ASSERT_TRUE(measurements) << measurements.error();
    ASSERT_EQ(measurements->size(), 1);
    EXPECT_LT(
        vector_error(measurements->front().acceleration, Eigen::Vector3d{0.0, 0.0, 9.81}), kTightTolerance);
    EXPECT_LT(measurements->front().angular_velocity.norm(), kTightTolerance);
}

TEST(SyntheticTest, StationarySyntheticImuKeepsStateStationary) {
    SyntheticTrajectory trajectory;
    const SyntheticImuConfig config{
        .rate_hz = 200.0,
        .duration_seconds = 1.0,
    };
    const auto measurements = synthesize_imu(trajectory, config);
    ASSERT_TRUE(measurements) << measurements.error();
    const auto imu_calibration = make_synthetic_imu_calibration(config.rate_hz);

    const auto result = propagate_over_measurements(trajectory, *measurements, imu_calibration);
    ASSERT_TRUE(result) << result.error();
    const auto truth = truth_at_last_sample(trajectory, *measurements);

    EXPECT_LT(vector_error(result->nominal_state.position, truth.position), kTightTolerance);
    EXPECT_LT(vector_error(result->nominal_state.velocity, truth.velocity), kTightTolerance);
    EXPECT_LT(orientation_error_rad(result->nominal_state.orientation, truth.orientation), kTightTolerance);
}

TEST(SyntheticTest, ConstantAccelerationIntegratesToAnalyticTruth) {
    SyntheticTrajectory trajectory{
        .initial_position = Eigen::Vector3d{1.0, -2.0, 0.5},
        .initial_velocity = Eigen::Vector3d{0.5, -0.2, 0.1},
        .world_acceleration = Eigen::Vector3d{0.3, -0.4, 0.2},
    };
    const SyntheticImuConfig config{
        .rate_hz = 200.0,
        .duration_seconds = 1.0,
    };
    const auto measurements = synthesize_imu(trajectory, config);
    ASSERT_TRUE(measurements) << measurements.error();
    const auto imu_calibration = make_synthetic_imu_calibration(config.rate_hz);

    const auto result = propagate_over_measurements(trajectory, *measurements, imu_calibration);
    ASSERT_TRUE(result) << result.error();
    const auto truth = truth_at_last_sample(trajectory, *measurements);

    EXPECT_LT(vector_error(result->nominal_state.position, truth.position), kTightTolerance);
    EXPECT_LT(vector_error(result->nominal_state.velocity, truth.velocity), kTightTolerance);
}

TEST(SyntheticTest, ConstantYawRateIntegratesToAnalyticTruth) {
    SyntheticTrajectory trajectory{
        .body_angular_velocity = Eigen::Vector3d{0.0, 0.0, 0.4},
    };
    const SyntheticImuConfig config{
        .rate_hz = 200.0,
        .duration_seconds = 1.0,
    };
    const auto measurements = synthesize_imu(trajectory, config);
    ASSERT_TRUE(measurements) << measurements.error();
    const auto imu_calibration = make_synthetic_imu_calibration(config.rate_hz);

    const auto result = propagate_over_measurements(trajectory, *measurements, imu_calibration);
    ASSERT_TRUE(result) << result.error();
    const auto truth = truth_at_last_sample(trajectory, *measurements);

    EXPECT_LT(orientation_error_rad(result->nominal_state.orientation, truth.orientation), kTightTolerance);
}

TEST(SyntheticTest, FixedBiasesAreRemovedDuringPropagation) {
    SyntheticTrajectory trajectory{
        .world_acceleration = Eigen::Vector3d{0.2, -0.1, 0.3},
        .body_angular_velocity = Eigen::Vector3d{0.1, -0.2, 0.3},
        .accelerometer_bias = Eigen::Vector3d{0.01, -0.02, 0.03},
        .gyroscope_bias = Eigen::Vector3d{-0.04, 0.05, -0.06},
    };
    const SyntheticImuConfig config{
        .rate_hz = 200.0,
        .duration_seconds = 1.0,
    };
    const auto measurements = synthesize_imu(trajectory, config);
    ASSERT_TRUE(measurements) << measurements.error();
    const auto imu_calibration = make_synthetic_imu_calibration(config.rate_hz);

    const auto result = propagate_over_measurements(trajectory, *measurements, imu_calibration);
    ASSERT_TRUE(result) << result.error();
    const auto truth = truth_at_last_sample(trajectory, *measurements);

    EXPECT_LT(vector_error(result->nominal_state.position, truth.position), kPropagationTolerance);
    EXPECT_LT(vector_error(result->nominal_state.velocity, truth.velocity), kPropagationTolerance);
    EXPECT_LT(orientation_error_rad(result->nominal_state.orientation, truth.orientation), kTightTolerance);
}

TEST(SyntheticTest, FullyExcitedTrajectoryPropagatesNearAnalyticTruth) {
    SyntheticTrajectory trajectory{
        .initial_position = Eigen::Vector3d{1.0, -2.0, 0.5},
        .initial_velocity = Eigen::Vector3d{0.2, -0.3, 0.4},
        .initial_orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.2, -0.1, 0.3}),
        .world_acceleration = Eigen::Vector3d{0.6, -0.4, 0.25},
        .body_angular_velocity = Eigen::Vector3d{0.35, -0.25, 0.45},
    };
    const SyntheticImuConfig config{
        .rate_hz = 200.0,
        .duration_seconds = 2.0,
    };
    const auto measurements = synthesize_imu(trajectory, config);
    ASSERT_TRUE(measurements) << measurements.error();
    const auto imu_calibration = make_synthetic_imu_calibration(config.rate_hz);

    const auto result = propagate_over_measurements(trajectory, *measurements, imu_calibration);
    ASSERT_TRUE(result) << result.error();
    const auto truth = truth_at_last_sample(trajectory, *measurements);
    const Eigen::SelfAdjointEigenSolver<StateCovariance> eigen_solver(result->covariance);

    EXPECT_LT(vector_error(result->nominal_state.position, truth.position), kPropagationTolerance);
    EXPECT_LT(vector_error(result->nominal_state.velocity, truth.velocity), kPropagationTolerance);
    EXPECT_LT(orientation_error_rad(result->nominal_state.orientation, truth.orientation), kTightTolerance);
    EXPECT_TRUE(result->covariance.allFinite());
    EXPECT_LT((result->covariance - result->covariance.transpose()).norm(), kTightTolerance);
    EXPECT_GE(eigen_solver.eigenvalues().minCoeff(), -kTightTolerance);
}

TEST(SyntheticTest, FullyExcitedTrajectoryErrorScalesWithTimestep) {
    const SyntheticTrajectory trajectory{
        .initial_position = Eigen::Vector3d{1.0, -2.0, 0.5},
        .initial_velocity = Eigen::Vector3d{0.2, -0.3, 0.4},
        .initial_orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.2, -0.1, 0.3}),
        .world_acceleration = Eigen::Vector3d{0.6, -0.4, 0.25},
        .sinusoidal_world_acceleration = Eigen::Vector3d{0.3, -0.2, 0.15},
        .sinusoidal_frequency_rad_per_sec = 1.7,
        .body_angular_velocity = Eigen::Vector3d{0.35, -0.25, 0.45},
    };

    const auto error_200_hz = propagation_translation_velocity_error(trajectory, 200.0, 2.0);
    const auto error_400_hz = propagation_translation_velocity_error(trajectory, 400.0, 2.0);
    const auto error_800_hz = propagation_translation_velocity_error(trajectory, 800.0, 2.0);

    ASSERT_TRUE(error_200_hz) << error_200_hz.error();
    ASSERT_TRUE(error_400_hz) << error_400_hz.error();
    ASSERT_TRUE(error_800_hz) << error_800_hz.error();
    EXPECT_GT(*error_200_hz, 0.0);
    EXPECT_GT(*error_400_hz, 0.0);
    EXPECT_LT(*error_400_hz / *error_200_hz, 0.5 + kConvergenceSlack)
        << "errors: 200 Hz=" << *error_200_hz << ", 400 Hz=" << *error_400_hz;
    EXPECT_LT(*error_800_hz / *error_400_hz, 0.5 + kConvergenceSlack)
        << "errors: 400 Hz=" << *error_400_hz << ", 800 Hz=" << *error_800_hz;
}

TEST(SyntheticTest, DefaultLandmarksUseVariedDepths) {
    const auto landmarks = make_synthetic_landmarks();

    ASSERT_GE(landmarks.size(), 5);
    const auto [min_depth, max_depth] = std::ranges::minmax(
        landmarks | std::views::transform([](const SyntheticLandmark& landmark) {
            return landmark.position_world.z();
        }));
    EXPECT_GT(min_depth, 0.0);
    EXPECT_GT(max_depth / min_depth, 2.0);
}

TEST(SyntheticTest, ProjectsHandComputedStereoObservation) {
    const SyntheticTrajectory trajectory;
    const std::vector<SyntheticLandmark> landmarks{
        {.id = 42, .position_world = Eigen::Vector3d{0.0, 0.0, 5.0}},
    };
    const SyntheticCameraConfig config{
        .rate_hz = 20.0,
        .duration_seconds = 0.0,
    };

    const auto observations =
        synthesize_stereo_observations(trajectory, landmarks, make_cam0(), make_cam1(), config);

    ASSERT_TRUE(observations) << observations.error();
    ASSERT_EQ(observations->size(), 1);
    EXPECT_EQ(observations->front().timestamp, 0);
    EXPECT_EQ(observations->front().landmark_id, 42);
    EXPECT_LT((observations->front().cam0_pixel - Eigen::Vector2d{320.0, 240.0}).norm(), kTightTolerance);
    EXPECT_LT((observations->front().cam1_pixel - Eigen::Vector2d{300.0, 240.0}).norm(), kTightTolerance);
    EXPECT_NEAR(observations->front().disparity_pixels, 20.0, kTightTolerance);
}

TEST(SyntheticTest, RejectsInvisibleStereoLandmarks) {
    const SyntheticTrajectory trajectory;
    const SyntheticCameraConfig config{
        .rate_hz = 20.0,
        .duration_seconds = 0.0,
    };
    const std::vector<SyntheticLandmark> landmarks{
        {.id = 0, .position_world = Eigen::Vector3d{0.0, 0.0, -5.0}},
        {.id = 1, .position_world = Eigen::Vector3d{10.0, 0.0, 5.0}},
    };

    const auto observations =
        synthesize_stereo_observations(trajectory, landmarks, make_cam0(), make_cam1(), config);

    ASSERT_TRUE(observations) << observations.error();
    EXPECT_TRUE(observations->empty());
}

// Disparity is fx * baseline / depth, so the threshold drops landmarks past a
// configured depth. 500 * 0.2 / 300 is 0.33 px; 500 * 0.2 / 100 is 1.0 px.
TEST(SyntheticTest, RejectsStereoLandmarksBelowTheDisparityThreshold) {
    const SyntheticTrajectory trajectory;
    const SyntheticCameraConfig config{
        .rate_hz = 20.0,
        .duration_seconds = 0.0,
        .minimum_disparity_pixels = 0.5,
    };
    const std::vector<SyntheticLandmark> landmarks{
        {.id = 0, .position_world = Eigen::Vector3d{0.0, 0.0, 300.0}},
        {.id = 1, .position_world = Eigen::Vector3d{0.0, 0.0, 100.0}},
    };

    const auto observations =
        synthesize_stereo_observations(trajectory, landmarks, make_cam0(), make_cam1(), config);

    ASSERT_TRUE(observations) << observations.error();
    ASSERT_EQ(observations->size(), 1);
    EXPECT_EQ(observations->front().landmark_id, 1);
    EXPECT_NEAR(observations->front().disparity_pixels, 1.0, kTightTolerance);
}

// u0 - u1 is only a disparity on a rectified rig, so a rig that is not one has to
// be an error rather than silently producing zero observations.
TEST(SyntheticTest, RejectsNonRectifiedStereoRigs) {
    const SyntheticTrajectory trajectory;
    const auto landmarks = make_synthetic_landmarks();
    const SyntheticCameraConfig config{
        .rate_hz = 20.0,
        .duration_seconds = 0.0,
    };

    Eigen::Matrix4d rotated_cam1 = make_transform_with_translation(Eigen::Vector3d{0.2, 0.0, 0.0});
    rotated_cam1.block<3, 3>(0, 0) = Sophus::SO3d::exp(Eigen::Vector3d{0.0, 0.1, 0.0}).matrix();

    // Negative baseline, which used to produce negative disparity and empty output.
    EXPECT_FALSE(synthesize_stereo_observations(
        trajectory, landmarks, make_cam0(), make_cam1(Eigen::Vector3d{-0.2, 0.0, 0.0}), config));
    EXPECT_FALSE(synthesize_stereo_observations(
        trajectory, landmarks, make_cam0(), make_cam1(Eigen::Vector3d{0.0, 0.2, 0.0}), config));
    EXPECT_FALSE(synthesize_stereo_observations(
        trajectory,
        landmarks,
        make_cam0(),
        make_synthetic_pinhole_camera_calibration(rotated_cam1),
        config));
}

TEST(SyntheticTest, SeededImuNoiseIsReproducible) {
    const SyntheticTrajectory trajectory{
        .world_acceleration = Eigen::Vector3d{0.2, -0.1, 0.3},
        .body_angular_velocity = Eigen::Vector3d{0.1, -0.2, 0.3},
    };
    const SyntheticImuConfig noiseless_config{
        .rate_hz = 200.0,
        .duration_seconds = 0.02,
    };
    const SyntheticImuConfig noisy_config{
        .rate_hz = 200.0,
        .duration_seconds = 0.02,
        .noise = SyntheticImuNoiseConfig{
            .accelerometer_noise_density = 0.01,
            .gyroscope_noise_density = 0.002,
            .seed = 7,
        },
    };

    const auto noiseless_measurements = synthesize_imu(trajectory, noiseless_config);
    const auto noisy_measurements_a = synthesize_imu(trajectory, noisy_config);
    const auto noisy_measurements_b = synthesize_imu(trajectory, noisy_config);

    ASSERT_TRUE(noiseless_measurements) << noiseless_measurements.error();
    ASSERT_TRUE(noisy_measurements_a) << noisy_measurements_a.error();
    ASSERT_TRUE(noisy_measurements_b) << noisy_measurements_b.error();
    EXPECT_TRUE(imu_samples_match(*noisy_measurements_a, *noisy_measurements_b));
    EXPECT_FALSE(imu_samples_match(*noiseless_measurements, *noisy_measurements_a));
}

// The injected noise and the calibration handed to the filter must describe the same
// process, which is what makes covariance consistency (NEES) measurable later.
TEST(SyntheticTest, InjectedNoiseMatchesTheCalibrationDensities) {
    const SyntheticTrajectory trajectory;
    const SyntheticImuNoiseConfig noise{
        .accelerometer_noise_density = 0.01,
        .gyroscope_noise_density = 0.002,
        .seed = 3,
    };
    const SyntheticImuConfig noiseless_config{.rate_hz = 200.0, .duration_seconds = 100.0};
    const SyntheticImuConfig noisy_config{
        .rate_hz = 200.0,
        .duration_seconds = 100.0,
        .noise = noise,
    };

    const auto noiseless_measurements = synthesize_imu(trajectory, noiseless_config);
    const auto noisy_measurements = synthesize_imu(trajectory, noisy_config);
    const auto imu_calibration = make_synthetic_imu_calibration(noisy_config.rate_hz, noise);

    ASSERT_TRUE(noiseless_measurements) << noiseless_measurements.error();
    ASSERT_TRUE(noisy_measurements) << noisy_measurements.error();
    ASSERT_EQ(noiseless_measurements->size(), noisy_measurements->size());
    EXPECT_DOUBLE_EQ(imu_calibration.accelerometer_noise_density, noise.accelerometer_noise_density);
    EXPECT_DOUBLE_EQ(imu_calibration.gyroscope_noise_density, noise.gyroscope_noise_density);

    std::vector<double> accelerometer_residuals;
    std::vector<double> gyroscope_residuals;
    accelerometer_residuals.reserve(noisy_measurements->size());
    gyroscope_residuals.reserve(noisy_measurements->size());
    for (std::size_t i = 0; i < noisy_measurements->size(); ++i) {
        accelerometer_residuals.push_back(
            (*noisy_measurements)[i].acceleration.x() - (*noiseless_measurements)[i].acceleration.x());
        gyroscope_residuals.push_back(
            (*noisy_measurements)[i].angular_velocity.x() - (*noiseless_measurements)[i].angular_velocity.x());
    }

    // Discrete stddev on the sample grid is density * sqrt(rate).
    const double expected_accelerometer_stddev =
        imu_calibration.accelerometer_noise_density * std::sqrt(noisy_config.rate_hz);
    const double expected_gyroscope_stddev =
        imu_calibration.gyroscope_noise_density * std::sqrt(noisy_config.rate_hz);
    EXPECT_NEAR(
        sample_stddev(accelerometer_residuals),
        expected_accelerometer_stddev,
        kNoiseStddevTolerance * expected_accelerometer_stddev);
    EXPECT_NEAR(
        sample_stddev(gyroscope_residuals),
        expected_gyroscope_stddev,
        kNoiseStddevTolerance * expected_gyroscope_stddev);
}

TEST(SyntheticTest, SeededPixelNoiseIsReproducible) {
    const SyntheticTrajectory trajectory;
    const std::vector<SyntheticLandmark> landmarks{
        {.id = 42, .position_world = Eigen::Vector3d{0.0, 0.0, 5.0}},
    };
    const SyntheticCameraConfig noiseless_config{
        .rate_hz = 20.0,
        .duration_seconds = 0.0,
    };
    const SyntheticCameraConfig noisy_config{
        .rate_hz = 20.0,
        .duration_seconds = 0.0,
        .pixel_noise = SyntheticPixelNoiseConfig{
            .pixel_stddev = 0.1,
            .seed = 11,
        },
    };

    const auto noiseless_observations =
        synthesize_stereo_observations(trajectory, landmarks, make_cam0(), make_cam1(), noiseless_config);
    const auto noisy_observations_a =
        synthesize_stereo_observations(trajectory, landmarks, make_cam0(), make_cam1(), noisy_config);
    const auto noisy_observations_b =
        synthesize_stereo_observations(trajectory, landmarks, make_cam0(), make_cam1(), noisy_config);

    ASSERT_TRUE(noiseless_observations) << noiseless_observations.error();
    ASSERT_TRUE(noisy_observations_a) << noisy_observations_a.error();
    ASSERT_TRUE(noisy_observations_b) << noisy_observations_b.error();
    EXPECT_TRUE(stereo_observations_match(*noisy_observations_a, *noisy_observations_b));
    EXPECT_FALSE(stereo_observations_match(*noiseless_observations, *noisy_observations_a));
}
