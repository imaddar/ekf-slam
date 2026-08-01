#include "propagation.hpp"

#include "parser.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <numbers>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

namespace {

constexpr double kTolerance = 1e-9;
constexpr int kPositionIndex = 0;
constexpr int kVelocityIndex = 3;
constexpr int kOrientationIndex = 6;
constexpr int kAccelerometerBiasIndex = 9;
constexpr int kGyroscopeBiasIndex = 12;
constexpr std::uint64_t kOneSecondNs = 1'000'000'000;

NominalState make_state() {
    return {
        .position = Eigen::Vector3d::Zero(),
        .velocity = Eigen::Vector3d::Zero(),
        .orientation = Sophus::SO3d{},
        .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = Eigen::Vector3d::Zero(),
    };
}

NominalState make_state_from_ground_truth(const GroundTruthState& ground_truth) {
    return {
        .position = ground_truth.position,
        .velocity = ground_truth.velocity,
        .orientation = Sophus::SO3d{ground_truth.orientation},
        .accelerometer_bias = ground_truth.accelerometer_bias,
        .gyroscope_bias = ground_truth.gyroscope_bias,
    };
}

ImuCalibration make_imu_calibration(
    double accelerometer_noise_density = 0.0,
    double gyroscope_noise_density = 0.0,
    double accelerometer_random_walk = 0.0,
    double gyroscope_random_walk = 0.0) {
    return {
        .t_bs = Eigen::Matrix4d::Identity(),
        .rate_hz = 200.0,
        .gyroscope_noise_density = gyroscope_noise_density,
        .gyroscope_random_walk = gyroscope_random_walk,
        .accelerometer_noise_density = accelerometer_noise_density,
        .accelerometer_random_walk = accelerometer_random_walk,
    };
}

const GroundTruthState& nearest_ground_truth(
    const std::vector<GroundTruthState>& ground_truth_states,
    TimestampNs timestamp) {
    const auto nearest = std::ranges::min_element(
        ground_truth_states,
        [timestamp](const GroundTruthState& lhs, const GroundTruthState& rhs) {
            const auto lhs_delta = lhs.timestamp > timestamp ? lhs.timestamp - timestamp : timestamp - lhs.timestamp;
            const auto rhs_delta = rhs.timestamp > timestamp ? rhs.timestamp - timestamp : timestamp - rhs.timestamp;
            return lhs_delta < rhs_delta;
        });

    return *nearest;
}

}  // namespace

TEST(PropagationTest, KeepsStationaryStateWhenSpecificForceBalancesGravity) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };
    const StateCovariance covariance = StateCovariance::Zero();

    const auto result = propagate(state, measurement, imu_calibration, 0.005, covariance);

    EXPECT_TRUE(result.nominal_state.position.isApprox(state.position, kTolerance));
    EXPECT_TRUE(result.nominal_state.velocity.isApprox(state.velocity, kTolerance));
    EXPECT_TRUE(result.nominal_state.orientation.unit_quaternion().isApprox(state.orientation.unit_quaternion(), kTolerance));
    EXPECT_TRUE(result.nominal_state.accelerometer_bias.isApprox(state.accelerometer_bias, kTolerance));
    EXPECT_TRUE(result.nominal_state.gyroscope_bias.isApprox(state.gyroscope_bias, kTolerance));
    EXPECT_TRUE(result.covariance.isApprox(covariance, kTolerance));
}

TEST(PropagationTest, IntegratesPositionAndVelocityFromWorldAcceleration) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{1.0, 2.0, 12.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };
    const StateCovariance covariance = StateCovariance::Identity();

    const auto result = propagate(state, measurement, imu_calibration, 0.1, covariance);

    EXPECT_TRUE(result.nominal_state.position.isApprox(Eigen::Vector3d{0.005, 0.01, 0.015}, kTolerance));
    EXPECT_TRUE(result.nominal_state.velocity.isApprox(Eigen::Vector3d{0.1, 0.2, 0.3}, kTolerance));
    EXPECT_TRUE(result.nominal_state.orientation.unit_quaternion().isApprox(state.orientation.unit_quaternion(), kTolerance));
}

TEST(PropagationTest, IntegratesOrientationFromAngularVelocity) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d{0.0, 0.0, std::numbers::pi},
    };
    const StateCovariance covariance = StateCovariance::Identity();

    const auto result = propagate(state, measurement, imu_calibration, 0.5, covariance);
    const Sophus::SO3d expected_orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.0, 0.0, std::numbers::pi / 2.0});

    EXPECT_TRUE(result.nominal_state.orientation.unit_quaternion().isApprox(expected_orientation.unit_quaternion(), kTolerance));
}

TEST(PropagationTest, RemovesAccelerometerAndGyroscopeBiasesBeforeIntegration) {
    NominalState state{
        .position = Eigen::Vector3d{1.0, 2.0, 3.0},
        .velocity = Eigen::Vector3d{4.0, 5.0, 6.0},
        .orientation = Sophus::SO3d{},
        .accelerometer_bias = Eigen::Vector3d{0.1, 0.2, 0.3},
        .gyroscope_bias = Eigen::Vector3d{0.4, 0.5, 0.6},
    };
    const auto imu_calibration = make_imu_calibration();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{1.1, 0.2, 10.11},
        .angular_velocity = Eigen::Vector3d{0.4, 0.5, 1.6},
    };
    const StateCovariance covariance = StateCovariance::Zero();

    const auto result = propagate(state, measurement, imu_calibration, 0.1, covariance);
    const Sophus::SO3d expected_orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.0, 0.0, 0.1});

    EXPECT_TRUE(result.nominal_state.position.isApprox(Eigen::Vector3d{1.405, 2.5, 3.6}, kTolerance));
    EXPECT_TRUE(result.nominal_state.velocity.isApprox(Eigen::Vector3d{4.1, 5.0, 6.0}, kTolerance));
    EXPECT_TRUE(result.nominal_state.orientation.unit_quaternion().isApprox(expected_orientation.unit_quaternion(), kTolerance));
    EXPECT_TRUE(result.nominal_state.accelerometer_bias.isApprox(state.accelerometer_bias, kTolerance));
    EXPECT_TRUE(result.nominal_state.gyroscope_bias.isApprox(state.gyroscope_bias, kTolerance));
    EXPECT_TRUE(result.covariance.isApprox(covariance, kTolerance));
}

TEST(PropagationTest, UpdatesCovarianceWithStateTransitionMatrix) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };
    const StateCovariance covariance = StateCovariance::Identity();

    const auto result = propagate(state, measurement, imu_calibration, 0.1, covariance);

    EXPECT_TRUE((result.covariance.block<3, 3>(kPositionIndex, kPositionIndex).isApprox(1.01 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kPositionIndex, kVelocityIndex).isApprox(0.1 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kVelocityIndex, kAccelerometerBiasIndex).isApprox(-0.1 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kOrientationIndex, kGyroscopeBiasIndex).isApprox(-0.1 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kVelocityIndex, kOrientationIndex).isApprox(
        (Eigen::Matrix3d{} << 0.0, 0.981, 0.0, -0.981, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
        kTolerance)));
    EXPECT_TRUE(result.covariance.isApprox(result.covariance.transpose(), kTolerance));
}

TEST(PropagationTest, AddsFirstOrderProcessNoiseFromImuCalibration) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration(
        2.0,
        3.0,
        4.0,
        5.0);
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };
    const StateCovariance covariance = StateCovariance::Zero();

    const auto result = propagate(state, measurement, imu_calibration, 0.1, covariance);

    EXPECT_TRUE((result.covariance.block<3, 3>(kPositionIndex, kPositionIndex).isZero(kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kVelocityIndex, kVelocityIndex).isApprox(0.4 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kOrientationIndex, kOrientationIndex).isApprox(0.9 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kAccelerometerBiasIndex, kAccelerometerBiasIndex).isApprox(1.6 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kGyroscopeBiasIndex, kGyroscopeBiasIndex).isApprox(2.5 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE(result.covariance.isApprox(result.covariance.transpose(), kTolerance));
}

TEST(PropagationTest, ProcessNoiseKeepsZeroInitialCovariancePositiveSemiDefinite) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration(
        0.02,
        0.03,
        0.004,
        0.005);
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{1.0, 2.0, 12.81},
        .angular_velocity = Eigen::Vector3d{0.1, 0.2, 0.3},
    };
    const StateCovariance covariance = StateCovariance::Zero();

    const auto result = propagate(state, measurement, imu_calibration, 0.005, covariance);
    const Eigen::SelfAdjointEigenSolver<StateCovariance> eigen_solver(result.covariance);

    EXPECT_TRUE(result.covariance.isApprox(result.covariance.transpose(), kTolerance));
    EXPECT_GE(eigen_solver.eigenvalues().minCoeff(), -kTolerance);
}

TEST(PropagationTest, SmokePropagatesShortEuRocWindowNearGroundTruth) {
    const auto sequence_root = std::filesystem::path(EKF_SLAM_SOURCE_DIR) / "datasets/machine_hall/MH_01_easy";
    ASSERT_TRUE(std::filesystem::exists(sequence_root))
        << "MH_01_easy dataset fixture is required for this smoke test: " << sequence_root;

    const auto dataset = parse_dataset(sequence_root);
    ASSERT_TRUE(dataset) << dataset.error();
    ASSERT_FALSE(dataset->imu_measurements.empty());
    ASSERT_FALSE(dataset->ground_truth_states.empty());

    const auto& initial_ground_truth = dataset->ground_truth_states.front();
    NominalState state = make_state_from_ground_truth(initial_ground_truth);
    StateCovariance covariance = 1e-6 * StateCovariance::Identity();
    TimestampNs previous_timestamp = initial_ground_truth.timestamp;
    const TimestampNs end_timestamp = initial_ground_truth.timestamp + kOneSecondNs;

    const auto first_imu = std::ranges::lower_bound(
        dataset->imu_measurements,
        initial_ground_truth.timestamp,
        {},
        &ImuMeasurement::timestamp);
    ASSERT_NE(first_imu, dataset->imu_measurements.end());

    TimestampNs propagated_timestamp = previous_timestamp;
    for (auto imu = first_imu; imu != dataset->imu_measurements.end() && imu->timestamp <= end_timestamp; ++imu) {
        ASSERT_GE(imu->timestamp, previous_timestamp);
        const double dt_seconds = static_cast<double>(imu->timestamp - previous_timestamp) * 1e-9;
        const auto result = propagate(state, *imu, dataset->imu_calibration, dt_seconds, covariance);
        state = result.nominal_state;
        covariance = result.covariance;
        previous_timestamp = imu->timestamp;
        propagated_timestamp = imu->timestamp;
    }

    const auto& final_ground_truth = nearest_ground_truth(dataset->ground_truth_states, propagated_timestamp);
    const double position_error_m = (state.position - final_ground_truth.position).norm();
    const double velocity_error_mps = (state.velocity - final_ground_truth.velocity).norm();
    const double orientation_error_rad =
        (Sophus::SO3d{final_ground_truth.orientation}.inverse() * state.orientation).log().norm();
    const Eigen::SelfAdjointEigenSolver<StateCovariance> eigen_solver(covariance);

    EXPECT_LT(position_error_m, 2.0);
    EXPECT_LT(velocity_error_mps, 3.0);
    EXPECT_LT(orientation_error_rad, 0.5);
    EXPECT_TRUE(covariance.allFinite());
    EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-8));
    EXPECT_GE(eigen_solver.eigenvalues().minCoeff(), -1e-8);
}
