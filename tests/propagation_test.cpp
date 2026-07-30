#include "propagation.hpp"

#include <numbers>

#include <gtest/gtest.h>

namespace {

constexpr double kTolerance = 1e-9;
constexpr int kPositionIndex = 0;
constexpr int kVelocityIndex = 3;
constexpr int kOrientationIndex = 6;
constexpr int kAccelerometerBiasIndex = 9;
constexpr int kGyroscopeBiasIndex = 12;

NominalState make_state() {
    return {
        .position = Eigen::Vector3d::Zero(),
        .velocity = Eigen::Vector3d::Zero(),
        .orientation = Sophus::SO3d{},
        .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = Eigen::Vector3d::Zero(),
    };
}

}  // namespace

TEST(PropagationTest, KeepsStationaryStateWhenSpecificForceBalancesGravity) {
    const auto state = make_state();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };
    const StateCovariance covariance = StateCovariance::Zero();

    const auto result = propagate(state, measurement, 0.005, covariance);

    EXPECT_TRUE(result.nominal_state.position.isApprox(state.position, kTolerance));
    EXPECT_TRUE(result.nominal_state.velocity.isApprox(state.velocity, kTolerance));
    EXPECT_TRUE(result.nominal_state.orientation.unit_quaternion().isApprox(state.orientation.unit_quaternion(), kTolerance));
    EXPECT_TRUE(result.nominal_state.accelerometer_bias.isApprox(state.accelerometer_bias, kTolerance));
    EXPECT_TRUE(result.nominal_state.gyroscope_bias.isApprox(state.gyroscope_bias, kTolerance));
    EXPECT_TRUE(result.covariance.isApprox(covariance, kTolerance));
}

TEST(PropagationTest, IntegratesPositionAndVelocityFromWorldAcceleration) {
    const auto state = make_state();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{1.0, 2.0, 12.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };
    const StateCovariance covariance = StateCovariance::Identity();

    const auto result = propagate(state, measurement, 0.1, covariance);

    EXPECT_TRUE(result.nominal_state.position.isApprox(Eigen::Vector3d{0.005, 0.01, 0.015}, kTolerance));
    EXPECT_TRUE(result.nominal_state.velocity.isApprox(Eigen::Vector3d{0.1, 0.2, 0.3}, kTolerance));
    EXPECT_TRUE(result.nominal_state.orientation.unit_quaternion().isApprox(state.orientation.unit_quaternion(), kTolerance));
}

TEST(PropagationTest, IntegratesOrientationFromAngularVelocity) {
    const auto state = make_state();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d{0.0, 0.0, std::numbers::pi},
    };
    const StateCovariance covariance = StateCovariance::Identity();

    const auto result = propagate(state, measurement, 0.5, covariance);
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
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{1.1, 0.2, 10.11},
        .angular_velocity = Eigen::Vector3d{0.4, 0.5, 1.6},
    };
    const StateCovariance covariance = StateCovariance::Zero();

    const auto result = propagate(state, measurement, 0.1, covariance);
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
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };
    const StateCovariance covariance = StateCovariance::Identity();

    const auto result = propagate(state, measurement, 0.1, covariance);

    EXPECT_TRUE((result.covariance.block<3, 3>(kPositionIndex, kPositionIndex).isApprox(1.01 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kPositionIndex, kVelocityIndex).isApprox(0.1 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kVelocityIndex, kAccelerometerBiasIndex).isApprox(-0.1 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kOrientationIndex, kGyroscopeBiasIndex).isApprox(-0.1 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kVelocityIndex, kOrientationIndex).isApprox(
        (Eigen::Matrix3d{} << 0.0, 0.981, 0.0, -0.981, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
        kTolerance)));
    EXPECT_TRUE(result.covariance.isApprox(result.covariance.transpose(), kTolerance));
}
