#include "slam_state.hpp"

#include <climits>
#include <cstddef>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <gtest/gtest.h>

struct SlamStateTestAccess {
    static const Eigen::MatrixXd& covariance(const SlamState& state) {
        return state.covariance_;
    }
};

namespace {

NominalState make_initial_robot() {
    return NominalState{
        .position = Eigen::Vector3d{1.0, -2.0, 3.0},
        .velocity = Eigen::Vector3d{-4.0, 5.0, -6.0},
        .orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.1, -0.2, 0.3}),
        .accelerometer_bias = Eigen::Vector3d{0.01, -0.02, 0.03},
        .gyroscope_bias = Eigen::Vector3d{-0.04, 0.05, -0.06},
    };
}

ImuStateCovariance make_initial_covariance() {
    ImuStateCovariance covariance = ImuStateCovariance::Zero();
    covariance.diagonal() = Eigen::Vector<double, kRobotDim>::LinSpaced(kRobotDim, 1.0, 15.0);
    return covariance;
}

auto make_state(std::size_t max_landmarks) {
    return SlamState::create(max_landmarks, make_initial_robot(), make_initial_covariance());
}

}

TEST(SlamStateTest, PreservesExplicitRobotInitialization) {
    const NominalState expected_robot = make_initial_robot();
    const ImuStateCovariance expected_covariance = make_initial_covariance();
    const auto result = SlamState::create(0, expected_robot, expected_covariance);

    ASSERT_TRUE(result) << result.error();
    EXPECT_TRUE(result->robot.position.isApprox(expected_robot.position, 0.0));
    EXPECT_TRUE(result->robot.velocity.isApprox(expected_robot.velocity, 0.0));
    EXPECT_TRUE(result->robot.orientation.matrix().isApprox(expected_robot.orientation.matrix(), 0.0));
    EXPECT_TRUE(result->robot.accelerometer_bias.isApprox(expected_robot.accelerometer_bias, 0.0));
    EXPECT_TRUE(result->robot.gyroscope_bias.isApprox(expected_robot.gyroscope_bias, 0.0));
    EXPECT_TRUE(result->robot_covariance().isApprox(expected_covariance, 0.0));
    EXPECT_TRUE(result->active_covariance().isApprox(expected_covariance, 0.0));
}

TEST(SlamStateTest, PreallocatesFullCovarianceForLandmarkBudget) {
    constexpr std::size_t kMaxLandmarks = 4;
    const auto result = make_state(kMaxLandmarks);

    ASSERT_TRUE(result) << result.error();
    EXPECT_EQ(result->max_landmarks(), kMaxLandmarks);
    EXPECT_EQ(result->storage_dim(), kRobotDim + 4 * kLandmarkDim);
    EXPECT_EQ(result->active_landmarks(), 0U);
    EXPECT_EQ(result->active_dim(), kRobotDim);
    EXPECT_EQ(result->active_covariance().rows(), kRobotDim);
    EXPECT_EQ(result->active_covariance().cols(), kRobotDim);
}

TEST(SlamStateTest, RejectsLandmarkCapacityThatCannotFitInEigenDimension) {
    const auto result = make_state(static_cast<std::size_t>(INT_MAX));

    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("max_landmarks"), std::string::npos) << result.error();
}

TEST(SlamStateTest, RobotCovarianceBlockRoundTripsThroughAccessor) {
    const auto result = make_state(3);

    ASSERT_TRUE(result) << result.error();
    SlamState state = std::move(*result);
    const Eigen::Matrix<double, kRobotDim, kRobotDim> expected =
        Eigen::Matrix<double, kRobotDim, kRobotDim>::Identity();

    state.robot_covariance() = expected;

    EXPECT_TRUE(state.robot_covariance().isApprox(expected, 0.0));
    EXPECT_TRUE(state.active_covariance().isApprox(expected, 0.0));
}

TEST(SlamStateTest, PoisonsInactiveCovarianceRegion) {
    const auto result = make_state(2);

    ASSERT_TRUE(result) << result.error();
    const auto& covariance = SlamStateTestAccess::covariance(*result);
    const int storage_dim = covariance.rows();
    const int inactive_dim = storage_dim - kRobotDim;

    ASSERT_GT(inactive_dim, 0);
    EXPECT_TRUE(covariance.block(kRobotDim, 0, inactive_dim, storage_dim).array().isNaN().all());
    EXPECT_TRUE(covariance.block(0, kRobotDim, kRobotDim, inactive_dim).array().isNaN().all());
}
