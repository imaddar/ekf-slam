#include "slam_state.hpp"

#include <climits>
#include <cstddef>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace {

ImuStateCovariance make_initial_covariance() {
    ImuStateCovariance covariance = ImuStateCovariance::Zero();
    covariance.diagonal() = Eigen::Vector<double, kRobotDim>::LinSpaced(kRobotDim, 1.0, 15.0);
    return covariance;
}

}

TEST(SlamStateTest, RequiresExplicitRobotCovarianceInitialization) {
    const ImuStateCovariance expected = make_initial_covariance();
    const auto result = SlamState::create(0, expected);

    ASSERT_TRUE(result) << result.error();
    EXPECT_TRUE(result->robot_covariance().isApprox(expected, 0.0));
    EXPECT_TRUE(result->active_covariance().isApprox(expected, 0.0));
}

TEST(SlamStateTest, PreallocatesFullCovarianceForLandmarkBudget) {
    constexpr std::size_t kMaxLandmarks = 4;
    const auto result = SlamState::create(kMaxLandmarks, make_initial_covariance());

    ASSERT_TRUE(result) << result.error();
    EXPECT_EQ(result->max_landmarks(), kMaxLandmarks);
    EXPECT_EQ(result->storage_dim(), kRobotDim + 4 * kLandmarkDim);
    EXPECT_EQ(result->active_landmarks(), 0U);
    EXPECT_EQ(result->active_dim(), kRobotDim);
    EXPECT_EQ(result->active_covariance().rows(), kRobotDim);
    EXPECT_EQ(result->active_covariance().cols(), kRobotDim);
}

TEST(SlamStateTest, ActivatingStorageDoesNotReallocate) {
    const auto result = SlamState::create(5, make_initial_covariance());

    ASSERT_TRUE(result) << result.error();
    SlamState state = std::move(*result);
    const auto* allocation = state.robot_covariance().data();
    const auto* active_view_data = state.active_covariance().data();

    EXPECT_EQ(allocation, active_view_data);
    EXPECT_EQ(state.robot_covariance().data(), allocation);
    EXPECT_EQ(state.active_covariance().data(), allocation);
}

TEST(SlamStateTest, RejectsLandmarkCapacityThatCannotFitInEigenDimension) {
    const auto result = SlamState::create(static_cast<std::size_t>(INT_MAX), make_initial_covariance());

    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("max_landmarks"), std::string::npos) << result.error();
}

TEST(SlamStateTest, RobotCovarianceBlockRoundTripsThroughAccessor) {
    const auto result = SlamState::create(3, make_initial_covariance());

    ASSERT_TRUE(result) << result.error();
    SlamState state = std::move(*result);
    const Eigen::Matrix<double, kRobotDim, kRobotDim> expected =
        Eigen::Matrix<double, kRobotDim, kRobotDim>::Identity();

    state.robot_covariance() = expected;

    EXPECT_TRUE(state.robot_covariance().isApprox(expected, 0.0));
    EXPECT_TRUE(state.active_covariance().isApprox(expected, 0.0));
}

TEST(SlamStateTest, DoesNotExposeResizableCovarianceMember) {
    const auto result = SlamState::create(1, make_initial_covariance());

    ASSERT_TRUE(result) << result.error();
    SlamState state = std::move(*result);

    EXPECT_EQ(state.storage_dim(), kRobotDim + kLandmarkDim);
}
