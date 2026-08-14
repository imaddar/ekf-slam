#include "slam_state.hpp"

#include <cstddef>

#include <Eigen/Core>
#include <gtest/gtest.h>

TEST(SlamStateTest, AllocatesRobotOnlyCovarianceWhenLandmarkBudgetIsZero) {
    SlamState state{0};

    EXPECT_EQ(state.max_landmarks(), 0U);
    EXPECT_EQ(state.active_landmarks(), 0U);
    EXPECT_EQ(state.active_dim(), kRobotDim);
    EXPECT_EQ(state.covariance.rows(), kRobotDim);
    EXPECT_EQ(state.covariance.cols(), kRobotDim);
    EXPECT_EQ(state.active_covariance().rows(), kRobotDim);
    EXPECT_EQ(state.active_covariance().cols(), kRobotDim);
}

TEST(SlamStateTest, PreallocatesFullCovarianceForLandmarkBudget) {
    constexpr std::size_t kMaxLandmarks = 4;
    SlamState state{kMaxLandmarks};

    constexpr int kExpectedDim = kRobotDim + static_cast<int>(kMaxLandmarks) * kLandmarkDim;
    EXPECT_EQ(state.max_landmarks(), kMaxLandmarks);
    EXPECT_EQ(state.covariance.rows(), kExpectedDim);
    EXPECT_EQ(state.covariance.cols(), kExpectedDim);
    EXPECT_EQ(state.active_dim(), kRobotDim);
    EXPECT_EQ(state.active_covariance().rows(), kRobotDim);
    EXPECT_EQ(state.active_covariance().cols(), kRobotDim);
}

TEST(SlamStateTest, TracksActiveDimensionSeparatelyFromAllocatedCapacity) {
    SlamState state{5};

    const auto activated = state.set_active_landmarks(3);

    ASSERT_TRUE(activated) << activated.error();
    EXPECT_EQ(state.active_landmarks(), 3U);
    EXPECT_EQ(state.active_dim(), kRobotDim + 3 * kLandmarkDim);
    EXPECT_EQ(state.covariance.rows(), kRobotDim + 5 * kLandmarkDim);
    EXPECT_EQ(state.active_covariance().rows(), state.active_dim());
    EXPECT_EQ(state.active_covariance().cols(), state.active_dim());
}

TEST(SlamStateTest, RejectsActiveLandmarkCountBeyondCapacity) {
    SlamState state{2};

    const auto activated = state.set_active_landmarks(3);

    ASSERT_FALSE(activated);
    EXPECT_NE(activated.error().find("active_landmarks"), std::string::npos) << activated.error();
    EXPECT_EQ(state.active_landmarks(), 0U);
    EXPECT_EQ(state.active_dim(), kRobotDim);
}

TEST(SlamStateTest, FixedSizeLandmarkBlocksRoundTripAtNonzeroOffset) {
    SlamState state{3};
    const auto activated = state.set_active_landmarks(3);
    ASSERT_TRUE(activated) << activated.error();

    const int second_landmark_offset = state.landmark_offset(1);
    const Eigen::Matrix3d expected =
        (Eigen::Matrix3d{} << 1.0, 2.0, 3.0,
            4.0, 5.0, 6.0,
            7.0, 8.0, 9.0)
            .finished();

    state.covariance.block<kLandmarkDim, kLandmarkDim>(second_landmark_offset, second_landmark_offset) = expected;

    EXPECT_TRUE((state.covariance.block<kLandmarkDim, kLandmarkDim>(
                                second_landmark_offset,
                                second_landmark_offset)
                     .isApprox(expected)));
}
