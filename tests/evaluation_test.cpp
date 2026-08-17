#include "evaluation.hpp"

#include <gtest/gtest.h>

namespace {

GroundTruthState truth(TimestampNs timestamp, double x) {
    return GroundTruthState{.timestamp = timestamp, .position = {x, 0.0, 0.0}, .orientation = Eigen::Quaterniond::Identity(),
        .velocity = {1.0, 0.0, 0.0}, .gyroscope_bias = Eigen::Vector3d::Zero(), .accelerometer_bias = Eigen::Vector3d::Zero()};
}

TrajectoryEstimate estimate(TimestampNs timestamp, double x) {
    return TrajectoryEstimate{.timestamp = timestamp, .state = NominalState{.position = {x, 0.0, 0.0},
        .velocity = {1.0, 0.0, 0.0}, .orientation = Sophus::SO3d{}, .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = Eigen::Vector3d::Zero()}, .covariance = ImuStateCovariance::Identity()};
}

}  // namespace

TEST(EvaluationTest, InterpolatesGroundTruthAtCameraTime) {
    const std::vector<GroundTruthState> truths{truth(0, 0.0), truth(2'000'000'000, 2.0)};
    const auto interpolated = interpolate_ground_truth(truths, 500'000'000);
    ASSERT_TRUE(interpolated) << interpolated.error();
    EXPECT_DOUBLE_EQ(interpolated->position.x(), 0.5);
    EXPECT_DOUBLE_EQ(interpolated->velocity.x(), 1.0);
}

TEST(EvaluationTest, ReportsRawAteRpeAndRobotNees) {
    const std::vector<GroundTruthState> truths{truth(0, 0.0), truth(1'000'000'000, 1.0), truth(2'000'000'000, 2.0)};
    const std::vector<TrajectoryEstimate> estimates{estimate(0, 1.0), estimate(1'000'000'000, 2.0), estimate(2'000'000'000, 3.0)};
    const auto metrics = evaluate_trajectory(estimates, truths);
    ASSERT_TRUE(metrics) << metrics.error();
    EXPECT_DOUBLE_EQ(metrics->ate_position_rmse_m, 1.0);
    EXPECT_DOUBLE_EQ(metrics->rpe_translation_rmse_m_per_s, 0.0);
    EXPECT_DOUBLE_EQ(metrics->rpe_rotation_rmse_rad_per_s, 0.0);
    EXPECT_DOUBLE_EQ(metrics->mean_robot_nees, 1.0);
    EXPECT_EQ(metrics->rpe_pair_count, 2U);
}
