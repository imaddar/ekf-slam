#include "landmark_augmentation.hpp"
#include "propagation.hpp"

#include "synthetic.hpp"

#include <array>
#include <chrono>
#include <iostream>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

namespace {

ImuStateCovariance make_initial_covariance() {
    return 1e-4 * ImuStateCovariance::Identity();
}

}  // namespace

TEST(SlamIntegrationTest, ExcitedPropagationAugmentationAndCompactionRemainValid) {
    const SyntheticTrajectory trajectory{
        .initial_position = Eigen::Vector3d{0.1, -0.2, 0.0},
        .initial_velocity = Eigen::Vector3d{0.3, -0.1, 0.0},
        .world_acceleration = Eigen::Vector3d{0.2, 0.1, 0.0},
        .sinusoidal_world_acceleration = Eigen::Vector3d{0.1, -0.15, 0.05},
        .sinusoidal_frequency_rad_per_sec = 1.3,
        .body_angular_velocity = Eigen::Vector3d{0.03, -0.02, 0.12},
    };
    const auto imu = synthesize_imu(trajectory, SyntheticImuConfig{
        .rate_hz = 200.0,
        .duration_seconds = 1.0,
    });
    ASSERT_TRUE(imu) << imu.error();
    const auto landmarks = make_synthetic_landmarks();
    const auto observations = synthesize_stereo_observations(
        trajectory,
        landmarks,
        make_synthetic_pinhole_camera_calibration(Eigen::Matrix4d::Identity()),
        make_synthetic_pinhole_camera_calibration([] {
            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
            transform(0, 3) = 0.2;
            return transform;
        }()),
        SyntheticCameraConfig{.rate_hz = 20.0, .duration_seconds = 1.0});
    ASSERT_TRUE(observations) << observations.error();

    const CameraCalibration cam0 =
        make_synthetic_pinhole_camera_calibration(Eigen::Matrix4d::Identity());
    Eigen::Matrix4d cam1_transform = Eigen::Matrix4d::Identity();
    cam1_transform(0, 3) = 0.2;
    const CameraCalibration cam1 = make_synthetic_pinhole_camera_calibration(cam1_transform);
    const GroundTruthState initial_truth = trajectory.state_at(0.0);
    auto state_result = SlamState::create(
        landmarks.size(),
        NominalState{
            .position = initial_truth.position,
            .velocity = initial_truth.velocity,
            .orientation = Sophus::SO3d{initial_truth.orientation},
            .accelerometer_bias = initial_truth.accelerometer_bias,
            .gyroscope_bias = initial_truth.gyroscope_bias,
        },
        make_initial_covariance());
    ASSERT_TRUE(state_result) << state_result.error();
    SlamState state = std::move(*state_result);
    const auto* allocation = state.active_covariance().data();
    const Eigen::Matrix4d pixel_covariance = 0.01 * Eigen::Matrix4d::Identity();

    const auto augmentation_start = std::chrono::steady_clock::now();
    std::size_t augmented = 0;
    for (const auto& observation : *observations) {
        if (observation.timestamp != trajectory.start_timestamp) {
            continue;
        }
        ASSERT_TRUE(augment_landmark(
            state,
            observation.landmark_id,
            observation.cam0_pixel,
            observation.cam1_pixel,
            cam0,
            cam1,
            pixel_covariance)) << observation.landmark_id;
        ++augmented;
    }
    ASSERT_EQ(augmented, landmarks.size());
    const auto augmentation_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - augmentation_start);
    std::cout << "augmentation_ns_per_landmark="
              << static_cast<double>(augmentation_elapsed.count()) / static_cast<double>(augmented)
              << '\n';

    for (const auto& measurement : *imu) {
        ASSERT_TRUE(propagate_slam(
            state,
            measurement,
            make_synthetic_imu_calibration(200.0),
            1.0 / 200.0));
        EXPECT_EQ(state.active_covariance().data(), allocation);
        EXPECT_TRUE(state.active_covariance().allFinite());
        EXPECT_TRUE(state.active_covariance().isApprox(state.active_covariance().transpose(), 1e-9));
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(state.active_covariance());
        ASSERT_EQ(solver.info(), Eigen::Success);
        EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-8);
    }

    const std::array<LandmarkId, 2> removed{1, 3};
    ASSERT_TRUE(state.remove_landmarks(removed));
    EXPECT_EQ(state.active_landmarks(), landmarks.size() - removed.size());
    EXPECT_TRUE(state.active_covariance().allFinite());
    EXPECT_EQ(state.active_covariance().data(), allocation);
}
