#include "landmark_augmentation.hpp"

#include "augmentation_jacobians.hpp"
#include "propagation.hpp"
#include "stereo_geometry.hpp"
#include "synthetic.hpp"
#include "triangulation.hpp"

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

namespace {

CameraCalibration make_camera(const Eigen::Vector3d& translation) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.topRightCorner<3, 1>() = translation;
    return make_synthetic_pinhole_camera_calibration(transform);
}

Eigen::Vector2d project_body_point(const Eigen::Vector3d& point_body, const CameraCalibration& camera) {
    const Eigen::Matrix3d rotation = camera.t_bs.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = camera.t_bs.block<3, 1>(0, 3);
    const Eigen::Vector3d point_camera = rotation.transpose() * (point_body - translation);
    return Eigen::Vector2d{
        camera.intrinsics.x() * point_camera.x() / point_camera.z() + camera.intrinsics.z(),
        camera.intrinsics.y() * point_camera.y() / point_camera.z() + camera.intrinsics.w(),
    };
}

NominalState make_robot() {
    return NominalState{
        .position = Eigen::Vector3d{1.0, -2.0, 0.5},
        .velocity = Eigen::Vector3d::Zero(),
        .orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.1, -0.2, 0.3}),
        .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = Eigen::Vector3d::Zero(),
    };
}

ImuStateCovariance make_dense_covariance() {
    ImuStateCovariance factor = ImuStateCovariance::Identity();
    factor.block<3, 3>(3, 0).setConstant(0.05);
    factor.block<3, 3>(9, 0).setConstant(0.02);
    factor.block<3, 3>(12, 0).setConstant(0.03);
    return factor * factor.transpose();
}

}  // namespace

TEST(LandmarkAugmentationTest, InsertsWorldLandmarkAndExpectedCovariance) {
    const CameraCalibration cam0 = make_camera(Eigen::Vector3d::Zero());
    const CameraCalibration cam1 = make_camera(Eigen::Vector3d{0.2, 0.0, 0.0});
    const NominalState robot = make_robot();
    const ImuStateCovariance robot_covariance = make_dense_covariance();
    auto state_result = SlamState::create(2, robot, robot_covariance);
    ASSERT_TRUE(state_result) << state_result.error();
    SlamState state = std::move(*state_result);

    const Eigen::Vector3d point_body{0.5, -0.25, 5.0};
    const Eigen::Vector2d pixel0 = project_body_point(point_body, cam0);
    const Eigen::Vector2d pixel1 = project_body_point(point_body, cam1);
    const Eigen::Matrix4d pixel_covariance = 0.01 * Eigen::Matrix4d::Identity();
    const auto result = augment_landmark(
        state, 7, pixel0, pixel1, cam0, cam1, pixel_covariance);

    ASSERT_TRUE(result) << result.error();
    ASSERT_EQ(state.active_landmarks(), 1U);
    const auto position = state.landmark_position(7);
    ASSERT_TRUE(position) << position.error();
    const auto expected_world = camera_point_to_world(
        robot, cam0, Eigen::Vector3d{0.5, -0.25, 5.0});
    ASSERT_TRUE(expected_world) << expected_world.error();
    EXPECT_TRUE(position->isApprox(*expected_world, 1e-12));

    EXPECT_TRUE(state.robot_landmark_covariance().bottomRows<3>().allFinite());
    EXPECT_GT(state.robot_landmark_covariance().block(3, 0, 3, 3).norm(), 0.0);
    EXPECT_GT(state.robot_landmark_covariance().block(9, 0, 6, 3).norm(), 0.0);
    const auto triangulation = triangulate_stereo(
        pixel0, pixel1, cam0, cam1, pixel_covariance);
    ASSERT_TRUE(triangulation) << triangulation.error();
    const auto jacobians = make_augmentation_jacobians(
        robot, cam0, triangulation->point_camera);
    ASSERT_TRUE(jacobians) << jacobians.error();
    const Eigen::Matrix<double, kRobotDim, 3> expected_cross =
        robot_covariance * jacobians->robot.transpose();
    const Eigen::Matrix3d expected_landmark_covariance =
        jacobians->robot * robot_covariance * jacobians->robot.transpose()
        + jacobians->camera * triangulation->covariance * jacobians->camera.transpose();
    EXPECT_TRUE(state.robot_landmark_covariance().isApprox(expected_cross, 1e-12));
    const auto inserted_block = state.landmark_block(0);
    ASSERT_TRUE(inserted_block) << inserted_block.error();
    EXPECT_TRUE(inserted_block->isApprox(expected_landmark_covariance, 1e-12));
    EXPECT_TRUE(state.active_covariance().isApprox(state.active_covariance().transpose(), 1e-12));
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(state.active_covariance());
    ASSERT_EQ(solver.info(), Eigen::Success);
    EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-10);
}

TEST(LandmarkAugmentationTest, ZeroRobotUncertaintyLeavesOnlyMeasurementCovariance) {
    const CameraCalibration cam0 = make_camera(Eigen::Vector3d::Zero());
    const CameraCalibration cam1 = make_camera(Eigen::Vector3d{0.2, 0.0, 0.0});
    auto state_result = SlamState::create(1, make_robot(), ImuStateCovariance::Zero());
    ASSERT_TRUE(state_result) << state_result.error();
    SlamState state = std::move(*state_result);
    const Eigen::Vector3d point_body{0.0, 0.0, 5.0};
    const Eigen::Matrix4d pixel_covariance = 0.01 * Eigen::Matrix4d::Identity();

    ASSERT_TRUE(augment_landmark(
        state,
        1,
        project_body_point(point_body, cam0),
        project_body_point(point_body, cam1),
        cam0,
        cam1,
        pixel_covariance));

    EXPECT_TRUE(state.robot_landmark_covariance().isApprox(Eigen::MatrixXd::Zero(15, 3), 0.0));
    EXPECT_GT(state.landmark_block(0)->trace(), 0.0);
}

TEST(LandmarkAugmentationTest, ExistingLandmarkCorrelationIsInheritedByNewLandmark) {
    const CameraCalibration cam0 = make_camera(Eigen::Vector3d::Zero());
    const CameraCalibration cam1 = make_camera(Eigen::Vector3d{0.2, 0.0, 0.0});
    auto state_result = SlamState::create(2, make_robot(), make_dense_covariance());
    ASSERT_TRUE(state_result) << state_result.error();
    SlamState state = std::move(*state_result);
    const Eigen::Matrix4d pixel_covariance = 0.01 * Eigen::Matrix4d::Identity();
    const Eigen::Vector3d first{0.5, 0.0, 5.0};
    const Eigen::Vector3d second{-0.5, 0.25, 6.0};

    ASSERT_TRUE(augment_landmark(
        state, 10, project_body_point(first, cam0), project_body_point(first, cam1),
        cam0, cam1, pixel_covariance));
    ASSERT_TRUE(propagate_slam(
        state,
        ImuMeasurement{
            .timestamp = 0,
            .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
            .angular_velocity = Eigen::Vector3d{0.01, -0.02, 0.03},
        },
        make_synthetic_imu_calibration(200.0),
        0.005));
    ASSERT_TRUE(augment_landmark(
        state, 11, project_body_point(second, cam0), project_body_point(second, cam1),
        cam0, cam1, pixel_covariance));

    const auto first_offset = state.landmark_offset(10);
    const auto second_offset = state.landmark_offset(11);
    ASSERT_TRUE(first_offset) << first_offset.error();
    ASSERT_TRUE(second_offset) << second_offset.error();
    const Eigen::Matrix3d cross = state.active_covariance().block<3, 3>(*first_offset, *second_offset);
    EXPECT_GT(cross.norm(), 0.0);
    EXPECT_TRUE(state.active_covariance().isApprox(state.active_covariance().transpose(), 1e-12));
}
