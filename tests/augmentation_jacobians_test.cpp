#include "augmentation_jacobians.hpp"

#include "stereo_geometry.hpp"

#include <limits>

#include <gtest/gtest.h>

namespace {

constexpr double kFiniteDifferenceStep = 1e-7;
constexpr double kTolerance = 1e-6;

CameraCalibration make_camera() {
    Eigen::Matrix4d t_bs = Eigen::Matrix4d::Identity();
    t_bs.topLeftCorner<3, 3>() = Sophus::SO3d::exp(Eigen::Vector3d{0.1, -0.2, 0.3}).matrix();
    t_bs.topRightCorner<3, 1>() = Eigen::Vector3d{0.4, -0.1, 0.2};
    return CameraCalibration{
        .t_bs = t_bs,
        .rate_hz = 20.0,
        .resolution = Eigen::Vector2i{640, 480},
        .intrinsics = Eigen::Vector4d{500.0, 500.0, 320.0, 240.0},
        .distortion_coefficients = Eigen::Vector4d::Zero(),
    };
}

NominalState make_robot() {
    return NominalState{
        .position = Eigen::Vector3d{1.0, -2.0, 3.0},
        .velocity = Eigen::Vector3d{0.2, -0.3, 0.4},
        .orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.2, -0.1, 0.3}),
        .accelerometer_bias = Eigen::Vector3d{0.01, -0.02, 0.03},
        .gyroscope_bias = Eigen::Vector3d{-0.04, 0.05, -0.06},
    };
}

Eigen::Vector3d evaluate(const NominalState& robot, const CameraCalibration& camera, const Eigen::Vector3d& point) {
    const auto result = camera_point_to_world(robot, camera, point);
    EXPECT_TRUE(result) << result.error();
    return result ? *result : Eigen::Vector3d::Constant(0.0);
}

}  // namespace

TEST(AugmentationJacobiansTest, MatchesFiniteDifferencesAndHasExpectedSparseStructure) {
    const NominalState robot = make_robot();
    const CameraCalibration camera = make_camera();
    const Eigen::Vector3d point_camera{1.5, -0.7, 4.0};
    const auto analytical = make_augmentation_jacobians(robot, camera, point_camera);

    ASSERT_TRUE(analytical) << analytical.error();
    Eigen::Matrix<double, 3, kImuErrorStateSize> numerical_robot =
        Eigen::Matrix<double, 3, kImuErrorStateSize>::Zero();
    for (int block = 0; block < 5; ++block) {
        for (int axis = 0; axis < 3; ++axis) {
            NominalState plus = robot;
            NominalState minus = robot;
            const int offset = block * 3 + axis;
            if (block == 0) {
                plus.position(axis) += kFiniteDifferenceStep;
                minus.position(axis) -= kFiniteDifferenceStep;
            } else if (block == 2) {
                plus.orientation = robot.orientation
                    * Sophus::SO3d::exp(Eigen::Vector3d::Unit(axis) * kFiniteDifferenceStep);
                minus.orientation = robot.orientation
                    * Sophus::SO3d::exp(-Eigen::Vector3d::Unit(axis) * kFiniteDifferenceStep);
            } else if (block == 1) {
                plus.velocity(axis) += kFiniteDifferenceStep;
                minus.velocity(axis) -= kFiniteDifferenceStep;
            } else if (block == 3) {
                plus.accelerometer_bias(axis) += kFiniteDifferenceStep;
                minus.accelerometer_bias(axis) -= kFiniteDifferenceStep;
            } else {
                plus.gyroscope_bias(axis) += kFiniteDifferenceStep;
                minus.gyroscope_bias(axis) -= kFiniteDifferenceStep;
            }
            numerical_robot.col(offset) =
                (evaluate(plus, camera, point_camera) - evaluate(minus, camera, point_camera))
                / (2.0 * kFiniteDifferenceStep);
        }
    }

    Eigen::Matrix3d numerical_camera;
    for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d plus = point_camera;
        Eigen::Vector3d minus = point_camera;
        plus(axis) += kFiniteDifferenceStep;
        minus(axis) -= kFiniteDifferenceStep;
        numerical_camera.col(axis) =
            (evaluate(robot, camera, plus) - evaluate(robot, camera, minus))
            / (2.0 * kFiniteDifferenceStep);
    }

    EXPECT_TRUE(analytical->robot.isApprox(numerical_robot, kTolerance));
    EXPECT_TRUE(analytical->camera.isApprox(numerical_camera, kTolerance));
    EXPECT_TRUE((analytical->robot.block<3, 3>(0, 0).isApprox(Eigen::Matrix3d::Identity(), 0.0)));
    EXPECT_TRUE((analytical->robot.block<3, 3>(0, 3).isZero(0.0)));
    EXPECT_TRUE((analytical->robot.block<3, 3>(0, 9).isZero(0.0)));
    EXPECT_TRUE((analytical->robot.block<3, 3>(0, 12).isZero(0.0)));
}

TEST(AugmentationJacobiansTest, RejectsInvalidInputs) {
    const CameraCalibration camera = make_camera();
    const Eigen::Vector3d point_camera{1.0, 2.0, 4.0};
    NominalState invalid_robot = make_robot();
    invalid_robot.position.x() = std::numeric_limits<double>::quiet_NaN();

    const auto invalid_pose = make_augmentation_jacobians(invalid_robot, camera, point_camera);
    EXPECT_FALSE(invalid_pose);

    Eigen::Matrix4d invalid_transform = camera.t_bs;
    invalid_transform(0, 0) = 2.0;
    CameraCalibration invalid_camera = camera;
    invalid_camera.t_bs = invalid_transform;
    const auto invalid_extrinsics = make_augmentation_jacobians(make_robot(), invalid_camera, point_camera);
    EXPECT_FALSE(invalid_extrinsics);
}
