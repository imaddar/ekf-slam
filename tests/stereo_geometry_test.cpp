#include "stereo_geometry.hpp"

#include <limits>
#include <string>

#include <gtest/gtest.h>

namespace {

CameraCalibration make_camera(const Eigen::Matrix4d& t_bs) {
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
        .velocity = Eigen::Vector3d::Zero(),
        .orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.2, -0.3, 0.4}),
        .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = Eigen::Vector3d::Zero(),
    };
}

Eigen::Matrix4d make_transform(const Eigen::Vector3d& rotation_vector, const Eigen::Vector3d& translation) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.topLeftCorner<3, 3>() = Sophus::SO3d::exp(rotation_vector).matrix();
    transform.topRightCorner<3, 1>() = translation;
    return transform;
}

}  // namespace

TEST(StereoGeometryTest, IdentityTransformMapsCameraPointDirectlyToWorld) {
    NominalState robot{
        .position = Eigen::Vector3d::Zero(),
        .velocity = Eigen::Vector3d::Zero(),
        .orientation = Sophus::SO3d{},
        .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = Eigen::Vector3d::Zero(),
    };
    const CameraCalibration camera = make_camera(Eigen::Matrix4d::Identity());
    const Eigen::Vector3d point_camera{1.0, 2.0, 5.0};

    const auto result = camera_point_to_world(robot, camera, point_camera);

    ASSERT_TRUE(result) << result.error();
    EXPECT_TRUE(result->isApprox(point_camera, 0.0));
}

TEST(StereoGeometryTest, WorldToCameraToWorldRoundTripMatchesToMachinePrecision) {
    const NominalState robot = make_robot();
    const CameraCalibration camera = make_camera(
        make_transform(Eigen::Vector3d{-0.1, 0.25, 0.15}, Eigen::Vector3d{0.3, -0.2, 0.4}));
    const Eigen::Vector3d point_world{4.0, -1.0, 8.0};

    const auto camera_point = world_point_to_camera(robot, camera, point_world);
    ASSERT_TRUE(camera_point) << camera_point.error();
    const auto world_again = camera_point_to_world(robot, camera, *camera_point);

    ASSERT_TRUE(world_again) << world_again.error();
    EXPECT_TRUE(world_again->isApprox(point_world, 1e-15));
}

TEST(StereoGeometryTest, NonzeroPoseAndExtrinsicsMatchHandComputedForwardMap) {
    NominalState robot{
        .position = Eigen::Vector3d{1.0, 2.0, 3.0},
        .velocity = Eigen::Vector3d::Zero(),
        .orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.0, 0.0, 1.5707963267948966}),
        .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = Eigen::Vector3d::Zero(),
    };
    const Eigen::Matrix4d t_bs = make_transform(Eigen::Vector3d::Zero(), Eigen::Vector3d{0.1, 0.0, 0.0});
    const CameraCalibration camera = make_camera(t_bs);
    const Eigen::Vector3d point_camera{1.0, 0.0, 5.0};

    const auto result = camera_point_to_world(robot, camera, point_camera);

    ASSERT_TRUE(result) << result.error();
    EXPECT_TRUE(result->isApprox(Eigen::Vector3d{1.0, 3.1, 8.0}, 1e-15));
}

TEST(StereoGeometryTest, RejectsNonRigidCameraExtrinsics) {
    Eigen::Matrix4d invalid = Eigen::Matrix4d::Identity();
    invalid(0, 0) = 2.0;
    const auto result = camera_point_to_world(make_robot(), make_camera(invalid), Eigen::Vector3d{1.0, 2.0, 3.0});

    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("t_bs"), std::string::npos);
}

TEST(StereoGeometryTest, RejectsReflectionExtrinsics) {
    Eigen::Matrix4d reflection = Eigen::Matrix4d::Identity();
    reflection(0, 0) = -1.0;
    const auto result = world_point_to_camera(make_robot(), make_camera(reflection), Eigen::Vector3d{1.0, 2.0, 3.0});

    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("proper rotation"), std::string::npos);
}

TEST(StereoGeometryTest, RejectsNonFiniteWorldPoint) {
    const Eigen::Vector3d invalid{
        std::numeric_limits<double>::quiet_NaN(), 2.0, 3.0};
    const auto result = world_point_to_camera(make_robot(), make_camera(Eigen::Matrix4d::Identity()), invalid);

    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("point_world"), std::string::npos);
}
