#include "measurement_model.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace {

constexpr double kTolerance = 1e-12;

CameraCalibration make_camera(
    const Eigen::Matrix4d& t_bs = Eigen::Matrix4d::Identity(),
    const Eigen::Vector4d& intrinsics = Eigen::Vector4d{500.0, 600.0, 320.0, 240.0}) {
    return {
        .t_bs = t_bs,
        .rate_hz = 20.0,
        .resolution = Eigen::Vector2i{640, 480},
        .intrinsics = intrinsics,
        .distortion_coefficients = Eigen::Vector4d::Zero(),
    };
}

Eigen::Matrix4d make_transform(
    const Eigen::Vector3d& rotation_vector,
    const Eigen::Vector3d& translation) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.topLeftCorner<3, 3>() = Sophus::SO3d::exp(rotation_vector).matrix();
    transform.topRightCorner<3, 1>() = translation;
    return transform;
}

}  // namespace

TEST(MeasurementModelTest, OpticalAxisProjectsToPrincipalPoint) {
    const auto prediction = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{0.0, 0.0, 1.0},
        make_camera());

    ASSERT_TRUE(prediction) << prediction.error();
    EXPECT_TRUE(prediction->pixel.isApprox(Eigen::Vector2d{320.0, 240.0}, kTolerance));
}

TEST(MeasurementModelTest, ProjectionIsInvariantAlongTheSameRay) {
    const auto near_prediction = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{0.0, 0.0, 1.0},
        make_camera());
    const auto far_prediction = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{0.0, 0.0, 5.0},
        make_camera());

    ASSERT_TRUE(near_prediction) << near_prediction.error();
    ASSERT_TRUE(far_prediction) << far_prediction.error();
    EXPECT_TRUE(far_prediction->pixel.isApprox(near_prediction->pixel, kTolerance));
}

TEST(MeasurementModelTest, UnitHorizontalNormalizedOffsetScalesByFx) {
    const auto prediction = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{1.0, 0.0, 1.0},
        make_camera());

    ASSERT_TRUE(prediction) << prediction.error();
    EXPECT_TRUE(prediction->normalized.isApprox(Eigen::Vector2d{1.0, 0.0}, kTolerance));
    EXPECT_TRUE(prediction->pixel.isApprox(Eigen::Vector2d{820.0, 240.0}, kTolerance));
}

TEST(MeasurementModelTest, CameraExtrinsicTranslationChangesEachCameraPrediction) {
    const Eigen::Vector3d landmark_world{0.0, 0.0, 5.0};
    const auto cam0_prediction = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        landmark_world,
        make_camera());
    const auto cam1_prediction = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        landmark_world,
        make_camera(make_transform(Eigen::Vector3d::Zero(), Eigen::Vector3d{0.2, 0.0, 0.0})));

    ASSERT_TRUE(cam0_prediction) << cam0_prediction.error();
    ASSERT_TRUE(cam1_prediction) << cam1_prediction.error();
    EXPECT_TRUE(cam0_prediction->pixel.isApprox(Eigen::Vector2d{320.0, 240.0}, kTolerance));
    EXPECT_TRUE(cam1_prediction->pixel.isApprox(Eigen::Vector2d{300.0, 240.0}, kTolerance));
}

TEST(MeasurementModelTest, NontrivialRobotRotationUsesInversePoseBeforeProjection) {
    const Sophus::SO3d rotation_world_from_body =
        Sophus::SO3d::exp(Eigen::Vector3d{0.0, 0.0, std::numbers::pi / 2.0});
    const Eigen::Vector3d position_world_from_body{1.0, 2.0, 3.0};
    const Eigen::Vector3d landmark_world = position_world_from_body
        + rotation_world_from_body * Eigen::Vector3d{1.0, 0.0, 2.0};

    const auto prediction = predict_pinhole_pixel(
        rotation_world_from_body,
        position_world_from_body,
        landmark_world,
        make_camera());

    ASSERT_TRUE(prediction) << prediction.error();
    EXPECT_TRUE(prediction->landmark_body.isApprox(Eigen::Vector3d{1.0, 0.0, 2.0}, kTolerance));
    EXPECT_TRUE(prediction->pixel.isApprox(Eigen::Vector2d{570.0, 240.0}, kTolerance));
}

TEST(MeasurementModelTest, RejectsNonFinitePoseAndLandmarkInputs) {
    Eigen::Vector3d invalid_position = Eigen::Vector3d::Zero();
    invalid_position.x() = std::numeric_limits<double>::quiet_NaN();

    const auto invalid_pose = predict_pinhole_pixel(
        Sophus::SO3d{},
        invalid_position,
        Eigen::Vector3d{0.0, 0.0, 1.0},
        make_camera());
    EXPECT_FALSE(invalid_pose);
    EXPECT_NE(invalid_pose.error().find("p_wb"), std::string::npos);

    Eigen::Vector3d invalid_landmark{0.0, 0.0, 1.0};
    invalid_landmark.z() = std::numeric_limits<double>::infinity();
    const auto invalid_landmark_result = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        invalid_landmark,
        make_camera());
    EXPECT_FALSE(invalid_landmark_result);
    EXPECT_NE(invalid_landmark_result.error().find("landmark_world"), std::string::npos);
}

TEST(MeasurementModelTest, RejectsInvalidCameraCalibrationButDoesNotGateDepthOrImageBounds) {
    CameraCalibration invalid_camera = make_camera();
    invalid_camera.intrinsics.x() = -1.0;
    const auto invalid_calibration = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{0.0, 0.0, 1.0},
        invalid_camera);
    EXPECT_FALSE(invalid_calibration);
    EXPECT_NE(invalid_calibration.error().find("intrinsics"), std::string::npos);

    const auto behind_camera = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{1.0, 0.0, -1.0},
        make_camera());
    ASSERT_TRUE(behind_camera) << behind_camera.error();
    EXPECT_TRUE(behind_camera->pixel.isApprox(Eigen::Vector2d{-180.0, 240.0}, kTolerance));

    const auto outside_image = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{2.0, 0.0, 1.0},
        make_camera());
    ASSERT_TRUE(outside_image) << outside_image.error();
    EXPECT_TRUE(outside_image->pixel.isApprox(Eigen::Vector2d{1320.0, 240.0}, kTolerance));
}
