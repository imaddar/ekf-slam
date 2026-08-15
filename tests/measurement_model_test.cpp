#include "measurement_model.hpp"

#include "stereo_geometry.hpp"
#include "synthetic.hpp"

#include <cmath>
#include <array>
#include <numbers>

#include <gtest/gtest.h>

namespace {

constexpr double kTolerance = 1e-12;
// h(.) and the harness apply the same transform in the same order, so the gap
// is float rounding only, not a modelling difference.
constexpr double kHarnessTolerance = 1e-9;

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

Eigen::Matrix<double, 2, 3> finite_difference_landmark(
    const Sophus::SO3d& rotation_world_from_body,
    const Eigen::Vector3d& position_world_from_body,
    const Eigen::Vector3d& landmark_world,
    const CameraCalibration& camera) {
    constexpr double kStep = 1e-6;
    Eigen::Matrix<double, 2, 3> result;
    const auto base = predict_pinhole_pixel(
        rotation_world_from_body, position_world_from_body, landmark_world, camera);
    EXPECT_TRUE(base) << base.error();
    for (int column = 0; column < 3; ++column) {
        Eigen::Vector3d plus_landmark = landmark_world;
        Eigen::Vector3d minus_landmark = landmark_world;
        plus_landmark(column) += kStep;
        minus_landmark(column) -= kStep;
        const auto plus = predict_pinhole_pixel(
            rotation_world_from_body, position_world_from_body, plus_landmark, camera);
        const auto minus = predict_pinhole_pixel(
            rotation_world_from_body, position_world_from_body, minus_landmark, camera);
        EXPECT_TRUE(plus) << plus.error();
        EXPECT_TRUE(minus) << minus.error();
        result.col(column) = (plus->pixel - minus->pixel) / (2.0 * kStep);
    }
    return result;
}

Eigen::Matrix<double, 2, 3> finite_difference_position(
    const Sophus::SO3d& rotation_world_from_body,
    const Eigen::Vector3d& position_world_from_body,
    const Eigen::Vector3d& landmark_world,
    const CameraCalibration& camera) {
    constexpr double kStep = 1e-6;
    Eigen::Matrix<double, 2, 3> result;
    const auto base = predict_pinhole_pixel(
        rotation_world_from_body, position_world_from_body, landmark_world, camera);
    EXPECT_TRUE(base) << base.error();
    for (int column = 0; column < 3; ++column) {
        Eigen::Vector3d plus_position = position_world_from_body;
        Eigen::Vector3d minus_position = position_world_from_body;
        plus_position(column) += kStep;
        minus_position(column) -= kStep;
        const auto plus = predict_pinhole_pixel(
            rotation_world_from_body, plus_position, landmark_world, camera);
        const auto minus = predict_pinhole_pixel(
            rotation_world_from_body, minus_position, landmark_world, camera);
        EXPECT_TRUE(plus) << plus.error();
        EXPECT_TRUE(minus) << minus.error();
        result.col(column) = (plus->pixel - minus->pixel) / (2.0 * kStep);
    }
    return result;
}

Eigen::Matrix<double, 2, 3> finite_difference_orientation(
    const Sophus::SO3d& rotation_world_from_body,
    const Eigen::Vector3d& position_world_from_body,
    const Eigen::Vector3d& landmark_world,
    const CameraCalibration& camera) {
    constexpr double kStep = 1e-6;
    Eigen::Matrix<double, 2, 3> result;
    const auto base = predict_pinhole_pixel(
        rotation_world_from_body, position_world_from_body, landmark_world, camera);
    EXPECT_TRUE(base) << base.error();
    for (int column = 0; column < 3; ++column) {
        Eigen::Vector3d perturbation = Eigen::Vector3d::Zero();
        perturbation(column) = kStep;
        const auto plus = predict_pinhole_pixel(
            rotation_world_from_body * Sophus::SO3d::exp(perturbation),
            position_world_from_body,
            landmark_world,
            camera);
        const auto minus = predict_pinhole_pixel(
            rotation_world_from_body * Sophus::SO3d::exp(-perturbation),
            position_world_from_body,
            landmark_world,
            camera);
        EXPECT_TRUE(plus) << plus.error();
        EXPECT_TRUE(minus) << minus.error();
        result.col(column) = (plus->pixel - minus->pixel) / (2.0 * kStep);
    }
    return result;
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

TEST(MeasurementModelTest, MeasurementJacobiansMatchWorldFrameFiniteDifferencesAndStateLayout) {
    struct JacobianCase {
        Sophus::SO3d rotation_world_from_body;
        Eigen::Vector3d position_world_from_body;
        Eigen::Vector3d landmark_world;
        CameraCalibration camera;
    };
    const std::array cases{
        JacobianCase{
            Sophus::SO3d::exp(Eigen::Vector3d{0.2, -0.1, 0.3}),
            Eigen::Vector3d{1.0, -2.0, 0.5},
            Eigen::Vector3d{3.0, 1.0, 6.0},
            make_camera(make_transform(
                Eigen::Vector3d{0.1, -0.2, 0.05}, Eigen::Vector3d{0.2, -0.1, 0.3})),
        },
        JacobianCase{Sophus::SO3d{}, Eigen::Vector3d::Zero(), Eigen::Vector3d{2.0, -3.0, 3.0}, make_camera()},
        JacobianCase{
            Sophus::SO3d::exp(Eigen::Vector3d{-0.3, 0.2, -0.1}),
            Eigen::Vector3d{-1.0, 0.5, 0.2},
            Eigen::Vector3d{0.0, 3.0, 7.0},
            make_camera(),
        },
        JacobianCase{
            Sophus::SO3d::exp(Eigen::Vector3d{0.1, 0.2, -0.2}),
            Eigen::Vector3d{0.5, -1.0, 2.0},
            Eigen::Vector3d{4.0, 0.0, 8.0},
            make_camera(make_transform(Eigen::Vector3d::Zero(), Eigen::Vector3d{0.3, 0.0, 0.0})),
        },
        JacobianCase{
            Sophus::SO3d::exp(Eigen::Vector3d{0.05, -0.15, 0.25}),
            Eigen::Vector3d{2.0, 1.0, -0.5},
            Eigen::Vector3d{3.5, -2.0, 5.0},
            make_camera(make_transform(
                Eigen::Vector3d{-0.1, 0.05, 0.2}, Eigen::Vector3d{-0.1, 0.2, 0.1})),
        },
    };

    for (const auto& test_case : cases) {
        const auto prediction = predict_pinhole_pixel(
            test_case.rotation_world_from_body,
            test_case.position_world_from_body,
            test_case.landmark_world,
            test_case.camera);
        ASSERT_TRUE(prediction) << prediction.error();
        ASSERT_NE(prediction->landmark_camera.z(), 1.0);

        const auto jacobians = make_measurement_jacobian_blocks(
            *prediction,
            test_case.rotation_world_from_body,
            test_case.camera,
            kImuErrorStateSize + 3 * 3);
        ASSERT_TRUE(jacobians) << jacobians.error();
        EXPECT_EQ(jacobians->landmark_offset, kImuErrorStateSize + 3 * 3);

        const double x = prediction->landmark_camera.x();
        const double y = prediction->landmark_camera.y();
        const double z = prediction->landmark_camera.z();
        Eigen::Matrix<double, 2, 3> projection = Eigen::Matrix<double, 2, 3>::Zero();
        projection(0, 0) = test_case.camera.intrinsics.x() / z;
        projection(0, 2) = -test_case.camera.intrinsics.x() * x / (z * z);
        projection(1, 1) = test_case.camera.intrinsics.y() / z;
        projection(1, 2) = -test_case.camera.intrinsics.y() * y / (z * z);
        const auto camera_from_body = camera_from_body_transform(test_case.camera);
        ASSERT_TRUE(camera_from_body) << camera_from_body.error();
        const Eigen::Matrix<double, 2, 3> expected_position = projection
            * camera_from_body->block<3, 3>(0, 0)
            * -test_case.rotation_world_from_body.inverse().matrix();
        EXPECT_TRUE(jacobians->pose.leftCols<3>().isApprox(expected_position, kTolerance));
        EXPECT_LT((jacobians->landmark - finite_difference_landmark(
            test_case.rotation_world_from_body,
            test_case.position_world_from_body,
            test_case.landmark_world,
            test_case.camera)).norm(), 2e-4);
        EXPECT_LT((jacobians->pose.leftCols<3>() - finite_difference_position(
            test_case.rotation_world_from_body,
            test_case.position_world_from_body,
            test_case.landmark_world,
            test_case.camera)).norm(), 2e-4);
        EXPECT_LT((jacobians->pose.rightCols<3>() - finite_difference_orientation(
            test_case.rotation_world_from_body,
            test_case.position_world_from_body,
            test_case.landmark_world,
            test_case.camera)).norm(), 2e-4);
    }
}

TEST(MeasurementModelTest, MeasurementJacobianIdentityCaseAndInvalidInputs) {
    const CameraCalibration camera = make_camera();
    const Eigen::Vector3d landmark_world{2.0, -3.0, 3.0};
    const auto prediction = predict_pinhole_pixel(
        Sophus::SO3d{}, Eigen::Vector3d::Zero(), landmark_world, camera);
    ASSERT_TRUE(prediction) << prediction.error();
    const auto jacobians = make_measurement_jacobian_blocks(
        *prediction, Sophus::SO3d{}, camera, kImuErrorStateSize);
    ASSERT_TRUE(jacobians) << jacobians.error();

    Eigen::Matrix<double, 2, 3> expected_projection;
    expected_projection << 500.0 / 3.0, 0.0, -1000.0 / 9.0,
        0.0, 200.0, 200.0;
    EXPECT_TRUE(jacobians->landmark.isApprox(expected_projection, kTolerance));
    EXPECT_TRUE(jacobians->pose.leftCols<3>().isApprox(-expected_projection, kTolerance));

    const auto invalid_offset = make_measurement_jacobian_blocks(
        *prediction, Sophus::SO3d{}, camera, kImuErrorStateSize + 1);
    EXPECT_FALSE(invalid_offset);
    EXPECT_NE(invalid_offset.error().find("landmark_offset"), std::string::npos);
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

// fx=500, cx=320, width=640 puts the right image edge at normalized x = 0.64;
// fy=600, cy=240, height=480 puts the bottom edge at normalized y = 0.4.
TEST(MeasurementModelTest, LandmarksOnTheImageBorderProjectToTheBoundaryPixels) {
    const auto bottom_right = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{0.64, 0.4, 1.0},
        make_camera());
    const auto top_left = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{-0.64, -0.4, 1.0},
        make_camera());

    ASSERT_TRUE(bottom_right) << bottom_right.error();
    ASSERT_TRUE(top_left) << top_left.error();
    // Absolute, not isApprox: the relative form never matches a zero expectation.
    EXPECT_LT((bottom_right->pixel - Eigen::Vector2d{640.0, 480.0}).norm(), kTolerance);
    EXPECT_LT(top_left->pixel.norm(), kTolerance);
}

// Pins the cost of leaving visibility gating out of h(.): allFinite() catches
// only numerical degeneracy, not behind-camera geometry.
TEST(MeasurementModelTest, NonPositiveDepthReturnsSuccessButRequiresExplicitVisibilityGating) {
    const auto zero_depth = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{1.0, 0.0, 0.0},
        make_camera());
    const auto negative_depth = predict_pinhole_pixel(
        Sophus::SO3d{},
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{6.0, 0.0, -3.0},
        make_camera());

    ASSERT_TRUE(zero_depth) << zero_depth.error();
    ASSERT_TRUE(negative_depth) << negative_depth.error();
    EXPECT_DOUBLE_EQ(zero_depth->landmark_camera.z(), 0.0);
    EXPECT_FALSE(zero_depth->pixel.allFinite());
    EXPECT_DOUBLE_EQ(negative_depth->landmark_camera.z(), -3.0);
    EXPECT_TRUE(negative_depth->pixel.allFinite());
    EXPECT_TRUE(negative_depth->normalized.isApprox(Eigen::Vector2d{-2.0, -0.0}, kTolerance));
}

// h(.) and the synthetic harness project independently; without this check the
// two could drift apart and every synthetic update test would still pass.
TEST(MeasurementModelTest, ReproducesNoiselessSyntheticHarnessPixels) {
    const SyntheticTrajectory trajectory{
        .initial_position = Eigen::Vector3d{0.3, -0.2, 0.0},
        .initial_velocity = Eigen::Vector3d{0.5, 0.0, 0.0},
        .world_acceleration = Eigen::Vector3d{0.0, 0.2, 0.0},
        .body_angular_velocity = Eigen::Vector3d{0.0, 0.0, 0.15},
    };
    const auto landmarks = make_synthetic_landmarks();
    const CameraCalibration cam0 =
        make_synthetic_pinhole_camera_calibration(Eigen::Matrix4d::Identity());
    const CameraCalibration cam1 =
        make_synthetic_pinhole_camera_calibration(make_transform(
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d{0.2, 0.0, 0.0}));
    const SyntheticCameraConfig config{.rate_hz = 20.0, .duration_seconds = 1.0};

    const auto observations =
        synthesize_stereo_observations(trajectory, landmarks, cam0, cam1, config);

    ASSERT_TRUE(observations) << observations.error();
    ASSERT_FALSE(observations->empty());

    for (const auto& observation : *observations) {
        const auto landmark = std::ranges::find(
            landmarks, observation.landmark_id, &SyntheticLandmark::id);
        ASSERT_NE(landmark, landmarks.end());

        const auto truth = trajectory.state_at(trajectory.time_at(observation.timestamp));
        const Sophus::SO3d rotation_world_from_body{truth.orientation};

        const auto cam0_prediction = predict_pinhole_pixel(
            rotation_world_from_body, truth.position, landmark->position_world, cam0);
        const auto cam1_prediction = predict_pinhole_pixel(
            rotation_world_from_body, truth.position, landmark->position_world, cam1);

        ASSERT_TRUE(cam0_prediction) << cam0_prediction.error();
        ASSERT_TRUE(cam1_prediction) << cam1_prediction.error();
        EXPECT_LT((cam0_prediction->pixel - observation.cam0_pixel).norm(), kHarnessTolerance)
            << "landmark " << observation.landmark_id << " at t=" << observation.timestamp;
        EXPECT_LT((cam1_prediction->pixel - observation.cam1_pixel).norm(), kHarnessTolerance)
            << "landmark " << observation.landmark_id << " at t=" << observation.timestamp;
    }
}
