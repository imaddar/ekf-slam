#include "triangulation.hpp"

#include "synthetic.hpp"

#include <cmath>
#include <random>
#include <string>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

namespace {

CameraCalibration make_camera(const Eigen::Vector3d& translation) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.topRightCorner<3, 1>() = translation;
    return make_synthetic_pinhole_camera_calibration(
        transform);
}

Eigen::Vector4d pack_pixels(const Eigen::Vector2d& cam0, const Eigen::Vector2d& cam1) {
    return Eigen::Vector4d{cam0.x(), cam0.y(), cam1.x(), cam1.y()};
}

Eigen::Vector2d project(const Eigen::Vector3d& point, const CameraCalibration& camera) {
    const Eigen::Matrix3d rotation_body_from_camera = camera.t_bs.block<3, 3>(0, 0);
    const Eigen::Vector3d translation_body_from_camera = camera.t_bs.block<3, 1>(0, 3);
    const Eigen::Vector3d point_camera = rotation_body_from_camera.transpose()
        * (point - translation_body_from_camera);
    return Eigen::Vector2d{
        camera.intrinsics.x() * point_camera.x() / point_camera.z() + camera.intrinsics.z(),
        camera.intrinsics.y() * point_camera.y() / point_camera.z() + camera.intrinsics.w(),
    };
}

}  // namespace

TEST(TriangulationTest, RectifiedRigRecoversMetricPointAndAnisotropicCovariance) {
    const CameraCalibration cam0 = make_camera(Eigen::Vector3d::Zero());
    const CameraCalibration cam1 = make_camera(Eigen::Vector3d{0.2, 0.0, 0.0});
    const Eigen::Vector3d point{0.5, -0.25, 5.0};
    const Eigen::Vector2d pixel0 = project(point, cam0);
    const Eigen::Vector2d pixel1 = project(point, cam1);
    const Eigen::Matrix4d pixel_covariance = 0.01 * Eigen::Matrix4d::Identity();

    const auto result = triangulate_stereo(pixel0, pixel1, cam0, cam1, pixel_covariance);

    ASSERT_TRUE(result) << result.error();
    EXPECT_TRUE(result->point_camera.isApprox(point, 1e-12));
    EXPECT_TRUE(result->covariance.isApprox(result->covariance.transpose(), 1e-15));
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(result->covariance);
    ASSERT_EQ(solver.info(), Eigen::Success);
    EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-14);
    const Eigen::Vector3d ray = point.normalized();
    const Eigen::Vector3d principal = solver.eigenvectors().col(2);
    EXPECT_GT(std::abs(principal.dot(ray)), 0.99);
}

TEST(TriangulationTest, DepthAndLateralStandardDeviationsScaleWithRange) {
    const CameraCalibration cam0 = make_camera(Eigen::Vector3d::Zero());
    const CameraCalibration cam1 = make_camera(Eigen::Vector3d{0.2, 0.0, 0.0});
    const Eigen::Matrix4d pixel_covariance = 0.01 * Eigen::Matrix4d::Identity();
    const auto near = triangulate_stereo(
        project(Eigen::Vector3d{0.0, 0.0, 5.0}, cam0),
        project(Eigen::Vector3d{0.0, 0.0, 5.0}, cam1),
        cam0, cam1, pixel_covariance);
    const auto far = triangulate_stereo(
        project(Eigen::Vector3d{0.0, 0.0, 10.0}, cam0),
        project(Eigen::Vector3d{0.0, 0.0, 10.0}, cam1),
        cam0, cam1, pixel_covariance);

    ASSERT_TRUE(near) << near.error();
    ASSERT_TRUE(far) << far.error();
    EXPECT_NEAR(std::sqrt(far->covariance(2, 2) / near->covariance(2, 2)), 4.0, 1e-12);
    EXPECT_NEAR(std::sqrt(far->covariance(0, 0) / near->covariance(0, 0)), 2.0, 1e-12);
    EXPECT_NEAR(std::sqrt(far->covariance(1, 1) / near->covariance(1, 1)), 2.0, 1e-12);
}

TEST(TriangulationTest, MonteCarloPixelNoiseMatchesAnalyticalCovariance) {
    const CameraCalibration cam0 = make_camera(Eigen::Vector3d::Zero());
    const CameraCalibration cam1 = make_camera(Eigen::Vector3d{0.2, 0.0, 0.0});
    const Eigen::Vector3d point{0.5, -0.25, 5.0};
    const Eigen::Vector2d pixel0 = project(point, cam0);
    const Eigen::Vector2d pixel1 = project(point, cam1);
    const Eigen::Matrix4d pixel_covariance = 0.01 * Eigen::Matrix4d::Identity();
    const auto nominal = triangulate_stereo(pixel0, pixel1, cam0, cam1, pixel_covariance);
    ASSERT_TRUE(nominal) << nominal.error();

    std::mt19937 generator(42);
    std::normal_distribution<double> noise(0.0, 0.1);
    constexpr int kSamples = 100000;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d second_moment = Eigen::Matrix3d::Zero();
    for (int sample = 0; sample < kSamples; ++sample) {
        Eigen::Vector4d pixels = pack_pixels(pixel0, pixel1);
        for (int index = 0; index < 4; ++index) {
            pixels(index) += noise(generator);
        }
        const auto estimate = triangulate_stereo(
            pixels.head<2>(), pixels.tail<2>(), cam0, cam1, pixel_covariance);
        ASSERT_TRUE(estimate) << estimate.error();
        mean += estimate->point_camera;
        second_moment += estimate->point_camera * estimate->point_camera.transpose();
    }
    mean /= static_cast<double>(kSamples);
    const Eigen::Matrix3d empirical = second_moment / static_cast<double>(kSamples)
        - mean * mean.transpose();
    EXPECT_TRUE(empirical.isApprox(nominal->covariance, 0.03));
}

TEST(TriangulationTest, RejectsDegenerateDisparityAndNonRectifiedRigs) {
    const CameraCalibration cam0 = make_camera(Eigen::Vector3d::Zero());
    const CameraCalibration cam1 = make_camera(Eigen::Vector3d{0.2, 0.0, 0.0});
    const Eigen::Matrix4d pixel_covariance = 0.01 * Eigen::Matrix4d::Identity();
    const Eigen::Vector2d pixel{320.0, 240.0};

    const auto degenerate = triangulate_stereo(
        pixel, pixel, cam0, cam1, pixel_covariance, 0.1);
    EXPECT_FALSE(degenerate);
    EXPECT_NE(degenerate.error().find("disparity"), std::string::npos);

    CameraCalibration rotated = cam1;
    rotated.t_bs.topLeftCorner<3, 3>() = Sophus::SO3d::exp(Eigen::Vector3d{0.0, 0.1, 0.0}).matrix();
    const auto non_rectified = triangulate_stereo(
        Eigen::Vector2d{340.0, 240.0}, Eigen::Vector2d{320.0, 240.0},
        cam0, rotated, pixel_covariance);
    EXPECT_FALSE(non_rectified);
    EXPECT_NE(non_rectified.error().find("shared orientation"), std::string::npos);
}
