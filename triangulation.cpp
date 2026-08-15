#include "triangulation.hpp"

#include "stereo_geometry.hpp"

#include <cmath>
#include <format>

#include <Eigen/Eigenvalues>

namespace {

constexpr double kRectificationTolerance = 1e-9;

ParseResult<void> validate_intrinsics(const CameraCalibration& camera, const char* name) {
    if (!camera.intrinsics.allFinite()) {
        return std::unexpected(std::format("{}.intrinsics: expected finite fx, fy, cx, cy", name));
    }
    if (camera.intrinsics.x() <= 0.0 || camera.intrinsics.y() <= 0.0) {
        return std::unexpected(std::format(
            "{}.intrinsics: expected positive fx and fy, found fx={} fy={}",
            name,
            camera.intrinsics.x(),
            camera.intrinsics.y()));
    }
    return {};
}

}  // namespace

ParseResult<StereoTriangulation> triangulate_stereo(
    const Eigen::Vector2d& pixel_cam0,
    const Eigen::Vector2d& pixel_cam1,
    const CameraCalibration& cam0,
    const CameraCalibration& cam1,
    const Eigen::Matrix4d& pixel_covariance,
    double minimum_disparity_pixels) {
    if (!pixel_cam0.allFinite() || !pixel_cam1.allFinite()) {
        return std::unexpected("stereo_pixels: expected finite [u0, v0, u1, v1]");
    }
    if (!pixel_covariance.allFinite()) {
        return std::unexpected("pixel_covariance: expected finite values");
    }
    if (!pixel_covariance.isApprox(pixel_covariance.transpose(), kRectificationTolerance)) {
        return std::unexpected("pixel_covariance: expected a symmetric matrix");
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> pixel_covariance_solver(pixel_covariance);
    if (pixel_covariance_solver.info() != Eigen::Success
        || pixel_covariance_solver.eigenvalues().minCoeff() < -kRectificationTolerance) {
        return std::unexpected("pixel_covariance: expected a positive semidefinite matrix");
    }
    if (minimum_disparity_pixels <= 0.0 || !std::isfinite(minimum_disparity_pixels)) {
        return std::unexpected("minimum_disparity_pixels: expected a positive finite value");
    }
    if (const auto valid = validate_intrinsics(cam0, "cam0"); !valid) {
        return std::unexpected(valid.error());
    }
    if (const auto valid = validate_intrinsics(cam1, "cam1"); !valid) {
        return std::unexpected(valid.error());
    }
    if (!cam0.intrinsics.isApprox(cam1.intrinsics, kRectificationTolerance)) {
        return std::unexpected(
            "stereo_rig: expected matching rectified pinhole intrinsics; "
            "raw EuRoC camera calibration must be rectified before triangulation");
    }

    const auto cam0_body = body_from_camera_transform(cam0, "cam0");
    if (!cam0_body) {
        return std::unexpected(cam0_body.error());
    }
    const auto cam1_body = body_from_camera_transform(cam1, "cam1");
    if (!cam1_body) {
        return std::unexpected(cam1_body.error());
    }

    const Eigen::Matrix3d rotation0 = cam0_body->block<3, 3>(0, 0);
    const Eigen::Matrix3d rotation1 = cam1_body->block<3, 3>(0, 0);
    if (!(rotation0.transpose() * rotation1).isApprox(Eigen::Matrix3d::Identity(), kRectificationTolerance)) {
        return std::unexpected("stereo_rig: expected cameras with a shared orientation");
    }

    const Eigen::Vector3d baseline_camera0 = rotation0.transpose()
        * (cam1_body->block<3, 1>(0, 3) - cam0_body->block<3, 1>(0, 3));
    if (std::abs(baseline_camera0.y()) > kRectificationTolerance
        || std::abs(baseline_camera0.z()) > kRectificationTolerance
        || baseline_camera0.x() <= 0.0) {
        return std::unexpected(std::format(
            "stereo_rig: expected a positive horizontal baseline in cam0, found [{}, {}, {}]",
            baseline_camera0.x(),
            baseline_camera0.y(),
            baseline_camera0.z()));
    }

    const double disparity = pixel_cam0.x() - pixel_cam1.x();
    if (disparity < minimum_disparity_pixels) {
        return std::unexpected(std::format(
            "disparity_pixels: expected at least {}, found {}",
            minimum_disparity_pixels,
            disparity));
    }

    const double fx = cam0.intrinsics.x();
    const double fy = cam0.intrinsics.y();
    const double cx = cam0.intrinsics.z();
    const double cy = cam0.intrinsics.w();
    const double baseline = baseline_camera0.x();
    const double depth = fx * baseline / disparity;
    const double x = (pixel_cam0.x() - cx) * baseline / disparity;
    const double y = (pixel_cam0.y() - cy) * fx * baseline / (fy * disparity);

    Eigen::Matrix<double, 3, 4> jacobian = Eigen::Matrix<double, 3, 4>::Zero();
    jacobian(0, 0) = baseline * (disparity - (pixel_cam0.x() - cx)) / (disparity * disparity);
    jacobian(0, 2) = baseline * (pixel_cam0.x() - cx) / (disparity * disparity);
    jacobian(1, 0) = -y / disparity;
    jacobian(1, 1) = fx * baseline / (fy * disparity);
    jacobian(1, 2) = y / disparity;
    jacobian(2, 0) = -depth / disparity;
    jacobian(2, 2) = depth / disparity;

    const Eigen::Matrix3d covariance = jacobian * pixel_covariance * jacobian.transpose();
    if (!covariance.allFinite()) {
        return std::unexpected("triangulation_covariance: expected finite values");
    }
    return StereoTriangulation{
        .point_camera = Eigen::Vector3d{x, y, depth},
        .covariance = (covariance + covariance.transpose()) * 0.5,
        .pixel_jacobian = jacobian,
        .disparity_pixels = disparity,
    };
}
