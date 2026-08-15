#include "measurement_model.hpp"

#include <cmath>
#include <format>

namespace {

constexpr double kRigidTransformTolerance = 1e-9;

ParseResult<void> validate_camera_model(const CameraCalibration& camera) {
    if (!camera.t_bs.allFinite()) {
        return std::unexpected("camera.t_bs: expected finite values");
    }

    const Eigen::Matrix3d rotation = camera.t_bs.block<3, 3>(0, 0);
    if (!(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), kRigidTransformTolerance)
        || std::abs(rotation.determinant() - 1.0) > kRigidTransformTolerance) {
        return std::unexpected("camera.t_bs: expected a rotation block in SO(3)");
    }

    const Eigen::Vector4d bottom_row = camera.t_bs.row(3);
    if ((bottom_row - Eigen::Vector4d{0.0, 0.0, 0.0, 1.0}).norm() > kRigidTransformTolerance) {
        return std::unexpected(std::format(
            "camera.t_bs: expected bottom row [0, 0, 0, 1], found [{}, {}, {}, {}]",
            bottom_row.x(),
            bottom_row.y(),
            bottom_row.z(),
            bottom_row.w()));
    }

    if (!camera.intrinsics.allFinite()) {
        return std::unexpected("camera.intrinsics: expected finite fx, fy, cx, cy");
    }
    if (camera.intrinsics.x() <= 0.0 || camera.intrinsics.y() <= 0.0) {
        return std::unexpected(std::format(
            "camera.intrinsics: expected positive fx and fy, found fx={} fy={}",
            camera.intrinsics.x(),
            camera.intrinsics.y()));
    }

    return {};
}

}  // namespace

ParseResult<PinholePixelPrediction> predict_pinhole_pixel(
    const Sophus::SO3d& rotation_world_from_body,
    const Eigen::Vector3d& position_world_from_body,
    const Eigen::Vector3d& landmark_world,
    const CameraCalibration& camera) {
    if (!rotation_world_from_body.unit_quaternion().coeffs().allFinite()) {
        return std::unexpected("r_wb: expected finite rotation coefficients");
    }
    if (!position_world_from_body.allFinite()) {
        return std::unexpected("p_wb: expected finite XYZ coordinates");
    }
    if (!landmark_world.allFinite()) {
        return std::unexpected("landmark_world: expected finite XYZ coordinates");
    }
    if (const auto valid = validate_camera_model(camera); !valid) {
        return std::unexpected(valid.error());
    }

    const Eigen::Matrix3d rotation_body_from_camera = camera.t_bs.block<3, 3>(0, 0);
    const Eigen::Vector3d position_body_from_camera = camera.t_bs.block<3, 1>(0, 3);

    const Eigen::Vector3d landmark_body =
        rotation_world_from_body.inverse() * (landmark_world - position_world_from_body);
    const Eigen::Vector3d landmark_camera =
        rotation_body_from_camera.transpose() * (landmark_body - position_body_from_camera);
    const Eigen::Vector2d normalized{
        landmark_camera.x() / landmark_camera.z(),
        landmark_camera.y() / landmark_camera.z(),
    };

    const Eigen::Vector4d intrinsics = camera.intrinsics;
    return PinholePixelPrediction{
        .landmark_body = landmark_body,
        .landmark_camera = landmark_camera,
        .normalized = normalized,
        .pixel = Eigen::Vector2d{
            intrinsics.x() * normalized.x() + intrinsics.z(),
            intrinsics.y() * normalized.y() + intrinsics.w(),
        },
    };
}
