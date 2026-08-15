#include "stereo_geometry.hpp"

#include <cmath>
#include <format>

namespace {

ParseResult<void> validate_camera_extrinsics(const CameraCalibration& camera) {
    if (!camera.t_bs.allFinite()) {
        return std::unexpected("camera.t_bs: expected finite values");
    }

    const Eigen::Matrix3d rotation = camera.t_bs.block<3, 3>(0, 0);
    if (!(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-9)) {
        return std::unexpected("camera.t_bs: expected an orthonormal rotation block");
    }
    if (!std::isfinite(rotation.determinant()) || rotation.determinant() <= 0.0) {
        return std::unexpected("camera.t_bs: expected a proper rotation");
    }
    if (!camera.t_bs.row(3).isApprox(Eigen::Vector4d{0.0, 0.0, 0.0, 1.0}.transpose(), 1e-9)) {
        return std::unexpected("camera.t_bs: expected a homogeneous rigid transform");
    }
    return {};
}

}  // namespace

ParseResult<Eigen::Vector3d> camera_point_to_world(
    const NominalState& robot,
    const CameraCalibration& camera,
    const Eigen::Vector3d& point_camera) {
    if (!robot.position.allFinite() || !robot.orientation.unit_quaternion().coeffs().allFinite()) {
        return std::unexpected("robot_pose: expected finite position and orientation");
    }
    if (!point_camera.allFinite()) {
        return std::unexpected("point_camera: expected finite XYZ coordinates");
    }
    if (const auto valid = validate_camera_extrinsics(camera); !valid) {
        return std::unexpected(valid.error());
    }

    const Eigen::Matrix3d rotation_body_from_camera = camera.t_bs.block<3, 3>(0, 0);
    const Eigen::Vector3d translation_body_from_camera = camera.t_bs.block<3, 1>(0, 3);
    return robot.orientation.matrix()
        * (rotation_body_from_camera * point_camera + translation_body_from_camera)
        + robot.position;
}

ParseResult<Eigen::Vector3d> world_point_to_camera(
    const NominalState& robot,
    const CameraCalibration& camera,
    const Eigen::Vector3d& point_world) {
    if (!point_world.allFinite()) {
        return std::unexpected("point_world: expected finite XYZ coordinates");
    }
    if (const auto valid = validate_camera_extrinsics(camera); !valid) {
        return std::unexpected(valid.error());
    }
    if (!robot.position.allFinite() || !robot.orientation.unit_quaternion().coeffs().allFinite()) {
        return std::unexpected("robot_pose: expected finite position and orientation");
    }

    const Eigen::Matrix3d rotation_body_from_camera = camera.t_bs.block<3, 3>(0, 0);
    const Eigen::Vector3d translation_body_from_camera = camera.t_bs.block<3, 1>(0, 3);
    const Eigen::Vector3d point_body = robot.orientation.inverse() * (point_world - robot.position);
    return rotation_body_from_camera.transpose() * (point_body - translation_body_from_camera);
}
