#include "stereo_geometry.hpp"

#include <format>
#include <string_view>

namespace {

constexpr double kRigidTransformTolerance = 1e-9;

ParseResult<Eigen::Matrix4d> validate_camera_extrinsics(
    const CameraCalibration& camera,
    std::string_view field_name) {
    if (!camera.t_bs.allFinite()) {
        return std::unexpected(std::format("{}.t_bs: expected finite values", field_name));
    }

    const Eigen::Matrix3d rotation = camera.t_bs.block<3, 3>(0, 0);
    const Eigen::Matrix3d gram = rotation.transpose() * rotation;
    if (!gram.isApprox(Eigen::Matrix3d::Identity(), kRigidTransformTolerance)) {
        return std::unexpected(std::format(
            "{}.t_bs: expected an orthonormal rotation block, found R^T R off identity by {}",
            field_name,
            (gram - Eigen::Matrix3d::Identity()).norm()));
    }
    const double determinant = rotation.determinant();
    if (determinant <= 0.0) {
        return std::unexpected(std::format(
            "{}.t_bs: expected a proper rotation, found determinant {}", field_name, determinant));
    }
    const Eigen::Vector4d bottom_row = camera.t_bs.row(3);
    if (!bottom_row.isApprox(Eigen::Vector4d{0.0, 0.0, 0.0, 1.0}, kRigidTransformTolerance)) {
        return std::unexpected(std::format(
            "{}.t_bs: expected bottom row [0, 0, 0, 1], found [{}, {}, {}, {}]",
            field_name,
            bottom_row.x(),
            bottom_row.y(),
            bottom_row.z(),
            bottom_row.w()));
    }
    return camera.t_bs;
}

}  // namespace

ParseResult<Eigen::Matrix4d> body_from_camera_transform(
    const CameraCalibration& camera,
    std::string_view field_name) {
    return validate_camera_extrinsics(camera, field_name);
}

ParseResult<Eigen::Matrix4d> camera_from_body_transform(
    const CameraCalibration& camera,
    std::string_view field_name) {
    const auto body_from_camera = validate_camera_extrinsics(camera, field_name);
    if (!body_from_camera) {
        return std::unexpected(body_from_camera.error());
    }

    Eigen::Matrix4d camera_from_body = Eigen::Matrix4d::Identity();
    const Eigen::Matrix3d rotation = body_from_camera->block<3, 3>(0, 0);
    const Eigen::Vector3d translation = body_from_camera->block<3, 1>(0, 3);
    camera_from_body.topLeftCorner<3, 3>() = rotation.transpose();
    camera_from_body.topRightCorner<3, 1>() = -rotation.transpose() * translation;
    return camera_from_body;
}

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
    const auto body_from_camera = body_from_camera_transform(camera);
    if (!body_from_camera) {
        return std::unexpected(body_from_camera.error());
    }
    const Eigen::Matrix3d rotation_body_from_camera = body_from_camera->block<3, 3>(0, 0);
    const Eigen::Vector3d translation_body_from_camera = body_from_camera->block<3, 1>(0, 3);
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
    if (!robot.position.allFinite() || !robot.orientation.unit_quaternion().coeffs().allFinite()) {
        return std::unexpected("robot_pose: expected finite position and orientation");
    }

    const auto camera_from_body = camera_from_body_transform(camera);
    if (!camera_from_body) {
        return std::unexpected(camera_from_body.error());
    }
    const Eigen::Vector3d point_body = robot.orientation.inverse() * (point_world - robot.position);
    return camera_from_body->block<3, 3>(0, 0) * point_body
        + camera_from_body->block<3, 1>(0, 3);
}
