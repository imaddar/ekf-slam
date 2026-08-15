#include "augmentation_jacobians.hpp"

#include "stereo_geometry.hpp"

namespace {

constexpr int kOrientationIndex = 6;

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d skew;
    skew << 0.0, -vector.z(), vector.y(),
        vector.z(), 0.0, -vector.x(),
        -vector.y(), vector.x(), 0.0;
    return skew;
}

}  // namespace

ParseResult<AugmentationJacobians> make_augmentation_jacobians(
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

    const Eigen::Matrix3d rotation_world_from_body = robot.orientation.matrix();
    const Eigen::Matrix3d rotation_body_from_camera = body_from_camera->block<3, 3>(0, 0);
    const Eigen::Vector3d translation_body_from_camera = body_from_camera->block<3, 1>(0, 3);
    const Eigen::Vector3d point_body =
        rotation_body_from_camera * point_camera + translation_body_from_camera;

    AugmentationJacobians jacobians{
        .robot = Eigen::Matrix<double, 3, kImuErrorStateSize>::Zero(),
        .camera = rotation_world_from_body * rotation_body_from_camera,
    };
    jacobians.robot.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    jacobians.robot.block<3, 3>(0, kOrientationIndex) =
        -rotation_world_from_body * skew_symmetric(point_body);
    return jacobians;
}
