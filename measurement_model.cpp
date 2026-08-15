#include "measurement_model.hpp"

#include "stereo_geometry.hpp"

#include <format>

namespace {

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d skew;
    skew << 0.0, -vector.z(), vector.y(),
        vector.z(), 0.0, -vector.x(),
        -vector.y(), vector.x(), 0.0;
    return skew;
}

// The extrinsic is validated by camera_from_body_transform, which also supplies
// the inverse; only the intrinsics need a check the geometry path does not do.
ParseResult<void> validate_camera_intrinsics(const CameraCalibration& camera) {
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

ParseResult<MeasurementJacobianBlocks> make_measurement_jacobian_blocks(
    const PinholePixelPrediction& prediction,
    const Sophus::SO3d& rotation_world_from_body,
    const CameraCalibration& camera,
    int landmark_offset) {
    if (!prediction.landmark_body.allFinite() || !prediction.landmark_camera.allFinite()) {
        return std::unexpected("prediction: expected finite landmark body and camera coordinates");
    }
    if (!rotation_world_from_body.unit_quaternion().coeffs().allFinite()) {
        return std::unexpected("r_wb: expected finite rotation coefficients");
    }
    if (landmark_offset < kImuErrorStateSize || (landmark_offset - kImuErrorStateSize) % 3 != 0) {
        return std::unexpected(std::format(
            "landmark_offset: expected a landmark state column offset >= {}, found {}",
            kImuErrorStateSize,
            landmark_offset));
    }
    if (const auto valid = validate_camera_intrinsics(camera); !valid) {
        return std::unexpected(valid.error());
    }
    const auto camera_from_body = camera_from_body_transform(camera);
    if (!camera_from_body) {
        return std::unexpected(camera_from_body.error());
    }

    const double x = prediction.landmark_camera.x();
    const double y = prediction.landmark_camera.y();
    const double z = prediction.landmark_camera.z();
    if (z == 0.0) {
        return std::unexpected("prediction.landmark_camera.z: expected non-zero depth");
    }

    const double fx = camera.intrinsics.x();
    const double fy = camera.intrinsics.y();
    Eigen::Matrix<double, 2, 3> projection = Eigen::Matrix<double, 2, 3>::Zero();
    projection(0, 0) = fx / z;
    projection(0, 2) = -fx * x / (z * z);
    projection(1, 1) = fy / z;
    projection(1, 2) = -fy * y / (z * z);

    const Eigen::Matrix3d rotation_camera_from_body = camera_from_body->block<3, 3>(0, 0);
    const Eigen::Matrix3d rotation_body_from_world = rotation_world_from_body.inverse().matrix();
    const Eigen::Matrix<double, 2, 3> pose_position =
        projection * rotation_camera_from_body * -rotation_body_from_world;
    const Eigen::Matrix<double, 2, 3> pose_orientation =
        projection * rotation_camera_from_body * skew_symmetric(prediction.landmark_body);

    MeasurementJacobianBlocks blocks{
        .pose = Eigen::Matrix<double, 2, 6>::Zero(),
        .landmark = projection * rotation_camera_from_body * rotation_body_from_world,
        .landmark_offset = landmark_offset,
    };
    blocks.pose.leftCols<3>() = pose_position;
    blocks.pose.rightCols<3>() = pose_orientation;
    return blocks;
}

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
    if (const auto valid = validate_camera_intrinsics(camera); !valid) {
        return std::unexpected(valid.error());
    }
    const auto camera_from_body = camera_from_body_transform(camera);
    if (!camera_from_body) {
        return std::unexpected(camera_from_body.error());
    }

    const Eigen::Vector3d landmark_body =
        rotation_world_from_body.inverse() * (landmark_world - position_world_from_body);
    const Eigen::Vector3d landmark_camera = camera_from_body->block<3, 3>(0, 0) * landmark_body
        + camera_from_body->block<3, 1>(0, 3);
    // Ungated by design: callers must check Z > 0 and image bounds before
    // forming an innovation. Z == 0 is non-finite; Z < 0 can still be finite.
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
