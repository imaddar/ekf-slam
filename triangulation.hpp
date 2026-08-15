#pragma once

#include "parser.hpp"
#include "types.hpp"

#include <Eigen/Core>

struct StereoTriangulation {
    Eigen::Vector3d point_camera;
    Eigen::Matrix3d covariance;
    Eigen::Matrix<double, 3, 4> pixel_jacobian;
    double disparity_pixels;
};

// Pixel ordering is [u0, v0, u1, v1]. The returned point and covariance are
// expressed in camera 0's frame. This is the metric XYZ model used by
// augmentation; it requires already-rectified, distortion-free stereo pixels and
// rectified pseudo-calibrations. Raw EuRoC cam0/cam1 calibration does not satisfy
// this contract until an undistortion/rectification step is added.
ParseResult<StereoTriangulation> triangulate_stereo(
    const Eigen::Vector2d& pixel_cam0,
    const Eigen::Vector2d& pixel_cam1,
    const CameraCalibration& cam0,
    const CameraCalibration& cam1,
    const Eigen::Matrix4d& pixel_covariance,
    double minimum_disparity_pixels = 1e-9);
