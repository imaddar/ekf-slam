#pragma once

#include "parser.hpp"
#include "slam_state.hpp"
#include "types.hpp"

#include <Eigen/Core>

// Triangulates one already-rectified stereo observation and inserts its metric
// XYZ world position and covariance into the preallocated joint state. Raw EuRoC
// camera observations need a rectification layer before this API can consume
// them.
ParseResult<void> augment_landmark(
    SlamState& state,
    LandmarkId id,
    const Eigen::Vector2d& pixel_cam0,
    const Eigen::Vector2d& pixel_cam1,
    const CameraCalibration& cam0,
    const CameraCalibration& cam1,
    const Eigen::Matrix4d& pixel_covariance,
    double minimum_disparity_pixels = 1e-9);
