#pragma once

#include "parser.hpp"
#include "types.hpp"

#include <Eigen/Core>
#include <sophus/so3.hpp>

struct PinholePixelPrediction {
    Eigen::Vector3d landmark_body;
    Eigen::Vector3d landmark_camera;
    Eigen::Vector2d normalized;
    Eigen::Vector2d pixel;
};

// Pure measurement model h(x): pose and one metric XYZ landmark in, predicted
// pinhole pixel out. Visibility and measurement noise stay outside this API.
ParseResult<PinholePixelPrediction> predict_pinhole_pixel(
    const Sophus::SO3d& rotation_world_from_body,
    const Eigen::Vector3d& position_world_from_body,
    const Eigen::Vector3d& landmark_world,
    const CameraCalibration& camera);
