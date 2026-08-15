#pragma once

#include "parser.hpp"
#include "state.hpp"
#include "types.hpp"

#include <Eigen/Core>
#include <sophus/so3.hpp>

struct PinholePixelPrediction {
    Eigen::Vector3d landmark_body;
    Eigen::Vector3d landmark_camera;
    Eigen::Vector2d normalized;
    Eigen::Vector2d pixel;
};

// The compact pose block is ordered [delta p_W, delta theta_B].  Consumers put
// its halves in the non-contiguous position and orientation error-state slots.
struct MeasurementJacobianBlocks {
    Eigen::Matrix<double, 2, 6> pose;
    Eigen::Matrix<double, 2, 3> landmark;
    int landmark_offset;
};

// Pure measurement model h(x): pose and one metric XYZ landmark in, predicted
// pinhole pixel out. Visibility and measurement noise stay outside this API.
ParseResult<PinholePixelPrediction> predict_pinhole_pixel(
    const Sophus::SO3d& rotation_world_from_body,
    const Eigen::Vector3d& position_world_from_body,
    const Eigen::Vector3d& landmark_world,
    const CameraCalibration& camera);

// Linearizes one predicted pixel with respect to the observing pose and one
// metric-XYZ world landmark. landmark_offset is resolved by SlamState first.
ParseResult<MeasurementJacobianBlocks> make_measurement_jacobian_blocks(
    const PinholePixelPrediction& prediction,
    const Sophus::SO3d& rotation_world_from_body,
    const CameraCalibration& camera,
    int landmark_offset);
