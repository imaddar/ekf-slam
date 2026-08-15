#pragma once

#include "parser.hpp"
#include "state.hpp"
#include "types.hpp"

#include <Eigen/Core>

// T_BS maps camera-frame points into the body frame; the inverse is computed
// from the validated rigid transform without a general matrix inversion.
ParseResult<Eigen::Matrix4d> body_from_camera_transform(const CameraCalibration& camera);
ParseResult<Eigen::Matrix4d> camera_from_body_transform(const CameraCalibration& camera);

ParseResult<Eigen::Vector3d> camera_point_to_world(
    const NominalState& robot,
    const CameraCalibration& camera,
    const Eigen::Vector3d& point_camera);

ParseResult<Eigen::Vector3d> world_point_to_camera(
    const NominalState& robot,
    const CameraCalibration& camera,
    const Eigen::Vector3d& point_world);
