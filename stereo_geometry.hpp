#pragma once

#include "parser.hpp"
#include "state.hpp"
#include "types.hpp"

#include <Eigen/Core>

ParseResult<Eigen::Vector3d> camera_point_to_world(
    const NominalState& robot,
    const CameraCalibration& camera,
    const Eigen::Vector3d& point_camera);

ParseResult<Eigen::Vector3d> world_point_to_camera(
    const NominalState& robot,
    const CameraCalibration& camera,
    const Eigen::Vector3d& point_world);
