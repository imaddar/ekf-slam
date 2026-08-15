#pragma once

#include "parser.hpp"
#include "state.hpp"
#include "types.hpp"

#include <Eigen/Core>

struct AugmentationJacobians {
    Eigen::Matrix<double, 3, kImuErrorStateSize> robot;
    Eigen::Matrix3d camera;
};

ParseResult<AugmentationJacobians> make_augmentation_jacobians(
    const NominalState& robot,
    const CameraCalibration& camera,
    const Eigen::Vector3d& point_camera);
