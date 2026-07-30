#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

inline constexpr int kErrorStateSize = 15;
// Keep this block order in mind when filling F, Q, or hand-checking covariance slices.
using StateCovariance = Eigen::Matrix<double, kErrorStateSize, kErrorStateSize>;

struct NominalState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Sophus::SO3d orientation;
    Eigen::Vector3d accelerometer_bias;
    Eigen::Vector3d gyroscope_bias;
};
