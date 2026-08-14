#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

inline constexpr int kImuErrorStateSize = 15;
// Keep this block order in mind when filling F, Q, or hand-checking covariance slices.
using ImuStateCovariance = Eigen::Matrix<double, kImuErrorStateSize, kImuErrorStateSize>;

// Compatibility aliases for the current IMU-only propagation tests. New SLAM
// code should use explicit robot/joint-state names instead of these generic ones.
inline constexpr int kErrorStateSize = kImuErrorStateSize;
using StateCovariance = ImuStateCovariance;

struct NominalState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Sophus::SO3d orientation;
    Eigen::Vector3d accelerometer_bias;
    Eigen::Vector3d gyroscope_bias;
};
