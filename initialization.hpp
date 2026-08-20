#pragma once

#include "parser.hpp"
#include "state.hpp"
#include "types.hpp"

#include <span>

struct StaticInitializationOptions {
    // World position and heading must come from an external alignment source;
    // gravity alone cannot observe either quantity.
    Eigen::Vector3d initial_position_world = Eigen::Vector3d::Zero();
    Sophus::SO3d heading_reference = Sophus::SO3d{};
    double gravity_magnitude = 9.81;
    std::size_t minimum_samples = 100;
    double maximum_mean_angular_velocity_rad_per_sec = 0.05;
    double gravity_magnitude_tolerance_m_per_sec_squared = 0.2;
};

// Estimates gravity-aligned roll/pitch and gyroscope bias from a stationary
// IMU interval. Velocity and accelerometer bias are initialized to zero. The
// heading reference contributes yaw to first order; gravity alone cannot
// observe yaw, so the caller-supplied heading determines it.
ParseResult<NominalState> initialize_static_imu(
    std::span<const ImuMeasurement> stationary_measurements,
    const StaticInitializationOptions& options = {});
