#include "initialization.hpp"

#include <cmath>
#include <format>

ParseResult<NominalState> initialize_static_imu(
    std::span<const ImuMeasurement> stationary_measurements,
    const StaticInitializationOptions& options) {
    if (stationary_measurements.size() < options.minimum_samples) {
        return std::unexpected(std::format(
            "stationary_measurements: expected at least {} samples, found {}",
            options.minimum_samples, stationary_measurements.size()));
    }
    if (!(std::isfinite(options.gravity_magnitude) && options.gravity_magnitude > 0.0)) {
        return std::unexpected(std::format(
            "gravity_magnitude: expected a positive finite value, found {}", options.gravity_magnitude));
    }
    if (!(std::isfinite(options.maximum_mean_angular_velocity_rad_per_sec)
        && options.maximum_mean_angular_velocity_rad_per_sec >= 0.0)) {
        return std::unexpected("maximum_mean_angular_velocity_rad_per_sec: expected a finite non-negative value");
    }
    if (!(std::isfinite(options.gravity_magnitude_tolerance_m_per_sec_squared)
        && options.gravity_magnitude_tolerance_m_per_sec_squared >= 0.0)) {
        return std::unexpected("gravity_magnitude_tolerance_m_per_sec_squared: expected a finite non-negative value");
    }
    Eigen::Vector3d mean_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_angular_velocity = Eigen::Vector3d::Zero();
    for (const ImuMeasurement& measurement : stationary_measurements) {
        if (!measurement.acceleration.allFinite() || !measurement.angular_velocity.allFinite()) {
            return std::unexpected("stationary_measurements: expected finite IMU values");
        }
        mean_acceleration += measurement.acceleration;
        mean_angular_velocity += measurement.angular_velocity;
    }
    const double count = static_cast<double>(stationary_measurements.size());
    mean_acceleration /= count;
    mean_angular_velocity /= count;
    if (mean_acceleration.norm() < 1e-9) {
        return std::unexpected(std::format(
            "stationary_measurements: mean acceleration has near-zero magnitude {}", mean_acceleration.norm()));
    }
    if (mean_angular_velocity.norm() > options.maximum_mean_angular_velocity_rad_per_sec) {
        return std::unexpected(std::format(
            "stationary_measurements: mean angular velocity {} exceeds stationary limit {}",
            mean_angular_velocity.norm(), options.maximum_mean_angular_velocity_rad_per_sec));
    }
    if (std::abs(mean_acceleration.norm() - options.gravity_magnitude)
        > options.gravity_magnitude_tolerance_m_per_sec_squared) {
        return std::unexpected(std::format(
            "stationary_measurements: mean acceleration magnitude {} differs from gravity {} by more than {}",
            mean_acceleration.norm(), options.gravity_magnitude,
            options.gravity_magnitude_tolerance_m_per_sec_squared));
    }

    // At rest, f_B = R_BW (-g_W). Rotate the measured up direction to the
    // body direction implied by the supplied heading, preserving its yaw.
    const Eigen::Vector3d measured_up_body = mean_acceleration.normalized();
    const Eigen::Vector3d reference_up_body =
        options.heading_reference.inverse() * Eigen::Vector3d::UnitZ();
    const Sophus::SO3d tilt_correction{
        Eigen::Quaterniond::FromTwoVectors(measured_up_body, reference_up_body)};

    return NominalState{
        .position = options.initial_position_world,
        .velocity = Eigen::Vector3d::Zero(),
        .orientation = options.heading_reference * tilt_correction,
        .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = mean_angular_velocity,
    };
}
