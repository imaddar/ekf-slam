#pragma once

#include "parser.hpp"
#include "types.hpp"

#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <sophus/so3.hpp>

// Noise is specified as continuous-time density, the same way EuRoC calibration
// reports it, so one config can drive both the injected noise and the filter's Q.
struct SyntheticImuNoiseConfig {
    double accelerometer_noise_density = 0.0;  // m/s^2/sqrt(Hz)
    double gyroscope_noise_density = 0.0;      // rad/s/sqrt(Hz)
    std::uint64_t seed = 0;
};

struct SyntheticPixelNoiseConfig {
    double pixel_stddev = 0.0;  // px
    std::uint64_t seed = 0;
};

// Owns the time origin and the true biases for every stream generated from it, so
// ground truth, IMU, and camera samples cannot drift onto different conventions.
struct SyntheticTrajectory {
    Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d initial_velocity = Eigen::Vector3d::Zero();
    Sophus::SO3d initial_orientation = Sophus::SO3d{};
    Eigen::Vector3d world_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d sinusoidal_world_acceleration = Eigen::Vector3d::Zero();
    double sinusoidal_frequency_rad_per_sec = 0.0;
    Eigen::Vector3d body_angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelerometer_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroscope_bias = Eigen::Vector3d::Zero();
    TimestampNs start_timestamp = 0;

    GroundTruthState state_at(double time_seconds) const;
    Eigen::Vector3d acceleration_at(double time_seconds) const;
    Eigen::Vector3d angular_velocity_at(double time_seconds) const;
    // Inverse of the sample timestamp mapping. Use it to look up truth at the
    // instant a generated sample actually landed on, rather than at a requested
    // duration the sample grid may not reach.
    double time_at(TimestampNs timestamp) const;
};

// Samples land on a 1/rate_hz grid from the trajectory start. A duration that is
// not a whole number of steps is truncated to the last grid point on or before it.
struct SyntheticImuConfig {
    double rate_hz = 200.0;
    double duration_seconds = 1.0;
    std::optional<SyntheticImuNoiseConfig> noise = std::nullopt;
};

struct SyntheticLandmark {
    int id = 0;
    Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
};

struct SyntheticCameraConfig {
    double rate_hz = 20.0;
    double duration_seconds = 1.0;
    double minimum_disparity_pixels = 1e-6;
    std::optional<SyntheticPixelNoiseConfig> pixel_noise = std::nullopt;
};

struct SyntheticStereoObservation {
    TimestampNs timestamp = 0;
    int landmark_id = 0;
    Eigen::Vector2d cam0_pixel = Eigen::Vector2d::Zero();
    Eigen::Vector2d cam1_pixel = Eigen::Vector2d::Zero();
    // Horizontal disparity, valid only for the rectified rig that
    // synthesize_stereo_observations enforces.
    double disparity_pixels = 0.0;
};

ImuCalibration make_synthetic_imu_calibration(
    double rate_hz,
    const std::optional<SyntheticImuNoiseConfig>& noise = std::nullopt);
CameraCalibration make_synthetic_pinhole_camera_calibration(
    const Eigen::Matrix4d& t_bs,
    double rate_hz = 20.0,
    const Eigen::Vector2i& resolution = Eigen::Vector2i{640, 480},
    const Eigen::Vector4d& intrinsics = Eigen::Vector4d{500.0, 500.0, 320.0, 240.0});
std::vector<SyntheticLandmark> make_synthetic_landmarks();
ParseResult<std::vector<GroundTruthState>> sample_ground_truth(
    const SyntheticTrajectory& trajectory,
    double rate_hz,
    double duration_seconds);
ParseResult<std::vector<ImuMeasurement>> synthesize_imu(
    const SyntheticTrajectory& trajectory,
    const SyntheticImuConfig& config);
// Requires a rectified rig: both cameras share an orientation and cam1 sits at a
// positive x baseline from cam0, which is what makes u0 - u1 a valid disparity.
ParseResult<std::vector<SyntheticStereoObservation>> synthesize_stereo_observations(
    const SyntheticTrajectory& trajectory,
    const std::vector<SyntheticLandmark>& landmarks,
    const CameraCalibration& cam0_calibration,
    const CameraCalibration& cam1_calibration,
    const SyntheticCameraConfig& config);
