#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

using TimestampNs = std::uint64_t;

struct CameraCalibration {
    Eigen::Matrix4d t_bs;
    double rate_hz;
    Eigen::Vector2i resolution;
    Eigen::Vector4d intrinsics;
    Eigen::Vector4d distortion_coefficients;
};

struct ImuCalibration {
    Eigen::Matrix4d t_bs;
    double rate_hz;
    double gyroscope_noise_density;
    double gyroscope_random_walk;
    double accelerometer_noise_density;
    double accelerometer_random_walk;
};

struct StereoPair {
    TimestampNs timestamp;
    std::filesystem::path cam0_image_path;
    std::filesystem::path cam1_image_path;
};

struct ImuMeasurement {
    TimestampNs timestamp;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_velocity;
};

struct GroundTruthState {
    TimestampNs timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d gyroscope_bias;
    Eigen::Vector3d accelerometer_bias;
};

struct Dataset {
    std::filesystem::path sequence_root;
    CameraCalibration cam0_calibration;
    CameraCalibration cam1_calibration;
    ImuCalibration imu_calibration;
    std::vector<StereoPair> stereo_pairs;
    std::vector<ImuMeasurement> imu_measurements;
    std::vector<GroundTruthState> ground_truth_states;
};
