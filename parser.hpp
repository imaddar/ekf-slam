#pragma once

#include <expected>
#include <filesystem>
#include <string>
#include <vector>

#include "types.hpp"

template <typename T>
using ParseResult = std::expected<T, std::string>;

ParseResult<CameraCalibration> parse_camera_yaml(const std::filesystem::path& file_path);
ParseResult<ImuCalibration> parse_imu_yaml(const std::filesystem::path& file_path);
ParseResult<std::vector<ImuMeasurement>> parse_imu_measurements_csv(const std::filesystem::path& file_path);
ParseResult<std::vector<GroundTruthState>> parse_ground_truth_csv(const std::filesystem::path& file_path);
ParseResult<std::vector<StereoPair>> parse_stereo_pairs_csv(
    const std::filesystem::path& cam0_csv_path,
    const std::filesystem::path& cam0_image_dir,
    const std::filesystem::path& cam1_csv_path,
    const std::filesystem::path& cam1_image_dir);
