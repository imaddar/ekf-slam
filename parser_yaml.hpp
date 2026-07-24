#pragma once

#include <filesystem>

#include "parser.hpp"

namespace parser_detail {

ParseResult<CameraCalibration> parse_camera_yaml(const std::filesystem::path& file_path);
ParseResult<ImuCalibration> parse_imu_yaml(const std::filesystem::path& file_path);

}  // namespace parser_detail
