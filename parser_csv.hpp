#pragma once

#include <filesystem>
#include <vector>

#include "parser.hpp"

namespace parser_detail {

struct StereoPairing {
    std::vector<StereoPair> pairs;
    std::vector<StereoFrameGap> gaps;
};

ParseResult<std::vector<ImuMeasurement>> parse_imu_measurements_csv(const std::filesystem::path& file_path);
ParseResult<std::vector<GroundTruthState>> parse_ground_truth_csv(const std::filesystem::path& file_path);
ParseResult<StereoPairing> parse_stereo_pairs_csv(
    const std::filesystem::path& cam0_csv_path,
    const std::filesystem::path& cam0_image_dir,
    const std::filesystem::path& cam1_csv_path,
    const std::filesystem::path& cam1_image_dir);

}  // namespace parser_detail
