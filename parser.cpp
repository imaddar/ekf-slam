#include "parser.hpp"

#include <expected>
#include <utility>

#include "parser_csv.hpp"
#include "parser_yaml.hpp"

#define TRY(var, expr)            \
    auto var##_result = (expr);   \
    if (!var##_result) {          \
        return std::unexpected(var##_result.error()); \
    }                             \
    auto var = std::move(*var##_result)

ParseResult<Dataset> parse_dataset(const std::filesystem::path& sequence_root) {
    const auto mav0_root = sequence_root / "mav0";
    const auto cam0_root = mav0_root / "cam0";
    const auto cam1_root = mav0_root / "cam1";
    const auto imu0_root = mav0_root / "imu0";
    const auto ground_truth_root = mav0_root / "state_groundtruth_estimate0";

    TRY(cam0_calibration, parser_detail::parse_camera_yaml(cam0_root / "sensor.yaml"));
    TRY(cam1_calibration, parser_detail::parse_camera_yaml(cam1_root / "sensor.yaml"));
    TRY(imu_calibration, parser_detail::parse_imu_yaml(imu0_root / "sensor.yaml"));
    TRY(stereo_pairs, parser_detail::parse_stereo_pairs_csv(
                          cam0_root / "data.csv",
                          cam0_root / "data",
                          cam1_root / "data.csv",
                          cam1_root / "data"));
    TRY(imu_measurements, parser_detail::parse_imu_measurements_csv(imu0_root / "data.csv"));
    TRY(ground_truth_states, parser_detail::parse_ground_truth_csv(ground_truth_root / "data.csv"));

    return Dataset{
        .sequence_root = sequence_root,
        .cam0_calibration = cam0_calibration,
        .cam1_calibration = cam1_calibration,
        .imu_calibration = imu_calibration,
        .stereo_pairs = std::move(stereo_pairs),
        .imu_measurements = std::move(imu_measurements),
        .ground_truth_states = std::move(ground_truth_states),
    };
}

#undef TRY
