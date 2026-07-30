#include "parser_csv.hpp"

#include <charconv>
#include <cctype>
#include <expected>
#include <format>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#define TRY(var, expr)            \
    auto var##_result = (expr);   \
    if (!var##_result) {          \
        return std::unexpected(var##_result.error()); \
    }                             \
    auto var = std::move(*var##_result)

namespace {

struct CameraFrame {
    TimestampNs timestamp;
    std::filesystem::path image_path;
};

std::string trim(std::string_view value) {
    auto begin = value.begin();
    auto end = value.end();

    while (begin != end && std::isspace(static_cast<unsigned char>(*begin))) {
        ++begin;
    }

    while (begin != end && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
        --end;
    }

    return std::string(begin, end);
}

ParseResult<std::string> read_csv_file(const std::filesystem::path& file_path, std::string_view record_kind) {
    std::ifstream file(file_path);
    if (!file) {
        return std::unexpected(std::format(
            "Failed to read {} CSV file: {}",
            record_kind,
            file_path.string()));
    }

    std::ostringstream content;
    content << file.rdbuf();
    return content.str();
}

ParseResult<TimestampNs> parse_timestamp(std::string_view value, std::string_view field_name, std::size_t line_number) {
    TimestampNs parsed = 0;
    const std::string trimmed = trim(value);
    const auto* begin = trimmed.data();
    const auto* end = begin + trimmed.size();
    const auto result = std::from_chars(begin, end, parsed);
    if (result.ec != std::errc{} || result.ptr != end) {
        return std::unexpected(std::format("Failed to parse {} on line {} as u64", field_name, line_number));
    }

    return parsed;
}

ParseResult<double> parse_csv_double(std::string_view value, std::string_view field_name, std::size_t line_number) {
    double parsed = 0.0;
    const std::string trimmed = trim(value);
    const auto* begin = trimmed.data();
    const auto* end = begin + trimmed.size();
    const auto result = std::from_chars(begin, end, parsed);
    if (result.ec != std::errc{} || result.ptr != end) {
        return std::unexpected(std::format("Failed to parse {} on line {} as f64", field_name, line_number));
    }

    return parsed;
}

std::vector<std::string> csv_fields(std::string_view line) {
    std::vector<std::string> fields;
    std::istringstream stream{std::string(line)};
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(trim(field));
    }

    return fields;
}

ParseResult<void> expect_field_count(
    const std::vector<std::string>& fields,
    std::size_t expected,
    std::string_view record_name,
    std::size_t line_number) {
    if (fields.size() != expected) {
        return std::unexpected(std::format(
            "{} line {} must contain {} fields, got {}",
            record_name,
            line_number,
            expected,
            fields.size()));
    }

    return {};
}

std::vector<std::pair<std::size_t, std::string>> csv_data_lines(std::string_view content) {
    std::vector<std::pair<std::size_t, std::string>> lines;
    std::istringstream stream{std::string(content)};
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(stream, line)) {
        ++line_number;
        std::string trimmed = trim(line);
        // Keep original line numbers in errors even though headers and blank lines are skipped.
        if (trimmed.empty() || trimmed.starts_with('#')) {
            continue;
        }

        lines.emplace_back(line_number, std::move(trimmed));
    }

    return lines;
}

ParseResult<ImuMeasurement> parse_imu_measurement_row(std::string_view line, std::size_t line_number) {
    const auto fields = csv_fields(line);
    if (auto field_count = expect_field_count(fields, 7, "IMU measurement", line_number); !field_count) {
        return std::unexpected(field_count.error());
    }

    TRY(timestamp, parse_timestamp(fields[0], "timestamp", line_number));
    TRY(wx, parse_csv_double(fields[1], "angular_velocity.x", line_number));
    TRY(wy, parse_csv_double(fields[2], "angular_velocity.y", line_number));
    TRY(wz, parse_csv_double(fields[3], "angular_velocity.z", line_number));
    TRY(ax, parse_csv_double(fields[4], "acceleration.x", line_number));
    TRY(ay, parse_csv_double(fields[5], "acceleration.y", line_number));
    TRY(az, parse_csv_double(fields[6], "acceleration.z", line_number));

    return ImuMeasurement{
        .timestamp = timestamp,
        .acceleration = Eigen::Vector3d(ax, ay, az),
        .angular_velocity = Eigen::Vector3d(wx, wy, wz),
    };
}

ParseResult<GroundTruthState> parse_ground_truth_row(std::string_view line, std::size_t line_number) {
    const auto fields = csv_fields(line);
    if (auto field_count = expect_field_count(fields, 17, "ground truth state", line_number); !field_count) {
        return std::unexpected(field_count.error());
    }

    TRY(timestamp, parse_timestamp(fields[0], "timestamp", line_number));
    TRY(px, parse_csv_double(fields[1], "position.x", line_number));
    TRY(py, parse_csv_double(fields[2], "position.y", line_number));
    TRY(pz, parse_csv_double(fields[3], "position.z", line_number));
    TRY(qw, parse_csv_double(fields[4], "orientation.w", line_number));
    TRY(qx, parse_csv_double(fields[5], "orientation.x", line_number));
    TRY(qy, parse_csv_double(fields[6], "orientation.y", line_number));
    TRY(qz, parse_csv_double(fields[7], "orientation.z", line_number));
    TRY(vx, parse_csv_double(fields[8], "velocity.x", line_number));
    TRY(vy, parse_csv_double(fields[9], "velocity.y", line_number));
    TRY(vz, parse_csv_double(fields[10], "velocity.z", line_number));
    TRY(bgx, parse_csv_double(fields[11], "gyroscope_bias.x", line_number));
    TRY(bgy, parse_csv_double(fields[12], "gyroscope_bias.y", line_number));
    TRY(bgz, parse_csv_double(fields[13], "gyroscope_bias.z", line_number));
    TRY(bax, parse_csv_double(fields[14], "accelerometer_bias.x", line_number));
    TRY(bay, parse_csv_double(fields[15], "accelerometer_bias.y", line_number));
    TRY(baz, parse_csv_double(fields[16], "accelerometer_bias.z", line_number));

    return GroundTruthState{
        .timestamp = timestamp,
        .position = Eigen::Vector3d(px, py, pz),
        .orientation = Eigen::Quaterniond(qw, qx, qy, qz),
        .velocity = Eigen::Vector3d(vx, vy, vz),
        .gyroscope_bias = Eigen::Vector3d(bgx, bgy, bgz),
        .accelerometer_bias = Eigen::Vector3d(bax, bay, baz),
    };
}

ParseResult<CameraFrame> parse_camera_frame_row(
    std::string_view line,
    std::size_t line_number,
    const std::filesystem::path& image_dir) {
    const auto fields = csv_fields(line);
    if (auto field_count = expect_field_count(fields, 2, "camera frame", line_number); !field_count) {
        return std::unexpected(field_count.error());
    }

    if (fields[1].empty()) {
        return std::unexpected(std::format("camera frame line {} has an empty filename", line_number));
    }

    TRY(timestamp, parse_timestamp(fields[0], "timestamp", line_number));
    return CameraFrame{.timestamp = timestamp, .image_path = image_dir / fields[1]};
}

ParseResult<std::vector<CameraFrame>> parse_camera_frames_csv_content(
    std::string_view content,
    const std::filesystem::path& image_dir) {
    std::vector<CameraFrame> frames;
    for (const auto& [line_number, line] : csv_data_lines(content)) {
        TRY(frame, parse_camera_frame_row(line, line_number, image_dir));
        frames.push_back(std::move(frame));
    }

    return frames;
}

ParseResult<std::vector<StereoPair>> pair_stereo_frames(
    std::vector<CameraFrame> cam0_frames,
    std::vector<CameraFrame> cam1_frames) {
    if (cam0_frames.size() != cam1_frames.size()) {
        return std::unexpected(std::format(
            "cam0 and cam1 must contain the same number of frames, got {} and {}",
            cam0_frames.size(),
            cam1_frames.size()));
    }

    std::vector<StereoPair> pairs;
    pairs.reserve(cam0_frames.size());
    for (std::size_t index = 0; index < cam0_frames.size(); ++index) {
        const auto& cam0_frame = cam0_frames[index];
        const auto& cam1_frame = cam1_frames[index];
        // Stereo CSVs should align by index, but compare timestamps so a shifted file fails loudly.
        if (cam0_frame.timestamp != cam1_frame.timestamp) {
            return std::unexpected(std::format(
                "cam0 timestamp {} does not match cam1 timestamp {}",
                cam0_frame.timestamp,
                cam1_frame.timestamp));
        }

        pairs.push_back(StereoPair{
            .timestamp = cam0_frame.timestamp,
            .cam0_image_path = std::move(cam0_frames[index].image_path),
            .cam1_image_path = std::move(cam1_frames[index].image_path),
        });
    }

    return pairs;
}

}  // namespace

namespace parser_detail {

ParseResult<std::vector<ImuMeasurement>> parse_imu_measurements_csv(const std::filesystem::path& file_path) {
    TRY(content, read_csv_file(file_path, "IMU"));

    std::vector<ImuMeasurement> measurements;
    for (const auto& [line_number, line] : csv_data_lines(content)) {
        TRY(measurement, parse_imu_measurement_row(line, line_number));
        measurements.push_back(std::move(measurement));
    }

    return measurements;
}

ParseResult<std::vector<GroundTruthState>> parse_ground_truth_csv(const std::filesystem::path& file_path) {
    TRY(content, read_csv_file(file_path, "ground truth"));

    std::vector<GroundTruthState> states;
    for (const auto& [line_number, line] : csv_data_lines(content)) {
        TRY(state, parse_ground_truth_row(line, line_number));
        states.push_back(std::move(state));
    }

    return states;
}

ParseResult<std::vector<StereoPair>> parse_stereo_pairs_csv(
    const std::filesystem::path& cam0_csv_path,
    const std::filesystem::path& cam0_image_dir,
    const std::filesystem::path& cam1_csv_path,
    const std::filesystem::path& cam1_image_dir) {
    TRY(cam0_content, read_csv_file(cam0_csv_path, "camera"));
    TRY(cam1_content, read_csv_file(cam1_csv_path, "camera"));
    TRY(cam0_frames, parse_camera_frames_csv_content(cam0_content, cam0_image_dir));
    TRY(cam1_frames, parse_camera_frames_csv_content(cam1_content, cam1_image_dir));

    return pair_stereo_frames(std::move(cam0_frames), std::move(cam1_frames));
}

}  // namespace parser_detail

#undef TRY
