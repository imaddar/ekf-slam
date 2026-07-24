#include <cctype>
#include <charconv>
#include <expected>
#include <format>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "parser.hpp"

#define TRY(var, expr)            \
    auto var##_result = (expr);   \
    if (!var##_result) {          \
        return std::unexpected(var##_result.error()); \
    }                             \
    auto var = std::move(*var##_result)

namespace {

struct RawTransform {
    std::size_t cols = 0;
    std::size_t rows = 0;
    std::vector<double> data;
};

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

ParseResult<std::string> read_file(const std::filesystem::path& file_path, std::string_view file_kind) {
    std::ifstream file(file_path);
    if (!file) {
        return std::unexpected(std::format(
            "Failed to read {} YAML file: {}",
            file_kind,
            file_path.string()));
    }

    std::ostringstream content;
    content << file.rdbuf();
    return content.str();
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

std::unordered_map<std::string, std::string> parse_flat_yaml(std::string_view content) {
    std::unordered_map<std::string, std::string> fields;
    std::string section;

    std::istringstream lines{std::string(content)};
    std::string line;
    while (std::getline(lines, line)) {
        std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        const auto colon = trimmed.find(':');
        if (colon == std::string::npos) {
            continue;
        }

        std::string key = trim(std::string_view(trimmed).substr(0, colon));
        std::string value = trim(std::string_view(trimmed).substr(colon + 1));
        if (value.empty()) {
            section = key;
            continue;
        }

        // EuRoC calibration files use simple nested maps; this assumes indentation
        // means one level under the most recent section and does not support tabs.
        if (!section.empty() && !line.empty() && std::isspace(static_cast<unsigned char>(line[0]))) {
            key = section + "." + key;
        } else {
            section.clear();
        }

        fields.emplace(std::move(key), std::move(value));
    }

    return fields;
}

ParseResult<std::string> required_field(
    const std::unordered_map<std::string, std::string>& fields,
    std::string_view field_name) {
    const auto field = fields.find(std::string(field_name));
    if (field == fields.end()) {
        return std::unexpected(std::format("Missing YAML field {}", field_name));
    }

    return field->second;
}

ParseResult<double> parse_double(std::string_view value, std::string_view field_name) {
    double parsed = 0.0;
    const std::string trimmed = trim(value);
    const auto* begin = trimmed.data();
    const auto* end = begin + trimmed.size();
    const auto result = std::from_chars(begin, end, parsed);
    if (result.ec != std::errc{} || result.ptr != end) {
        return std::unexpected(std::format("Failed to parse {} as f64", field_name));
    }

    return parsed;
}

ParseResult<std::size_t> parse_size(std::string_view value, std::string_view field_name) {
    std::size_t parsed = 0;
    const std::string trimmed = trim(value);
    const auto* begin = trimmed.data();
    const auto* end = begin + trimmed.size();
    const auto result = std::from_chars(begin, end, parsed);
    if (result.ec != std::errc{} || result.ptr != end) {
        return std::unexpected(std::format("Failed to parse {} as usize", field_name));
    }

    return parsed;
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

ParseResult<std::vector<std::string>> parse_flat_list_items(std::string_view value, std::string_view field_name) {
    const std::string trimmed = trim(value);
    if (trimmed.size() < 2 || trimmed.front() != '[' || trimmed.back() != ']') {
        return std::unexpected(std::format("{} must be a YAML inline list", field_name));
    }

    // This only supports flat scalar lists like [1.0, 2.0]; it is not a general
    // YAML list parser for nested collections or quoted strings with commas.
    std::vector<std::string> items;
    std::string inner = trimmed.substr(1, trimmed.size() - 2);
    std::istringstream stream(inner);
    std::string item;
    while (std::getline(stream, item, ',')) {
        items.push_back(trim(item));
    }

    return items;
}

ParseResult<std::vector<double>> parse_double_list(std::string_view value, std::string_view field_name) {
    TRY(items, parse_flat_list_items(value, field_name));

    std::vector<double> values;
    values.reserve(items.size());
    for (const auto& item : items) {
        TRY(parsed, parse_double(item, field_name));
        values.push_back(parsed);
    }

    return values;
}

ParseResult<Eigen::Vector4d> parse_vector4(
    const std::unordered_map<std::string, std::string>& fields,
    std::string_view field_name) {
    TRY(raw_value, required_field(fields, field_name));
    TRY(values, parse_double_list(raw_value, field_name));

    if (values.size() != 4) {
        return std::unexpected(std::format("{} must contain 4 values, got {}", field_name, values.size()));
    }

    return Eigen::Vector4d(values[0], values[1], values[2], values[3]);
}

ParseResult<Eigen::Vector2i> parse_resolution(const std::unordered_map<std::string, std::string>& fields) {
    TRY(raw_value, required_field(fields, "resolution"));
    TRY(items, parse_flat_list_items(raw_value, "resolution"));

    if (items.size() != 2) {
        return std::unexpected(std::format("resolution must contain 2 values, got {}", items.size()));
    }

    TRY(width, parse_size(items[0], "resolution.width"));
    TRY(height, parse_size(items[1], "resolution.height"));

    return Eigen::Vector2i(static_cast<int>(width), static_cast<int>(height));
}

ParseResult<RawTransform> parse_raw_transform(const std::unordered_map<std::string, std::string>& fields) {
    TRY(cols_value, required_field(fields, "T_BS.cols"));
    TRY(rows_value, required_field(fields, "T_BS.rows"));
    TRY(data_value, required_field(fields, "T_BS.data"));
    TRY(cols, parse_size(cols_value, "T_BS.cols"));
    TRY(rows, parse_size(rows_value, "T_BS.rows"));
    TRY(data, parse_double_list(data_value, "T_BS.data"));

    return RawTransform{.cols = cols, .rows = rows, .data = std::move(data)};
}

ParseResult<Eigen::Matrix4d> parse_t_bs(const std::unordered_map<std::string, std::string>& fields) {
    TRY(raw, parse_raw_transform(fields));

    if (raw.rows != 4 || raw.cols != 4) {
        return std::unexpected(std::format("T_BS must be 4x4, got {}x{}", raw.rows, raw.cols));
    }

    if (raw.data.size() != 16) {
        return std::unexpected(std::format("T_BS must contain 16 values, got {}", raw.data.size()));
    }

    Eigen::Matrix4d matrix;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            matrix(row, col) = raw.data[static_cast<std::size_t>(row * 4 + col)];
        }
    }

    return matrix;
}

ParseResult<double> parse_required_double(
    const std::unordered_map<std::string, std::string>& fields,
    std::string_view field_name) {
    TRY(value, required_field(fields, field_name));

    return parse_double(value, field_name);
}

}  // namespace

ParseResult<CameraCalibration> parse_camera_yaml(const std::filesystem::path& file_path) {
    TRY(content, read_file(file_path, "camera"));

    const auto fields = parse_flat_yaml(content);

    TRY(t_bs, parse_t_bs(fields));
    TRY(rate_hz, parse_required_double(fields, "rate_hz"));
    TRY(resolution, parse_resolution(fields));
    TRY(intrinsics, parse_vector4(fields, "intrinsics"));
    TRY(distortion_coefficients, parse_vector4(fields, "distortion_coefficients"));

    return CameraCalibration{
        .t_bs = t_bs,
        .rate_hz = rate_hz,
        .resolution = resolution,
        .intrinsics = intrinsics,
        .distortion_coefficients = distortion_coefficients,
    };
}

ParseResult<ImuCalibration> parse_imu_yaml(const std::filesystem::path& file_path) {
    TRY(content, read_file(file_path, "IMU"));

    const auto fields = parse_flat_yaml(content);

    TRY(t_bs, parse_t_bs(fields));
    TRY(rate_hz, parse_required_double(fields, "rate_hz"));
    TRY(gyroscope_noise_density, parse_required_double(fields, "gyroscope_noise_density"));
    TRY(gyroscope_random_walk, parse_required_double(fields, "gyroscope_random_walk"));
    TRY(accelerometer_noise_density, parse_required_double(fields, "accelerometer_noise_density"));
    TRY(accelerometer_random_walk, parse_required_double(fields, "accelerometer_random_walk"));

    return ImuCalibration{
        .t_bs = t_bs,
        .rate_hz = rate_hz,
        .gyroscope_noise_density = gyroscope_noise_density,
        .gyroscope_random_walk = gyroscope_random_walk,
        .accelerometer_noise_density = accelerometer_noise_density,
        .accelerometer_random_walk = accelerometer_random_walk,
    };
}

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

#undef TRY
