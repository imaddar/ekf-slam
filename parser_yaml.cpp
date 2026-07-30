#include "parser_yaml.hpp"

#include <cctype>
#include <charconv>
#include <expected>
#include <format>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

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

std::string strip_inline_comment(std::string_view value) {
    return trim(value.substr(0, value.find('#')));
}

ParseResult<std::string> read_yaml_file(const std::filesystem::path& file_path, std::string_view file_kind) {
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

std::unordered_map<std::string, std::string> parse_flat_yaml(std::string_view content) {
    std::unordered_map<std::string, std::string> fields;
    std::string section;
    std::string pending_key;
    std::string pending_value;

    std::istringstream lines{std::string(content)};
    std::string line;
    while (std::getline(lines, line)) {
        std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        if (!pending_key.empty()) {
            pending_value += " " + strip_inline_comment(trimmed);
            if (pending_value.find(']') != std::string::npos) {
                fields.emplace(std::move(pending_key), std::move(pending_value));
                pending_key.clear();
                pending_value.clear();
            }
            continue;
        }

        const auto colon = trimmed.find(':');
        if (colon == std::string::npos) {
            continue;
        }

        std::string key = trim(std::string_view(trimmed).substr(0, colon));
        std::string value = strip_inline_comment(std::string_view(trimmed).substr(colon + 1));
        if (value.empty()) {
            section = key;
            continue;
        }

        // EuRoC sensor.yaml only needs one-level nesting here, so avoid pretending this is full YAML.
        if (!section.empty() && !line.empty() && std::isspace(static_cast<unsigned char>(line[0]))) {
            key = section + "." + key;
        } else {
            section.clear();
        }

        if (value.find('[') != std::string::npos && value.find(']') == std::string::npos) {
            pending_key = std::move(key);
            pending_value = std::move(value);
            continue;
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

ParseResult<std::vector<std::string>> parse_flat_list_items(std::string_view value, std::string_view field_name) {
    const std::string trimmed = trim(value);
    const auto list_begin = trimmed.find('[');
    const auto list_end = trimmed.find(']', list_begin);
    if (list_begin == std::string::npos || list_end == std::string::npos || list_end <= list_begin) {
        return std::unexpected(std::format("{} must be a YAML inline list", field_name));
    }

    // These calibration lists are flat scalars; nested YAML belongs in a real parser later.
    std::vector<std::string> items;
    std::string inner = trimmed.substr(list_begin + 1, list_end - list_begin - 1);
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

namespace parser_detail {

ParseResult<CameraCalibration> parse_camera_yaml(const std::filesystem::path& file_path) {
    TRY(content, read_yaml_file(file_path, "camera"));

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
    TRY(content, read_yaml_file(file_path, "IMU"));

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

}  // namespace parser_detail

#undef TRY
