#include "image_io.hpp"

#include <format>

#include <opencv2/imgcodecs.hpp>

ParseResult<GrayImage> load_grayscale_png(const std::filesystem::path& path) {
    const cv::Mat decoded = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
    if (decoded.empty()) {
        return std::unexpected(std::format("image '{}': expected a readable grayscale PNG", path.string()));
    }
    if (decoded.type() != CV_8UC1) {
        return std::unexpected(std::format("image '{}': expected 8-bit single-channel pixels", path.string()));
    }
    GrayImage image{.width = decoded.cols, .height = decoded.rows};
    image.data.reserve(static_cast<std::size_t>(image.width) * image.height);
    for (int row = 0; row < image.height; ++row) {
        const auto* begin = decoded.ptr<std::uint8_t>(row);
        image.data.insert(image.data.end(), begin, begin + image.width);
    }
    return image;
}
