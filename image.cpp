#include "image.hpp"

#include <algorithm>
#include <cmath>
#include <format>

bool in_bounds(const GrayImage& image, double u, double v, double margin) {
    return image.width > 0 && image.height > 0 && std::isfinite(u) && std::isfinite(v)
        && std::isfinite(margin) && margin >= 0.0
        && u >= margin && v >= margin
        && u < static_cast<double>(image.width) - margin
        && v < static_cast<double>(image.height) - margin;
}

ParseResult<double> sample_bilinear(const GrayImage& image, double u, double v) {
    // A bilinear footprint needs the four neighbouring pixel centres.
    if (!in_bounds(image, u, v) || u > image.width - 1.0 || v > image.height - 1.0) {
        return std::unexpected(std::format("image sample: expected pixel inside [0, {}) x [0, {}), found [{}, {}]", image.width, image.height, u, v));
    }
    const int x0 = static_cast<int>(std::floor(u));
    const int y0 = static_cast<int>(std::floor(v));
    const int x1 = std::min(x0 + 1, image.width - 1);
    const int y1 = std::min(y0 + 1, image.height - 1);
    const double dx = u - x0;
    const double dy = v - y0;
    const auto at = [&image](int x, int y) { return static_cast<double>(image.data[static_cast<std::size_t>(y) * image.width + x]); };
    return (1.0 - dy) * ((1.0 - dx) * at(x0, y0) + dx * at(x1, y0))
        + dy * ((1.0 - dx) * at(x0, y1) + dx * at(x1, y1));
}
