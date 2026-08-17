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
    return sample_bilinear_unchecked(image, u, v);
}
