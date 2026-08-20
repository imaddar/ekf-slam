#pragma once

#include "parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

// An owned, tightly packed grayscale image. Pixel coordinates refer to pixel
// centres, so (0, 0) names the centre of the top-left sample.
struct GrayImage {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> data;
};

bool in_bounds(const GrayImage& image, double u, double v, double margin = 0.0);
ParseResult<double> sample_bilinear(const GrayImage& image, double u, double v);

// Bilinear sample with the bounds check left to the caller, who must have
// already established `in_bounds(image, u, v)` and `u <= width - 1`,
// `v <= height - 1` for the whole patch footprint. The checked form's
// ParseResult return sits in the tracker's innermost loop, where the error
// string it can never produce still costs a fat return value on every sample.
inline double sample_bilinear_unchecked(const GrayImage& image, double u, double v) {
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
