#pragma once

#include "parser.hpp"

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
