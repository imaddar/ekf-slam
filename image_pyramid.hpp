#pragma once

#include "image.hpp"

#include <vector>

inline constexpr int kPyramidLevels = 4;
struct ImagePyramid { std::vector<GrayImage> levels; };
ParseResult<ImagePyramid> build_pyramid(const GrayImage& image, int levels = kPyramidLevels);
