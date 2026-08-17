#pragma once

#include "image.hpp"

#include <filesystem>

ParseResult<GrayImage> load_grayscale_png(const std::filesystem::path& path);
