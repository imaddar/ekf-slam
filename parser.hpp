#pragma once

#include <expected>
#include <filesystem>
#include <string>

#include "types.hpp"

template <typename T>
using ParseResult = std::expected<T, std::string>;

ParseResult<Dataset> parse_dataset(const std::filesystem::path& sequence_root);
