#pragma once

#include "image.hpp"

#include <span>

#include <Eigen/Core>

struct DetectorOptions {
    int grid_cols = 12, grid_rows = 8, max_features_per_cell = 3;
    double min_eigenvalue = 1e-3, min_separation_px = 15.0;
    int structure_tensor_window = 3;
};
struct Corner { Eigen::Vector2d pixel; double score; };
std::vector<Corner> detect_corners(const GrayImage&, std::span<const Eigen::Vector2d>, const DetectorOptions& = {});
