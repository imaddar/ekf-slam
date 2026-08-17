#pragma once

#include "klt_tracker.hpp"

struct StereoMatchOptions { double min_disparity_px = 1.0, max_disparity_px = 200.0, max_row_residual_px = 1.0; KltOptions klt{}; };
struct StereoMatch { Eigen::Vector2d pixel_cam0, pixel_cam1; double disparity = 0.0; bool valid = false; };
StereoMatch match_stereo(const ImagePyramid&, const ImagePyramid&, const Eigen::Vector2d&, double disparity_prior_px, const StereoMatchOptions& = {});
