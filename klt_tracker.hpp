#pragma once

#include "image_pyramid.hpp"

#include <Eigen/Core>

struct KltOptions {
    int window_half_size = 10, max_iterations = 30;
    double convergence_px = 0.01, max_residual_per_pixel = 20.0, min_hessian_eigenvalue = 1e-4;
    double forward_backward_px = 0.5;
    bool constrain_to_row = false;
};
enum class KltStatus { kTracked, kDiverged, kOutOfBounds, kIllConditioned, kHighResidual };
struct KltResult { Eigen::Vector2d pixel; KltStatus status; double residual = 0.0; };
KltResult track_feature(const ImagePyramid&, const ImagePyramid&, const Eigen::Vector2d&, const Eigen::Vector2d&, const KltOptions& = {});
bool passes_forward_backward(const ImagePyramid&, const ImagePyramid&, const Eigen::Vector2d&, const KltResult&, const KltOptions& = {});
