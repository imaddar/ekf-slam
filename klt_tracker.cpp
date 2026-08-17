#include "klt_tracker.hpp"

#include <cmath>


namespace {

bool valid_patch(const GrayImage& image, const Eigen::Vector2d& point, int half) {
    return in_bounds(image, point.x(), point.y(), static_cast<double>(half + 1));
}

}  // namespace

KltResult track_feature(const ImagePyramid& from, const ImagePyramid& to, const Eigen::Vector2d& from_pixel,
                        const Eigen::Vector2d& initial_guess, const KltOptions& options) {
    if (from.levels.empty() || from.levels.size() != to.levels.size()) return {initial_guess, KltStatus::kDiverged};
    Eigen::Vector2d estimate = initial_guess / static_cast<double>(1 << (static_cast<int>(from.levels.size()) - 1));
    for (int level = static_cast<int>(from.levels.size()) - 1; level >= 0; --level) {
        const double scale = static_cast<double>(1 << level);
        const Eigen::Vector2d template_point = from_pixel / scale;
        if (level != static_cast<int>(from.levels.size()) - 1) estimate *= 2.0;
        const GrayImage& template_image = from.levels[level];
        const GrayImage& target_image = to.levels[level];
        if (!valid_patch(template_image, template_point, options.window_half_size)
            || !valid_patch(target_image, estimate, options.window_half_size)) return {estimate * scale, KltStatus::kOutOfBounds};
        Eigen::Matrix2d hessian = Eigen::Matrix2d::Zero();
        for (int y = -options.window_half_size; y <= options.window_half_size; ++y) for (int x = -options.window_half_size; x <= options.window_half_size; ++x) {
            const Eigen::Vector2d p = template_point + Eigen::Vector2d{x, y};
            const auto left = sample_bilinear(template_image, p.x() - 1.0, p.y()); const auto right = sample_bilinear(template_image, p.x() + 1.0, p.y());
            const auto up = sample_bilinear(template_image, p.x(), p.y() - 1.0); const auto down = sample_bilinear(template_image, p.x(), p.y() + 1.0);
            if (!left || !right || !up || !down) return {estimate * scale, KltStatus::kOutOfBounds};
            const Eigen::Vector2d gradient{(*right - *left) * 0.5, (*down - *up) * 0.5}; hessian.noalias() += gradient * gradient.transpose();
        }
        const double min_eigenvalue = 0.5 * (hessian.trace() - std::sqrt(
            (hessian(0, 0) - hessian(1, 1)) * (hessian(0, 0) - hessian(1, 1)) + 4.0 * hessian(0, 1) * hessian(0, 1)));
        if (min_eigenvalue < options.min_hessian_eigenvalue) return {estimate * scale, KltStatus::kIllConditioned};
        for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
            double template_mean = 0.0, target_mean = 0.0;
            const int n = (2 * options.window_half_size + 1) * (2 * options.window_half_size + 1);
            for (int y = -options.window_half_size; y <= options.window_half_size; ++y) for (int x = -options.window_half_size; x <= options.window_half_size; ++x) {
                auto a = sample_bilinear(template_image, template_point.x() + x, template_point.y() + y);
                auto b = sample_bilinear(target_image, estimate.x() + x, estimate.y() + y);
                if (!a || !b) return {estimate * scale, KltStatus::kOutOfBounds}; template_mean += *a; target_mean += *b;
            }
            template_mean /= n; target_mean /= n; Eigen::Vector2d gradient_error = Eigen::Vector2d::Zero(); double residual = 0;
            for (int y = -options.window_half_size; y <= options.window_half_size; ++y) for (int x = -options.window_half_size; x <= options.window_half_size; ++x) {
                const Eigen::Vector2d p = template_point + Eigen::Vector2d{x, y};
                const double gx = (*sample_bilinear(template_image, p.x() + 1, p.y()) - *sample_bilinear(template_image, p.x() - 1, p.y())) * 0.5;
                const double gy = (*sample_bilinear(template_image, p.x(), p.y() + 1) - *sample_bilinear(template_image, p.x(), p.y() - 1)) * 0.5;
                const double error = (*sample_bilinear(target_image, estimate.x() + x, estimate.y() + y) - target_mean)
                    - (*sample_bilinear(template_image, p.x(), p.y()) - template_mean);
                gradient_error += Eigen::Vector2d{gx, gy} * error; residual += std::abs(error);
            }
            Eigen::Vector2d delta = hessian.ldlt().solve(gradient_error); if (options.constrain_to_row) delta.y() = 0.0;
            estimate -= delta;
            if (!valid_patch(target_image, estimate, options.window_half_size)) return {estimate * scale, KltStatus::kOutOfBounds};
            if (delta.norm() < options.convergence_px) return {estimate * scale, KltStatus::kTracked, residual / n};
        }
    }
    return {estimate, KltStatus::kDiverged};
}

bool passes_forward_backward(const ImagePyramid& from, const ImagePyramid& to, const Eigen::Vector2d& source,
                             const KltResult& forward, const KltOptions& options) {
    if (forward.status != KltStatus::kTracked || forward.residual > options.max_residual_per_pixel) return false;
    KltOptions reverse_options = options; reverse_options.constrain_to_row = false;
    const KltResult backward = track_feature(to, from, forward.pixel, source, reverse_options);
    return backward.status == KltStatus::kTracked && (backward.pixel - source).norm() <= options.forward_backward_px;
}
