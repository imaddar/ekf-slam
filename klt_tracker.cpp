#include "klt_tracker.hpp"

#include <cmath>


namespace {

// Margin `half + 2` rather than `half + 1`: the patch samples central-difference
// gradients one pixel outside the window, and bilinear interpolation needs the
// pixel after that. Guaranteeing the whole footprint here is what lets the inner
// loops sample unchecked. It costs a one-pixel band at the image border.
bool valid_patch(const GrayImage& image, const Eigen::Vector2d& point, int half) {
    return in_bounds(image, point.x(), point.y(), static_cast<double>(half + 2));
}

// Per-level template quantities. The template patch, its gradients, and its
// mean do not change while the iteration refines `estimate` against the target,
// so they are computed once alongside the Hessian instead of on every pass.
struct TemplatePatch {
    std::vector<double> value, gradient_x, gradient_y, target_value;
    double mean = 0.0;

    void resize(std::size_t count) {
        value.resize(count); gradient_x.resize(count); gradient_y.resize(count); target_value.resize(count);
    }
};

}  // namespace

KltResult track_feature(const ImagePyramid& from, const ImagePyramid& to, const Eigen::Vector2d& from_pixel,
                        const Eigen::Vector2d& initial_guess, const KltOptions& options) {
    if (from.levels.empty() || from.levels.size() != to.levels.size()) return {initial_guess, KltStatus::kDiverged};
    thread_local TemplatePatch patch;
    // Start at the deepest level whose patch footprint still fits. The margin is
    // a constant number of pixels at each level, so in level-0 terms it costs
    // `(half + 2) * 2^level` at every border -- with 4 levels and a 21x21 window
    // that is 96 px, excluding 55% of a 752x480 frame where a match is
    // impossible rather than merely hard. The condition is monotone in level
    // (coarser is strictly tighter), so the deepest fitting level also
    // guarantees every finer one, and a border feature tracks with fewer levels
    // instead of failing before its first iteration. Interior features are
    // unaffected and still start at the top.
    int start_level = 0;
    for (int level = static_cast<int>(from.levels.size()) - 1; level > 0; --level) {
        const double scale = static_cast<double>(1 << level);
        if (valid_patch(from.levels[level], from_pixel / scale, options.window_half_size)
            && valid_patch(to.levels[level], initial_guess / scale, options.window_half_size)) {
            start_level = level;
            break;
        }
    }
    Eigen::Vector2d estimate = initial_guess / static_cast<double>(1 << start_level);
    for (int level = start_level; level >= 0; --level) {
        const double scale = static_cast<double>(1 << level);
        const Eigen::Vector2d template_point = from_pixel / scale;
        if (level != start_level) estimate *= 2.0;
        const GrayImage& template_image = from.levels[level];
        const GrayImage& target_image = to.levels[level];
        if (!valid_patch(template_image, template_point, options.window_half_size)
            || !valid_patch(target_image, estimate, options.window_half_size)) return {estimate * scale, KltStatus::kOutOfBounds};
        const int n = (2 * options.window_half_size + 1) * (2 * options.window_half_size + 1);
        patch.resize(static_cast<std::size_t>(n));
        Eigen::Matrix2d hessian = Eigen::Matrix2d::Zero();
        patch.mean = 0.0;
        std::size_t k = 0;
        for (int y = -options.window_half_size; y <= options.window_half_size; ++y) for (int x = -options.window_half_size; x <= options.window_half_size; ++x, ++k) {
            const Eigen::Vector2d p = template_point + Eigen::Vector2d{x, y};
            const double left = sample_bilinear_unchecked(template_image, p.x() - 1.0, p.y());
            const double right = sample_bilinear_unchecked(template_image, p.x() + 1.0, p.y());
            const double up = sample_bilinear_unchecked(template_image, p.x(), p.y() - 1.0);
            const double down = sample_bilinear_unchecked(template_image, p.x(), p.y() + 1.0);
            const Eigen::Vector2d gradient{(right - left) * 0.5, (down - up) * 0.5};
            patch.gradient_x[k] = gradient.x(); patch.gradient_y[k] = gradient.y();
            patch.value[k] = sample_bilinear_unchecked(template_image, p.x(), p.y());
            patch.mean += patch.value[k];
            hessian.noalias() += gradient * gradient.transpose();
        }
        patch.mean /= n;
        const double min_eigenvalue = 0.5 * (hessian.trace() - std::sqrt(
            (hessian(0, 0) - hessian(1, 1)) * (hessian(0, 0) - hessian(1, 1)) + 4.0 * hessian(0, 1) * hessian(0, 1)));
        if (min_eigenvalue < options.min_hessian_eigenvalue) return {estimate * scale, KltStatus::kIllConditioned};
        bool converged = false;
        double residual_per_pixel = 0.0;
        for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
            // Only the target moves between iterations, and it is sampled once
            // per pixel rather than once for the mean and again for the error.
            double target_mean = 0.0;
            k = 0;
            for (int y = -options.window_half_size; y <= options.window_half_size; ++y) for (int x = -options.window_half_size; x <= options.window_half_size; ++x, ++k) {
                patch.target_value[k] = sample_bilinear_unchecked(target_image, estimate.x() + x, estimate.y() + y);
                target_mean += patch.target_value[k];
            }
            target_mean /= n; Eigen::Vector2d gradient_error = Eigen::Vector2d::Zero(); double residual = 0;
            for (k = 0; k < static_cast<std::size_t>(n); ++k) {
                const double error = (patch.target_value[k] - target_mean) - (patch.value[k] - patch.mean);
                gradient_error += Eigen::Vector2d{patch.gradient_x[k], patch.gradient_y[k]} * error;
                residual += std::abs(error);
            }
            Eigen::Vector2d delta = hessian.ldlt().solve(gradient_error); if (options.constrain_to_row) delta.y() = 0.0;
            estimate -= delta;
            residual_per_pixel = residual / n;
            if (!valid_patch(target_image, estimate, options.window_half_size)) return {estimate * scale, KltStatus::kOutOfBounds};
            if (delta.norm() < options.convergence_px) { converged = true; break; }
        }
        // Converging at a coarse level means the estimate is good at that
        // level's resolution, not at full resolution. Carry it down and refine;
        // only level 0 produces a reportable result.
        if (level == 0) return {estimate, converged ? KltStatus::kTracked : KltStatus::kDiverged, residual_per_pixel};
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
