#include "corner_detector.hpp"

#include <algorithm>
#include <cmath>

std::vector<Corner> detect_corners(const GrayImage& image, std::span<const Eigen::Vector2d> existing, const DetectorOptions& options) {
    std::vector<Corner> candidates;
    const int radius = options.structure_tensor_window;
    for (int y = radius + 1; y < image.height - radius - 1; ++y) for (int x = radius + 1; x < image.width - radius - 1; ++x) {
        double xx = 0, xy = 0, yy = 0;
        for (int dy = -radius; dy <= radius; ++dy) for (int dx = -radius; dx <= radius; ++dx) {
            const auto at = [&image](int px, int py) { return image.data[static_cast<std::size_t>(py) * image.width + px] / 255.0; };
            const double gx = (at(x + dx + 1, y + dy) - at(x + dx - 1, y + dy)) * 0.5;
            const double gy = (at(x + dx, y + dy + 1) - at(x + dx, y + dy - 1)) * 0.5;
            xx += gx * gx; xy += gx * gy; yy += gy * gy;
        }
        const double score = 0.5 * (xx + yy - std::sqrt((xx - yy) * (xx - yy) + 4.0 * xy * xy));
        if (score >= options.min_eigenvalue) candidates.push_back({Eigen::Vector2d{x, y}, score});
    }
    std::sort(candidates.begin(), candidates.end(), [](const Corner& a, const Corner& b) { return a.score > b.score; });
    std::vector<int> per_cell(static_cast<std::size_t>(options.grid_cols) * options.grid_rows);
    for (const Eigen::Vector2d& feature : existing) {
        if (!in_bounds(image, feature.x(), feature.y())) continue;
        const int cx = std::min(options.grid_cols - 1, static_cast<int>(feature.x() * options.grid_cols / image.width));
        const int cy = std::min(options.grid_rows - 1, static_cast<int>(feature.y() * options.grid_rows / image.height));
        ++per_cell[static_cast<std::size_t>(cy) * options.grid_cols + cx];
    }
    std::vector<Corner> result;
    for (const Corner& corner : candidates) {
        const int cx = std::min(options.grid_cols - 1, static_cast<int>(corner.pixel.x() * options.grid_cols / image.width));
        const int cy = std::min(options.grid_rows - 1, static_cast<int>(corner.pixel.y() * options.grid_rows / image.height));
        int& count = per_cell[static_cast<std::size_t>(cy) * options.grid_cols + cx];
        if (count >= options.max_features_per_cell) continue;
        const auto close = [&](const Eigen::Vector2d& point) { return (point - corner.pixel).norm() < options.min_separation_px; };
        if (std::ranges::any_of(existing, close) || std::ranges::any_of(result, [&](const Corner& kept) { return close(kept.pixel); })) continue;
        result.push_back(corner); ++count;
    }
    return result;
}
