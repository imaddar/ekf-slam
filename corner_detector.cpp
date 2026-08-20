#include "corner_detector.hpp"

#include <algorithm>
#include <cmath>

namespace {

// Sliding-window sum of `source` over `2 * radius + 1` consecutive entries,
// written at the window centre. `stride` selects the axis: 1 walks a row,
// `width` walks a column. Each output costs one add and one subtract
// regardless of window size.
void sliding_sum(const std::vector<double>& source, std::vector<double>& destination, int radius,
                 int outer_begin, int outer_end, int inner_begin, int inner_end, int outer_stride, int inner_stride) {
    for (int outer = outer_begin; outer < outer_end; ++outer) {
        const int base = outer * outer_stride;
        double running = 0.0;
        for (int offset = -radius; offset <= radius; ++offset) {
            running += source[static_cast<std::size_t>(base + (inner_begin + offset) * inner_stride)];
        }
        destination[static_cast<std::size_t>(base + inner_begin * inner_stride)] = running;
        for (int inner = inner_begin + 1; inner < inner_end; ++inner) {
            running += source[static_cast<std::size_t>(base + (inner + radius) * inner_stride)]
                - source[static_cast<std::size_t>(base + (inner - radius - 1) * inner_stride)];
            destination[static_cast<std::size_t>(base + inner * inner_stride)] = running;
        }
    }
}

// Descending score, ties broken by raster position. A total order makes the
// selection independent of which unstable sort runs over the candidates, which
// is what lets a bounded prefix stand in for a full sort below.
bool higher_score(const Corner& a, const Corner& b) {
    if (a.score != b.score) return a.score > b.score;
    if (a.pixel.y() != b.pixel.y()) return a.pixel.y() < b.pixel.y();
    return a.pixel.x() < b.pixel.x();
}

// Image-sized scratch reused across frames. Every element the later passes read
// is written earlier in the same call, so after the first allocation this needs
// neither reallocation nor zeroing -- both of which cost more than the
// arithmetic they support.
struct DetectorWorkspace {
    std::vector<double> gxx, gxy, gyy, row_xx, row_xy, row_yy, sum_xx, sum_xy, sum_yy;
    std::vector<std::vector<Corner>> buckets;
    std::vector<Corner> selection;

    void resize(std::size_t pixels) {
        for (std::vector<double>* buffer : {&gxx, &gxy, &gyy, &row_xx, &row_xy, &row_yy, &sum_xx, &sum_xy, &sum_yy}) {
            if (buffer->size() < pixels) buffer->resize(pixels);
        }
    }
};

}  // namespace

std::vector<Corner> detect_corners(const GrayImage& image, std::span<const Eigen::Vector2d> existing, const DetectorOptions& options) {
    const int radius = options.structure_tensor_window;
    const int width = image.width, height = image.height;
    // Window centres run over [radius + 1, extent - radius - 2]; the gradients
    // they need extend one pixel further on each side.
    if (width < 2 * radius + 4 || height < 2 * radius + 4) return {};
    const std::size_t pixels = static_cast<std::size_t>(width) * height;

    thread_local DetectorWorkspace workspace;
    workspace.resize(pixels);
    std::vector<double>& gxx = workspace.gxx; std::vector<double>& gxy = workspace.gxy; std::vector<double>& gyy = workspace.gyy;

    // Each gradient is computed once here. The original formulation recomputed
    // it once per window tap, so about (2 * radius + 1)^2 times over.
    const auto at = [&image, width](int px, int py) { return image.data[static_cast<std::size_t>(py) * width + px] / 255.0; };
    for (int y = 1; y < height - 1; ++y) for (int x = 1; x < width - 1; ++x) {
        const double gx = (at(x + 1, y) - at(x - 1, y)) * 0.5;
        const double gy = (at(x, y + 1) - at(x, y - 1)) * 0.5;
        const std::size_t index = static_cast<std::size_t>(y) * width + x;
        gxx[index] = gx * gx; gxy[index] = gx * gy; gyy[index] = gy * gy;
    }

    // The box sum over the window is separable, so two O(1)-per-pixel passes
    // replace the O(radius^2) inner loops.
    const int x_begin = radius + 1, x_end = width - radius - 1;
    const int y_begin = radius + 1, y_end = height - radius - 1;
    std::vector<double>& row_xx = workspace.row_xx; std::vector<double>& row_xy = workspace.row_xy; std::vector<double>& row_yy = workspace.row_yy;
    std::vector<double>& sum_xx = workspace.sum_xx; std::vector<double>& sum_xy = workspace.sum_xy; std::vector<double>& sum_yy = workspace.sum_yy;
    sliding_sum(gxx, row_xx, radius, 1, height - 1, x_begin, x_end, width, 1);
    sliding_sum(gxy, row_xy, radius, 1, height - 1, x_begin, x_end, width, 1);
    sliding_sum(gyy, row_yy, radius, 1, height - 1, x_begin, x_end, width, 1);
    sliding_sum(row_xx, sum_xx, radius, x_begin, x_end, y_begin, y_end, 1, width);
    sliding_sum(row_xy, sum_xy, radius, x_begin, x_end, y_begin, y_end, 1, width);
    sliding_sum(row_yy, sum_yy, radius, x_begin, x_end, y_begin, y_end, 1, width);

    // Candidates are bucketed by grid cell as they are scored. Selection keeps
    // at most `max_features_per_cell` from any cell, so only each cell's top
    // few can ever matter -- and finding those is linear per cell, where one
    // global sort over every candidate (roughly 250k on a textured frame) was
    // the dominant cost of the whole detector.
    const std::size_t cells = static_cast<std::size_t>(options.grid_cols) * options.grid_rows;
    std::vector<std::vector<Corner>>& buckets = workspace.buckets;
    buckets.resize(cells);
    for (std::vector<Corner>& bucket : buckets) bucket.clear();
    for (int y = y_begin; y < y_end; ++y) for (int x = x_begin; x < x_end; ++x) {
        const std::size_t index = static_cast<std::size_t>(y) * width + x;
        const double xx = sum_xx[index], xy = sum_xy[index], yy = sum_yy[index];
        const double score = 0.5 * (xx + yy - std::sqrt((xx - yy) * (xx - yy) + 4.0 * xy * xy));
        if (score < options.min_eigenvalue) continue;
        const int cx = std::min(options.grid_cols - 1, x * options.grid_cols / width);
        const int cy = std::min(options.grid_rows - 1, y * options.grid_rows / height);
        buckets[static_cast<std::size_t>(cy) * options.grid_cols + cx].push_back({Eigen::Vector2d{x, y}, score});
    }
    const std::vector<int> occupancy = [&] {
        std::vector<int> per_cell(static_cast<std::size_t>(options.grid_cols) * options.grid_rows);
        for (const Eigen::Vector2d& feature : existing) {
            if (!in_bounds(image, feature.x(), feature.y())) continue;
            const int cx = std::min(options.grid_cols - 1, static_cast<int>(feature.x() * options.grid_cols / image.width));
            const int cy = std::min(options.grid_rows - 1, static_cast<int>(feature.y() * options.grid_rows / image.height));
            ++per_cell[static_cast<std::size_t>(cy) * options.grid_cols + cx];
        }
        return per_cell;
    }();

    // Retain each cell's top `retained` candidates and run selection over their
    // union in global score order. A candidate below its own cell's cut can
    // only matter if every retained one above it was rejected by the separation
    // radius, leaving that cell under its cap -- which is exactly the condition
    // checked below. When it happens the cut widens and selection repeats, so
    // the result matches a full sort by construction, not by assumption.
    std::vector<Corner>& selection = workspace.selection;
    std::vector<Corner> result;
    for (std::size_t retained = 4 * static_cast<std::size_t>(options.max_features_per_cell); ; retained *= 4) {
        selection.clear();
        for (std::vector<Corner>& bucket : buckets) {
            const std::size_t take = std::min(retained, bucket.size());
            if (take < bucket.size()) {
                std::nth_element(bucket.begin(), bucket.begin() + static_cast<std::ptrdiff_t>(take), bucket.end(), higher_score);
            }
            selection.insert(selection.end(), bucket.begin(), bucket.begin() + static_cast<std::ptrdiff_t>(take));
        }
        std::sort(selection.begin(), selection.end(), higher_score);

        std::vector<int> per_cell = occupancy;
        result.clear();
        for (const Corner& corner : selection) {
            const int cx = std::min(options.grid_cols - 1, static_cast<int>(corner.pixel.x() * options.grid_cols / image.width));
            const int cy = std::min(options.grid_rows - 1, static_cast<int>(corner.pixel.y() * options.grid_rows / image.height));
            int& count = per_cell[static_cast<std::size_t>(cy) * options.grid_cols + cx];
            if (count >= options.max_features_per_cell) continue;
            const auto close = [&](const Eigen::Vector2d& point) { return (point - corner.pixel).norm() < options.min_separation_px; };
            if (std::ranges::any_of(existing, close) || std::ranges::any_of(result, [&](const Corner& kept) { return close(kept.pixel); })) continue;
            result.push_back(corner); ++count;
        }

        const bool truncated_cell_left_unfilled = [&] {
            for (std::size_t cell = 0; cell < cells; ++cell) {
                if (buckets[cell].size() > retained && per_cell[cell] < options.max_features_per_cell) return true;
            }
            return false;
        }();
        if (!truncated_cell_left_unfilled) return result;
    }
}
