#include "corner_detector.hpp"
#include "image_io.hpp"
#include "rectification.hpp"

#include <gtest/gtest.h>

#include <filesystem>

namespace {

// Direct transcription of the original detector: for every pixel, rebuild the
// structure tensor by summing central-difference gradients over the window.
// Kept as a test oracle so the O(1)-per-pixel implementation has something
// independent to be checked against.
std::vector<Corner> detect_corners_reference(const GrayImage& image, std::span<const Eigen::Vector2d> existing,
                                             const DetectorOptions& options) {
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
    // Total order, matching the implementation: equal scores are common on real
    // frames, and an unstable sort would otherwise pick arbitrarily among them.
    std::sort(candidates.begin(), candidates.end(), [](const Corner& a, const Corner& b) {
        if (a.score != b.score) return a.score > b.score;
        if (a.pixel.y() != b.pixel.y()) return a.pixel.y() < b.pixel.y();
        return a.pixel.x() < b.pixel.x();
    });
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

// A real rectified MH_01 frame; synthetic patterns are too smooth to exercise
// the score threshold and grid buckets the way real texture does.
GrayImage load_rectified_mh01_frame() {
    const std::filesystem::path root = std::filesystem::path{EKF_SLAM_SOURCE_DIR} / "datasets/machine_hall/MH_01_easy";
    const auto dataset = parse_dataset(root);
    EXPECT_TRUE(dataset.has_value()) << (dataset ? "" : dataset.error());
    const auto rectification = make_stereo_rectification(dataset->cam0_calibration, dataset->cam1_calibration);
    EXPECT_TRUE(rectification.has_value()) << (rectification ? "" : rectification.error());
    const auto raw = load_grayscale_png(dataset->stereo_pairs.front().cam0_image_path);
    EXPECT_TRUE(raw.has_value()) << (raw ? "" : raw.error());
    const auto rectified = rectify_cam0(*rectification, *raw);
    EXPECT_TRUE(rectified.has_value()) << (rectified ? "" : rectified.error());
    return *rectified;
}

}  // namespace

TEST(CornerDetectorTest, MatchesTheReferenceImplementationOnARealFrame) {
    const GrayImage image = load_rectified_mh01_frame();
    const DetectorOptions options{};
    const std::vector<Corner> expected = detect_corners_reference(image, {}, options);
    const std::vector<Corner> actual = detect_corners(image, {}, options);

    ASSERT_FALSE(expected.empty());
    ASSERT_EQ(expected.size(), actual.size());
    for (std::size_t index = 0; index < expected.size(); ++index) {
        EXPECT_EQ(expected[index].pixel.x(), actual[index].pixel.x()) << "at index " << index;
        EXPECT_EQ(expected[index].pixel.y(), actual[index].pixel.y()) << "at index " << index;
        // Box sums accumulate in a different order than the per-window sums, so
        // scores agree to floating-point rounding rather than exactly.
        EXPECT_NEAR(expected[index].score, actual[index].score, 1e-12) << "at index " << index;
    }
}

TEST(CornerDetectorTest, MatchesTheReferenceImplementationWithOccupiedCells) {
    const GrayImage image = load_rectified_mh01_frame();
    const DetectorOptions options{};
    // Seed existing features so the grid caps and separation radius both bite.
    std::vector<Eigen::Vector2d> existing;
    for (int y = 40; y < image.height - 40; y += 60) for (int x = 40; x < image.width - 40; x += 60) {
        existing.push_back(Eigen::Vector2d{x, y});
    }
    const std::vector<Corner> expected = detect_corners_reference(image, existing, options);
    const std::vector<Corner> actual = detect_corners(image, existing, options);

    ASSERT_FALSE(expected.empty());
    ASSERT_EQ(expected.size(), actual.size());
    for (std::size_t index = 0; index < expected.size(); ++index) {
        EXPECT_EQ(expected[index].pixel.x(), actual[index].pixel.x()) << "at index " << index;
        EXPECT_EQ(expected[index].pixel.y(), actual[index].pixel.y()) << "at index " << index;
    }
}

TEST(CornerDetectorTest, ReturnsNothingWhenTheImageIsSmallerThanTheWindow) {
    const DetectorOptions options{};
    GrayImage tiny{.width = 2 * options.structure_tensor_window, .height = 2 * options.structure_tensor_window};
    tiny.data.assign(static_cast<std::size_t>(tiny.width) * tiny.height, 0);
    EXPECT_TRUE(detect_corners(tiny, {}, options).empty());
}
