#include "klt_tracker.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

// Multi-scale texture. White noise would vanish under 8x downsampling and leave
// the coarse levels flat, which the eigenvalue floor would reject before any
// bounds question arose; incommensurate sinusoids survive the pyramid and keep
// the structure tensor well conditioned in both directions.
double texture(double x, double y) {
    return 128.0 + 60.0 * std::sin(x * 0.05) * std::cos(y * 0.07)
        + 40.0 * std::sin(x * 0.31 + y * 0.17) + 25.0 * std::cos(x * 0.11 - y * 0.13);
}

// `shift` moves image content, so a feature at p in the unshifted image appears
// at p + shift here.
GrayImage make_image(int width, int height, Eigen::Vector2d shift = {0.0, 0.0}) {
    GrayImage image{.width = width, .height = height};
    image.data.resize(static_cast<std::size_t>(width) * height);
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
        const double value = texture(x - shift.x(), y - shift.y());
        image.data[static_cast<std::size_t>(y) * width + x] =
            static_cast<std::uint8_t>(std::clamp(value, 0.0, 255.0));
    }
    return image;
}

constexpr int kWidth = 752, kHeight = 480;

}  // namespace

TEST(KltTrackerTest, TracksAFeatureNearTheBorderThatDoesNotFitTheCoarsestLevel) {
    // With `window_half_size = 10` the margin is 12 px per level, so the
    // coarsest of 4 levels needs a level-0 feature at x >= 96. This one sits at
    // 60 and used to fail out of bounds before its first iteration.
    const Eigen::Vector2d shift{3.0, 2.0};
    const auto from = build_pyramid(make_image(kWidth, kHeight));
    const auto to = build_pyramid(make_image(kWidth, kHeight, shift));
    ASSERT_TRUE(from.has_value());
    ASSERT_TRUE(to.has_value());

    const Eigen::Vector2d feature{60.0, 120.0};
    const KltResult result = track_feature(*from, *to, feature, feature);

    EXPECT_EQ(result.status, KltStatus::kTracked);
    EXPECT_NEAR(result.pixel.x(), feature.x() + shift.x(), 0.1);
    EXPECT_NEAR(result.pixel.y(), feature.y() + shift.y(), 0.1);
}

TEST(KltTrackerTest, StillTracksAnInteriorFeatureThroughEveryLevel) {
    const Eigen::Vector2d shift{5.0, 3.0};
    const auto from = build_pyramid(make_image(kWidth, kHeight));
    const auto to = build_pyramid(make_image(kWidth, kHeight, shift));
    ASSERT_TRUE(from.has_value());
    ASSERT_TRUE(to.has_value());

    const Eigen::Vector2d feature{400.0, 240.0};
    const KltResult result = track_feature(*from, *to, feature, feature);

    EXPECT_EQ(result.status, KltStatus::kTracked);
    EXPECT_NEAR(result.pixel.x(), feature.x() + shift.x(), 0.1);
    EXPECT_NEAR(result.pixel.y(), feature.y() + shift.y(), 0.1);
}

TEST(KltTrackerTest, RejectsAFeatureThatDoesNotFitEvenAtTheFinestLevel) {
    const auto from = build_pyramid(make_image(kWidth, kHeight));
    const auto to = build_pyramid(make_image(kWidth, kHeight));
    ASSERT_TRUE(from.has_value());
    ASSERT_TRUE(to.has_value());

    // Inside the image but closer to the edge than the patch footprint needs.
    const Eigen::Vector2d feature{5.0, 240.0};
    const KltResult result = track_feature(*from, *to, feature, feature);

    EXPECT_EQ(result.status, KltStatus::kOutOfBounds);
}

TEST(KltTrackerTest, RecoversTheSameShiftFromEveryCornerOfTheFrame) {
    const Eigen::Vector2d shift{2.0, 2.0};
    const auto from = build_pyramid(make_image(kWidth, kHeight));
    const auto to = build_pyramid(make_image(kWidth, kHeight, shift));
    ASSERT_TRUE(from.has_value());
    ASSERT_TRUE(to.has_value());

    // All four lie outside the level-3 box [96, 656) x [96, 384) and so used to
    // fail out of bounds; all four fit level 2 with room for the iteration to
    // move. A feature at the very edge of a level's validity band -- (700, 430)
    // sits at level-2 centre 175.0 against a limit of 176.0 -- still tracks to
    // about 0.1 px but can miss the 0.01 px convergence test, because the patch
    // only just fits and any drift leaves the image. Choosing the deepest
    // fitting level maximizes convergence range at the cost of that margin.
    for (const Eigen::Vector2d& feature : {Eigen::Vector2d{40.0, 40.0}, Eigen::Vector2d{700.0, 40.0},
                                           Eigen::Vector2d{40.0, 430.0}, Eigen::Vector2d{680.0, 410.0}}) {
        const KltResult result = track_feature(*from, *to, feature, feature);
        EXPECT_EQ(result.status, KltStatus::kTracked) << "at " << feature.transpose();
        EXPECT_NEAR(result.pixel.x(), feature.x() + shift.x(), 0.1) << "at " << feature.transpose();
        EXPECT_NEAR(result.pixel.y(), feature.y() + shift.y(), 0.1) << "at " << feature.transpose();
    }
}
