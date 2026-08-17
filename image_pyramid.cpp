#include "image_pyramid.hpp"

#include <algorithm>

ParseResult<ImagePyramid> build_pyramid(const GrayImage& image, int levels) {
    if (levels <= 0 || image.width <= 0 || image.height <= 0
        || image.data.size() != static_cast<std::size_t>(image.width) * image.height) {
        return std::unexpected("image pyramid: expected positive levels and dense non-empty image");
    }
    ImagePyramid result{{image}};
    for (int level = 1; level < levels; ++level) {
        const GrayImage& source = result.levels.back();
        GrayImage next{.width = (source.width + 1) / 2, .height = (source.height + 1) / 2};
        next.data.resize(static_cast<std::size_t>(next.width) * next.height);
        const auto sample = [&source](int x, int y) {
            x = std::clamp(x, 0, source.width - 1); y = std::clamp(y, 0, source.height - 1);
            return static_cast<int>(source.data[static_cast<std::size_t>(y) * source.width + x]);
        };
        for (int y = 0; y < next.height; ++y) for (int x = 0; x < next.width; ++x) {
            int total = 0;
            constexpr int weights[5]{1, 4, 6, 4, 1};
            for (int dy = -2; dy <= 2; ++dy) for (int dx = -2; dx <= 2; ++dx)
                total += weights[dx + 2] * weights[dy + 2] * sample(2 * x + dx, 2 * y + dy);
            next.data[static_cast<std::size_t>(y) * next.width + x] = static_cast<std::uint8_t>((total + 128) / 256);
        }
        result.levels.push_back(std::move(next));
    }
    return result;
}
