#include "stereo_matcher.hpp"

StereoMatch match_stereo(const ImagePyramid& cam0, const ImagePyramid& cam1, const Eigen::Vector2d& pixel_cam0,
                         double disparity_prior_px, const StereoMatchOptions& options) {
    // Rectification predicts equal rows; leave v free here so its residual is
    // an independent calibration/noise diagnostic, then gate it explicitly.
    KltOptions klt = options.klt; klt.constrain_to_row = false;
    const Eigen::Vector2d initial{pixel_cam0.x() - disparity_prior_px, pixel_cam0.y()};
    const KltResult right = track_feature(cam0, cam1, pixel_cam0, initial, klt);
    const double disparity = pixel_cam0.x() - right.pixel.x();
    if (right.status != KltStatus::kTracked || right.residual > klt.max_residual_per_pixel
        || std::abs(pixel_cam0.y() - right.pixel.y()) > options.max_row_residual_px
        || disparity < options.min_disparity_px || disparity > options.max_disparity_px) return {};
    return {.pixel_cam0 = pixel_cam0, .pixel_cam1 = right.pixel, .disparity = disparity, .valid = true};
}
