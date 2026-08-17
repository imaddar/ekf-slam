#include "rectification.hpp"

#include "stereo_geometry.hpp"

#include <format>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

struct StereoRectification::Maps {
    cv::Mat cam0_x;
    cv::Mat cam0_y;
    cv::Mat cam1_x;
    cv::Mat cam1_y;
};

namespace {

cv::Mat camera_matrix(const CameraCalibration& camera) {
    cv::Mat result = cv::Mat::eye(3, 3, CV_64F);
    result.at<double>(0, 0) = camera.intrinsics.x(); result.at<double>(1, 1) = camera.intrinsics.y();
    result.at<double>(0, 2) = camera.intrinsics.z(); result.at<double>(1, 2) = camera.intrinsics.w();
    return result;
}

cv::Mat distortion(const CameraCalibration& camera) {
    cv::Mat result(1, 4, CV_64F);
    for (int index = 0; index < 4; ++index) result.at<double>(index) = camera.distortion_coefficients[index];
    return result;
}

Eigen::Matrix4d rectified_body_from_camera(
    const Eigen::Matrix4d& raw_body_from_camera, const cv::Mat& rectification_rotation) {
    Eigen::Matrix4d result = raw_body_from_camera;
    Eigen::Matrix3d rotation;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            rotation(row, column) = rectification_rotation.at<double>(row, column);
        }
    }
    result.topLeftCorner<3, 3>() = raw_body_from_camera.topLeftCorner<3, 3>() * rotation.transpose();
    return result;
}

ParseResult<GrayImage> remap_image(const GrayImage& raw, const cv::Mat& x, const cv::Mat& y) {
    if (raw.width != x.cols || raw.height != x.rows || raw.data.size() != static_cast<std::size_t>(raw.width) * raw.height) {
        return std::unexpected("rectification image: expected calibration resolution and dense grayscale storage");
    }
    const cv::Mat source(raw.height, raw.width, CV_8UC1, const_cast<std::uint8_t*>(raw.data.data()));
    cv::Mat output;
    cv::remap(source, output, x, y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    GrayImage result{.width = output.cols, .height = output.rows};
    result.data.reserve(static_cast<std::size_t>(result.width) * result.height);
    for (int row = 0; row < result.height; ++row) {
        const auto* begin = output.ptr<std::uint8_t>(row);
        result.data.insert(result.data.end(), begin, begin + result.width);
    }
    return result;
}

}  // namespace

ParseResult<StereoRectification> make_stereo_rectification(
    const CameraCalibration& cam0_raw, const CameraCalibration& cam1_raw) {
    if (cam0_raw.resolution != cam1_raw.resolution || cam0_raw.resolution.x() <= 0 || cam0_raw.resolution.y() <= 0) {
        return std::unexpected("stereo rectification: expected matching positive camera resolutions");
    }
    if (!cam0_raw.intrinsics.allFinite() || !cam1_raw.intrinsics.allFinite()
        || !cam0_raw.distortion_coefficients.allFinite() || !cam1_raw.distortion_coefficients.allFinite()) {
        return std::unexpected("stereo rectification: expected finite intrinsics and distortion coefficients");
    }
    const auto body0 = body_from_camera_transform(cam0_raw, "cam0");
    const auto body1 = body_from_camera_transform(cam1_raw, "cam1");
    if (!body0) return std::unexpected(body0.error());
    if (!body1) return std::unexpected(body1.error());

    // OpenCV receives the transform from its first (cam0) coordinates into its
    // second (cam1) coordinates: T_C1C0 = T_BC1^-1 T_BC0.
    const Eigen::Matrix4d cam1_from_cam0 = body1->inverse() * *body0;
    cv::Mat rotation(3, 3, CV_64F);
    cv::Mat translation(3, 1, CV_64F);
    for (int row = 0; row < 3; ++row) {
        translation.at<double>(row) = cam1_from_cam0(row, 3);
        for (int column = 0; column < 3; ++column) rotation.at<double>(row, column) = cam1_from_cam0(row, column);
    }

    cv::Mat r0, r1, p0, p1, q;
    try {
        cv::stereoRectify(camera_matrix(cam0_raw), distortion(cam0_raw),
            camera_matrix(cam1_raw), distortion(cam1_raw),
            cv::Size(cam0_raw.resolution.x(), cam0_raw.resolution.y()), rotation, translation,
            r0, r1, p0, p1, q, cv::CALIB_ZERO_DISPARITY, 0.0);
    } catch (const cv::Exception& error) {
        return std::unexpected(std::format("stereo rectification: stereoRectify failed: {}", error.what()));
    }
    if (r0.empty() || r1.empty() || p0.empty() || p1.empty()) {
        return std::unexpected("stereo rectification: OpenCV failed to derive rectified cameras");
    }

    StereoRectification result{.cam0_rectified = cam0_raw, .cam1_rectified = cam1_raw};
    result.cam0_rectified.t_bs = rectified_body_from_camera(*body0, r0);
    result.cam1_rectified.t_bs = rectified_body_from_camera(*body1, r1);
    result.cam0_rectified.intrinsics = Eigen::Vector4d{p0.at<double>(0, 0), p0.at<double>(1, 1), p0.at<double>(0, 2), p0.at<double>(1, 2)};
    result.cam1_rectified.intrinsics = Eigen::Vector4d{p1.at<double>(0, 0), p1.at<double>(1, 1), p1.at<double>(0, 2), p1.at<double>(1, 2)};
    result.cam0_rectified.distortion_coefficients.setZero();
    result.cam1_rectified.distortion_coefficients.setZero();

    const Eigen::Vector3d baseline = result.cam0_rectified.t_bs.topLeftCorner<3, 3>().transpose()
        * (result.cam1_rectified.t_bs.topRightCorner<3, 1>() - result.cam0_rectified.t_bs.topRightCorner<3, 1>());
    if (baseline.x() <= 0.0 || std::abs(baseline.y()) > 1e-8 || std::abs(baseline.z()) > 1e-8) {
        return std::unexpected(std::format("stereo rectification: expected positive horizontal baseline, found [{}, {}, {}]", baseline.x(), baseline.y(), baseline.z()));
    }
    result.baseline_meters = baseline.x();
    auto maps = std::make_shared<StereoRectification::Maps>();
    try {
        cv::initUndistortRectifyMap(camera_matrix(cam0_raw), distortion(cam0_raw), r0, p0.colRange(0, 3).clone(),
            cv::Size(cam0_raw.resolution.x(), cam0_raw.resolution.y()), CV_32FC1, maps->cam0_x, maps->cam0_y);
        cv::initUndistortRectifyMap(camera_matrix(cam1_raw), distortion(cam1_raw), r1, p1.colRange(0, 3).clone(),
            cv::Size(cam1_raw.resolution.x(), cam1_raw.resolution.y()), CV_32FC1, maps->cam1_x, maps->cam1_y);
    } catch (const cv::Exception& error) {
        return std::unexpected(std::format("stereo rectification: initUndistortRectifyMap failed: {}", error.what()));
    }
    result.maps = std::move(maps);
    return result;
}

ParseResult<GrayImage> rectify_cam0(const StereoRectification& rectification, const GrayImage& raw) {
    if (!rectification.maps) return std::unexpected("rectification: expected initialized maps");
    return remap_image(raw, rectification.maps->cam0_x, rectification.maps->cam0_y);
}

ParseResult<GrayImage> rectify_cam1(const StereoRectification& rectification, const GrayImage& raw) {
    if (!rectification.maps) return std::unexpected("rectification: expected initialized maps");
    return remap_image(raw, rectification.maps->cam1_x, rectification.maps->cam1_y);
}
