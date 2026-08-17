#pragma once

#include "image.hpp"
#include "types.hpp"

#include <memory>

struct StereoRectification {
    CameraCalibration cam0_rectified;
    CameraCalibration cam1_rectified;
    double baseline_meters = 0.0;

    struct Maps;
    std::shared_ptr<const Maps> maps;
};

ParseResult<StereoRectification> make_stereo_rectification(
    const CameraCalibration& cam0_raw,
    const CameraCalibration& cam1_raw);
ParseResult<GrayImage> rectify_cam0(const StereoRectification& rectification, const GrayImage& raw);
ParseResult<GrayImage> rectify_cam1(const StereoRectification& rectification, const GrayImage& raw);
