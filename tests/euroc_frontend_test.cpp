#include "feature_frontend.hpp"
#include "image_io.hpp"
#include "parser.hpp"
#include "landmark_augmentation.hpp"
#include "propagation.hpp"
#include "slam_state.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <algorithm>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <iostream>

TEST(EurocFrontendTest, RectifiesAndMaintainsStereoTracksOnMh01) {
    const auto dataset = parse_dataset(std::filesystem::path{EKF_SLAM_SOURCE_DIR} / "datasets/machine_hall/MH_01_easy");
    ASSERT_TRUE(dataset) << dataset.error();
    const auto rectification = make_stereo_rectification(dataset->cam0_calibration, dataset->cam1_calibration);
    ASSERT_TRUE(rectification) << rectification.error();
    ASSERT_GT(rectification->baseline_meters, 0.1);
    const auto frontend_result = FeatureFrontend::create(*rectification);
    ASSERT_TRUE(frontend_result) << frontend_result.error();
    FeatureFrontend frontend = *frontend_result;

    int peak_tracks = 0;
    int birth_candidates = 0;
    for (std::size_t index = 0; index < 8; ++index) {
        const StereoPair& pair = dataset->stereo_pairs.at(index);
        const auto cam0 = load_grayscale_png(pair.cam0_image_path);
        const auto cam1 = load_grayscale_png(pair.cam1_image_path);
        ASSERT_TRUE(cam0) << cam0.error();
        ASSERT_TRUE(cam1) << cam1.error();
        const auto frame = frontend.process(*cam0, *cam1, pair.timestamp);
        ASSERT_TRUE(frame) << frame.error();
        peak_tracks = std::max(peak_tracks, frame->active_track_count);
        birth_candidates += static_cast<int>(frame->birth_candidates.size());
    }
    EXPECT_GT(peak_tracks, 20);
    EXPECT_GT(birth_candidates, 0);
}

TEST(EurocFrontendTest, HoldsTheTrackPoolWithinItsCapAndRefillsAfterDraining) {
    const auto dataset = parse_dataset(std::filesystem::path{EKF_SLAM_SOURCE_DIR} / "datasets/machine_hall/MH_01_easy");
    ASSERT_TRUE(dataset) << dataset.error();
    const auto rectification = make_stereo_rectification(dataset->cam0_calibration, dataset->cam1_calibration);
    ASSERT_TRUE(rectification) << rectification.error();
    // Tight thresholds so both the cap and the refill are exercised within a
    // short prefix rather than only under a long run's attrition.
    FrontendOptions options;
    options.max_mapped_landmarks = 40;
    options.max_tracks = 60;
    options.redetect_below = 40;
    const auto frontend_result = FeatureFrontend::create(*rectification, options);
    ASSERT_TRUE(frontend_result) << frontend_result.error();
    FeatureFrontend frontend = *frontend_result;

    int peak_tracks = 0, minimum_after_first_fill = std::numeric_limits<int>::max();
    for (std::size_t index = 0; index < 30; ++index) {
        const StereoPair& pair = dataset->stereo_pairs.at(index);
        const auto cam0 = load_grayscale_png(pair.cam0_image_path);
        const auto cam1 = load_grayscale_png(pair.cam1_image_path);
        ASSERT_TRUE(cam0) << cam0.error();
        ASSERT_TRUE(cam1) << cam1.error();
        const auto frame = frontend.process(*cam0, *cam1, pair.timestamp);
        ASSERT_TRUE(frame) << frame.error();
        EXPECT_LE(frame->active_track_count, static_cast<int>(options.max_tracks)) << "at frame " << index;
        peak_tracks = std::max(peak_tracks, frame->active_track_count);
        if (index > 0) minimum_after_first_fill = std::min(minimum_after_first_fill, frame->active_track_count);
    }
    // The pool fills to the cap, and attrition never strands it below the
    // hysteresis threshold for long, because dropping under it triggers a refill.
    EXPECT_EQ(peak_tracks, static_cast<int>(options.max_tracks));
    EXPECT_GT(minimum_after_first_fill, 0);
}

TEST(EurocFrontendTest, RejectsATrackPoolConfigurationItCannotHonor) {
    const auto dataset = parse_dataset(std::filesystem::path{EKF_SLAM_SOURCE_DIR} / "datasets/machine_hall/MH_01_easy");
    ASSERT_TRUE(dataset) << dataset.error();
    const auto rectification = make_stereo_rectification(dataset->cam0_calibration, dataset->cam1_calibration);
    ASSERT_TRUE(rectification) << rectification.error();

    FrontendOptions fewer_tracks_than_landmarks;
    fewer_tracks_than_landmarks.max_mapped_landmarks = 100;
    fewer_tracks_than_landmarks.max_tracks = 50;
    EXPECT_FALSE(FeatureFrontend::create(*rectification, fewer_tracks_than_landmarks));

    FrontendOptions threshold_above_cap;
    threshold_above_cap.max_tracks = 200;
    threshold_above_cap.redetect_below = 201;
    EXPECT_FALSE(FeatureFrontend::create(*rectification, threshold_above_cap));

    FrontendOptions never_detects;
    never_detects.redetect_below = 0;
    EXPECT_FALSE(FeatureFrontend::create(*rectification, never_detects));
}

TEST(EurocFrontendTest, PreliminaryMh01ClosedLoopRunAppliesRealImageMeasurements) {
    const auto dataset = parse_dataset(std::filesystem::path{EKF_SLAM_SOURCE_DIR} / "datasets/machine_hall/MH_01_easy");
    ASSERT_TRUE(dataset) << dataset.error();
    const auto rectification = make_stereo_rectification(dataset->cam0_calibration, dataset->cam1_calibration);
    ASSERT_TRUE(rectification) << rectification.error();
    auto frontend_result = FeatureFrontend::create(*rectification);
    ASSERT_TRUE(frontend_result) << frontend_result.error();
    FeatureFrontend frontend = std::move(*frontend_result);
    const GroundTruthState& initial = dataset->ground_truth_states.front();
    auto state_result = SlamState::create(100, NominalState{
        .position = initial.position, .velocity = initial.velocity, .orientation = Sophus::SO3d{initial.orientation},
        .accelerometer_bias = initial.accelerometer_bias, .gyroscope_bias = initial.gyroscope_bias,
    }, 1e-4 * ImuStateCovariance::Identity());
    ASSERT_TRUE(state_result) << state_result.error();
    SlamState state = std::move(*state_result);
    std::size_t imu_index = 0;
    TimestampNs last_imu_timestamp = initial.timestamp;
    int applied = 0, augmented = 0, gated = 0, peak_tracks = 0;
    std::vector<double> row_residuals;
    double pixel_sigma = 0.5;
    if (const char* value = std::getenv("EKF_SLAM_FRONTEND_PIXEL_SIGMA")) {
        pixel_sigma = std::strtod(value, nullptr);
    }
    ASSERT_GT(pixel_sigma, 0.0);
    const Eigen::Matrix4d pixel_covariance = pixel_sigma * pixel_sigma * Eigen::Matrix4d::Identity();
    std::size_t frame_limit = 30;
    if (const char* value = std::getenv("EKF_SLAM_FRONTEND_FRAMES")) {
        frame_limit = static_cast<std::size_t>(std::strtoull(value, nullptr, 10));
    }
    const std::size_t kFrames = std::min(frame_limit, dataset->stereo_pairs.size());
    ASSERT_GT(kFrames, 0U);
    std::chrono::nanoseconds tracking_time{}, filter_time{};
    for (std::size_t frame_index = 0; frame_index < kFrames; ++frame_index) {
        const StereoPair& pair = dataset->stereo_pairs.at(frame_index);
        while (imu_index < dataset->imu_measurements.size() && dataset->imu_measurements[imu_index].timestamp <= pair.timestamp) {
            const ImuMeasurement& measurement = dataset->imu_measurements[imu_index++];
            if (measurement.timestamp <= last_imu_timestamp) continue;
            const double dt = static_cast<double>(measurement.timestamp - last_imu_timestamp) * 1e-9;
            ASSERT_TRUE(propagate_slam(state, measurement, dataset->imu_calibration, dt));
            last_imu_timestamp = measurement.timestamp;
        }
        const auto cam0 = load_grayscale_png(pair.cam0_image_path); const auto cam1 = load_grayscale_png(pair.cam1_image_path);
        ASSERT_TRUE(cam0) << cam0.error(); ASSERT_TRUE(cam1) << cam1.error();
        const auto frontend_start = std::chrono::steady_clock::now();
        const auto frame = frontend.process(*cam0, *cam1, pair.timestamp);
        tracking_time += std::chrono::steady_clock::now() - frontend_start;
        ASSERT_TRUE(frame) << frame.error(); peak_tracks = std::max(peak_tracks, frame->active_track_count);
        row_residuals.insert(row_residuals.end(), frame->stereo_row_residuals.begin(), frame->stereo_row_residuals.end());
        const auto update_start = std::chrono::steady_clock::now();
        const auto update = update_stereo_frame(state, frame->mapped_observations, rectification->cam0_rectified,
            rectification->cam1_rectified, pixel_covariance);
        filter_time += std::chrono::steady_clock::now() - update_start;
        ASSERT_TRUE(update) << update.error(); applied += update->applied_count; gated += update->gated_count;
        frontend.report_outcomes(update->diagnostics);
        std::vector<LandmarkId> births;
        for (const StereoObservation& candidate : frame->birth_candidates) {
            if (state.active_landmarks() >= state.max_landmarks()) break;
            ASSERT_TRUE(augment_landmark(state, candidate.id, candidate.pixel_cam0, candidate.pixel_cam1,
                rectification->cam0_rectified, rectification->cam1_rectified, pixel_covariance, 1.0));
            births.push_back(candidate.id); ++augmented;
        }
        frontend.report_augmentation(births);
        ASSERT_TRUE(state.remove_landmarks(frame->dead_landmarks));
    }
    const TimestampNs final_timestamp = dataset->stereo_pairs.at(kFrames - 1).timestamp;
    const auto truth = std::min_element(dataset->ground_truth_states.begin(), dataset->ground_truth_states.end(),
        [final_timestamp](const GroundTruthState& lhs, const GroundTruthState& rhs) {
            return std::llabs(static_cast<long long>(lhs.timestamp) - static_cast<long long>(final_timestamp))
                < std::llabs(static_cast<long long>(rhs.timestamp) - static_cast<long long>(final_timestamp));
        });
    const double position_error = (state.robot.position - truth->position).norm();
    const double row_mean = std::accumulate(row_residuals.begin(), row_residuals.end(), 0.0) / row_residuals.size();
    double row_square_error = 0.0; for (double residual : row_residuals) row_square_error += (residual - row_mean) * (residual - row_mean);
    const double row_standard_deviation = std::sqrt(row_square_error / row_residuals.size());
    std::cout << "mh01_frontend_preliminary frames=" << kFrames << " peak_tracks=" << peak_tracks
              << " augmented=" << augmented << " applied=" << applied << " gated=" << gated
              << " active_landmarks=" << state.active_landmarks() << " final_position_error_m=" << position_error
              << " pixel_sigma=" << pixel_sigma
              << " row_residual_mean_px=" << row_mean << " row_residual_stddev_px=" << row_standard_deviation
              << " frontend_ms_per_frame=" << static_cast<double>(tracking_time.count()) / kFrames / 1e6
              << " update_ms_per_frame=" << static_cast<double>(filter_time.count()) / kFrames / 1e6 << '\n';
    EXPECT_GT(peak_tracks, 20); EXPECT_GT(augmented, 0); EXPECT_GT(applied, 0);
}
