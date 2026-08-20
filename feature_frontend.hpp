#pragma once

#include "corner_detector.hpp"
#include "measurement_update.hpp"
#include "rectification.hpp"
#include "stereo_matcher.hpp"

#include <unordered_map>

enum class TrackState { kCandidate, kMapped };
struct FrontendOptions { DetectorOptions detector{}; KltOptions temporal{}; StereoMatchOptions stereo{}; std::size_t max_mapped_landmarks = 100; int min_track_age_for_mapping = 2, max_consecutive_gated = 3; double correlation_inflation = 0.0, decorrelation_parallax_px = 3.0, decorrelation_interval_seconds = 0.15; };
struct FrontendFrame { TimestampNs timestamp; std::vector<StereoObservation> mapped_observations, birth_candidates; std::vector<LandmarkId> dead_landmarks; std::vector<double> stereo_row_residuals; int active_track_count = 0; };

class FeatureFrontend {
public:
    static ParseResult<FeatureFrontend> create(const StereoRectification&, const FrontendOptions& = {});
    ParseResult<FrontendFrame> process(const GrayImage& cam0_raw, const GrayImage& cam1_raw, TimestampNs timestamp);
    void report_outcomes(std::span<const LandmarkUpdateDiagnostics> outcomes);
    void report_augmentation(std::span<const LandmarkId> ids);
private:
    struct Track { LandmarkId id; Eigen::Vector2d pixel; double disparity; Eigen::Vector2d last_update_pixel = Eigen::Vector2d::Zero(); TimestampNs last_update_timestamp = 0; int age = 0, gated = 0; bool has_last_update = false; TrackState state = TrackState::kCandidate; };
    StereoRectification rectification_;
    FrontendOptions options_;
    std::unordered_map<LandmarkId, Track> tracks_;
    std::optional<ImagePyramid> previous_cam0_;
    LandmarkId next_id_ = 1;
    TimestampNs current_timestamp_ = 0;
};
