#pragma once

#include "corner_detector.hpp"
#include "measurement_update.hpp"
#include "rectification.hpp"
#include "stereo_matcher.hpp"

#include <chrono>
#include <unordered_map>

enum class TrackState { kCandidate, kMapped };
struct FrontendOptions { DetectorOptions detector{}; KltOptions temporal{}; StereoMatchOptions stereo{}; std::size_t max_mapped_landmarks = 100; int min_track_age_for_mapping = 2, max_consecutive_gated = 3; };
struct FrontendFrame { TimestampNs timestamp; std::vector<StereoObservation> mapped_observations, birth_candidates; std::vector<LandmarkId> dead_landmarks; std::vector<double> stereo_row_residuals; int active_track_count = 0; };

// Wall-clock totals per frontend stage, accumulated across every processed
// frame. `total` covers all of `process`, so `total` minus the stages is
// bookkeeping. Callers divide by `frames` for a per-frame cost.
struct FrontendStageTimings {
    std::chrono::nanoseconds rectify{}, pyramid{}, temporal_klt{}, forward_backward{},
        stereo_tracked{}, detect{}, stereo_new{}, total{};
    // Call counts separate a fixed per-frame cost from a per-feature one.
    std::size_t temporal_calls = 0, stereo_new_calls = 0;
    int frames = 0;
};

class FeatureFrontend {
public:
    static ParseResult<FeatureFrontend> create(const StereoRectification&, const FrontendOptions& = {});
    ParseResult<FrontendFrame> process(const GrayImage& cam0_raw, const GrayImage& cam1_raw, TimestampNs timestamp);
    void report_outcomes(std::span<const LandmarkUpdateDiagnostics> outcomes);
    void report_augmentation(std::span<const LandmarkId> ids);
    const FrontendStageTimings& stage_timings() const { return timings_; }
private:
    // `pixel_cam1` is the matched cam1 location, not a copy of `pixel` with the
    // disparity subtracted: its row is an independent observation of the same
    // rectified row, and reconstructing it would hand the filter the cam0 row
    // twice under an R that assumes four independent components.
    struct Track { LandmarkId id; Eigen::Vector2d pixel, pixel_cam1; double disparity; int age = 0, gated = 0; TrackState state = TrackState::kCandidate; };
    StereoRectification rectification_;
    FrontendOptions options_;
    std::unordered_map<LandmarkId, Track> tracks_;
    std::optional<ImagePyramid> previous_cam0_;
    LandmarkId next_id_ = 1;
    FrontendStageTimings timings_;
};
