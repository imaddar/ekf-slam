#pragma once

#include "corner_detector.hpp"
#include "measurement_update.hpp"
#include "rectification.hpp"
#include "stereo_matcher.hpp"

#include <chrono>
#include <unordered_map>

enum class TrackState { kCandidate, kMapped };
// `max_tracks` bounds per-frame tracking cost, which is paid per track per
// frame whether or not the filter can use the track. It needs headroom above
// `max_mapped_landmarks` so mature replacements are available as tracks die.
//
// `redetect_below` adds hysteresis: detection is a fixed whole-frame cost that
// does not scale with how many features it is asked for, so running it every
// frame to top up a handful of dead tracks pays that cost for a trickle of
// features. Letting the pool drain to `redetect_below` and then refilling to
// `max_tracks` amortizes detection over the frames in between, without holding
// fewer tracks on average.
//
// `correlation_inflation` scales up a repeat observation's covariance as a
// track ages without moving or without a fresh update, approximating the
// temporal correlation between successive observations of the same landmark
// (see BENCHMARKS.md's innovation-autocorrelation finding). `0.0` disables
// it. `decorrelation_parallax_px` and `decorrelation_interval_seconds` are
// the pixel and time scales over which a track is treated as decorrelated
// again.
struct FrontendOptions { DetectorOptions detector{}; KltOptions temporal{}; StereoMatchOptions stereo{}; std::size_t max_mapped_landmarks = 100, max_tracks = 300, redetect_below = 200; int min_track_age_for_mapping = 2, max_consecutive_gated = 3; double correlation_inflation = 0.0, decorrelation_parallax_px = 3.0, decorrelation_interval_seconds = 0.15; };
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
    //
    // `last_update_pixel`/`last_update_timestamp`/`has_last_update` record
    // where and when a track was last applied to the filter, so
    // `correlation_inflation` can scale down once a track has moved or aged
    // past the decorrelation thresholds.
    struct Track { LandmarkId id; Eigen::Vector2d pixel, pixel_cam1; double disparity; Eigen::Vector2d last_update_pixel = Eigen::Vector2d::Zero(); TimestampNs last_update_timestamp = 0; int age = 0, gated = 0; bool has_last_update = false; TrackState state = TrackState::kCandidate; };
    StereoRectification rectification_;
    FrontendOptions options_;
    std::unordered_map<LandmarkId, Track> tracks_;
    std::optional<ImagePyramid> previous_cam0_;
    LandmarkId next_id_ = 1;
    TimestampNs current_timestamp_ = 0;
    FrontendStageTimings timings_;
};
