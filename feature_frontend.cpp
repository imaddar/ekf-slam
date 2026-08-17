#include "feature_frontend.hpp"

#include <algorithm>

namespace {

using Clock = std::chrono::steady_clock;

// Adds its own lifetime to an accumulator so a stage stays timed across the
// early returns inside `process`.
class ScopedTimer {
public:
    explicit ScopedTimer(std::chrono::nanoseconds& sink) : sink_(sink), start_(Clock::now()) {}
    ~ScopedTimer() { sink_ += Clock::now() - start_; }
private:
    std::chrono::nanoseconds& sink_;
    Clock::time_point start_;
};

}  // namespace

ParseResult<FeatureFrontend> FeatureFrontend::create(const StereoRectification& rectification, const FrontendOptions& options) {
    if (!rectification.maps || options.max_mapped_landmarks == 0 || options.min_track_age_for_mapping < 1) return std::unexpected("feature frontend: expected rectification and positive capacities");
    FeatureFrontend frontend;
    frontend.rectification_ = rectification;
    frontend.options_ = options;
    return frontend;
}

ParseResult<FrontendFrame> FeatureFrontend::process(const GrayImage& raw0, const GrayImage& raw1, TimestampNs timestamp) {
    ScopedTimer total_timer{timings_.total}; ++timings_.frames;
    const auto rectify_start = Clock::now();
    auto image0 = rectify_cam0(rectification_, raw0); auto image1 = rectify_cam1(rectification_, raw1);
    timings_.rectify += Clock::now() - rectify_start;
    if (!image0) return std::unexpected(image0.error()); if (!image1) return std::unexpected(image1.error());
    const auto pyramid_start = Clock::now();
    auto current0 = build_pyramid(*image0); auto current1 = build_pyramid(*image1);
    timings_.pyramid += Clock::now() - pyramid_start;
    if (!current0) return std::unexpected(current0.error()); if (!current1) return std::unexpected(current1.error());
    FrontendFrame frame{.timestamp = timestamp}; std::vector<LandmarkId> lost;
    for (const auto& [id, track] : tracks_) {
        if (track.state == TrackState::kMapped && track.gated >= options_.max_consecutive_gated) lost.push_back(id);
    }
    for (LandmarkId id : lost) { frame.dead_landmarks.push_back(id); tracks_.erase(id); }
    lost.clear();
    if (previous_cam0_) for (auto& [id, track] : tracks_) {
        ++timings_.temporal_calls;
        const auto temporal_start = Clock::now();
        const KltResult temporal = track_feature(*previous_cam0_, *current0, track.pixel, track.pixel, options_.temporal);
        const auto forward_backward_start = Clock::now();
        const bool consistent = passes_forward_backward(*previous_cam0_, *current0, track.pixel, temporal, options_.temporal);
        const auto stereo_start = Clock::now();
        timings_.temporal_klt += forward_backward_start - temporal_start;
        timings_.forward_backward += stereo_start - forward_backward_start;
        if (!consistent) { lost.push_back(id); continue; }
        const StereoMatch stereo = match_stereo(*current0, *current1, temporal.pixel, track.disparity, options_.stereo);
        timings_.stereo_tracked += Clock::now() - stereo_start;
        if (!stereo.valid) { lost.push_back(id); continue; }
        track.pixel = stereo.pixel_cam0; track.pixel_cam1 = stereo.pixel_cam1; track.disparity = stereo.disparity; ++track.age;
        frame.stereo_row_residuals.push_back(stereo.pixel_cam0.y() - stereo.pixel_cam1.y());
    }
    for (LandmarkId id : lost) { if (tracks_.at(id).state == TrackState::kMapped) frame.dead_landmarks.push_back(id); tracks_.erase(id); }
    std::vector<Eigen::Vector2d> occupied; for (const auto& [_, track] : tracks_) occupied.push_back(track.pixel);
    const auto detect_start = Clock::now();
    const std::vector<Corner> corners = detect_corners(*image0, occupied, options_.detector);
    timings_.detect += Clock::now() - detect_start;
    timings_.stereo_new_calls += corners.size();
    for (const Corner& corner : corners) {
        const auto match_start = Clock::now();
        const StereoMatch stereo = match_stereo(*current0, *current1, corner.pixel, 0.0, options_.stereo);
        timings_.stereo_new += Clock::now() - match_start;
        if (!stereo.valid) continue;
        frame.stereo_row_residuals.push_back(stereo.pixel_cam0.y() - stereo.pixel_cam1.y());
        tracks_.emplace(next_id_, Track{.id = next_id_, .pixel = corner.pixel, .pixel_cam1 = stereo.pixel_cam1, .disparity = stereo.disparity}); ++next_id_;
    }
    std::size_t mapped = 0;
    for (const auto& [id, track] : tracks_) {
        const StereoObservation observation{.id = id, .pixel_cam0 = track.pixel, .pixel_cam1 = track.pixel_cam1};
        if (track.state == TrackState::kMapped) { if (mapped++ < options_.max_mapped_landmarks) frame.mapped_observations.push_back(observation); }
        else if (track.age >= options_.min_track_age_for_mapping) frame.birth_candidates.push_back(observation);
    }
    frame.active_track_count = static_cast<int>(tracks_.size()); previous_cam0_ = std::move(*current0); return frame;
}

void FeatureFrontend::report_augmentation(std::span<const LandmarkId> ids) { for (LandmarkId id : ids) if (auto it = tracks_.find(id); it != tracks_.end()) it->second.state = TrackState::kMapped; }
void FeatureFrontend::report_outcomes(std::span<const LandmarkUpdateDiagnostics> outcomes) { for (const auto& outcome : outcomes) if (auto it = tracks_.find(outcome.id); it != tracks_.end()) it->second.gated = outcome.outcome == ObservationOutcome::kGated ? it->second.gated + 1 : 0; }
