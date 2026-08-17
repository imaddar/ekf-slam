#include "feature_frontend.hpp"

#include <algorithm>

ParseResult<FeatureFrontend> FeatureFrontend::create(const StereoRectification& rectification, const FrontendOptions& options) {
    if (!rectification.maps || options.max_mapped_landmarks == 0 || options.min_track_age_for_mapping < 1) return std::unexpected("feature frontend: expected rectification and positive capacities");
    FeatureFrontend frontend;
    frontend.rectification_ = rectification;
    frontend.options_ = options;
    return frontend;
}

ParseResult<FrontendFrame> FeatureFrontend::process(const GrayImage& raw0, const GrayImage& raw1, TimestampNs timestamp) {
    auto image0 = rectify_cam0(rectification_, raw0); if (!image0) return std::unexpected(image0.error());
    auto image1 = rectify_cam1(rectification_, raw1); if (!image1) return std::unexpected(image1.error());
    auto current0 = build_pyramid(*image0); auto current1 = build_pyramid(*image1);
    if (!current0) return std::unexpected(current0.error()); if (!current1) return std::unexpected(current1.error());
    FrontendFrame frame{.timestamp = timestamp}; std::vector<LandmarkId> lost;
    for (const auto& [id, track] : tracks_) {
        if (track.state == TrackState::kMapped && track.gated >= options_.max_consecutive_gated) lost.push_back(id);
    }
    for (LandmarkId id : lost) { frame.dead_landmarks.push_back(id); tracks_.erase(id); }
    lost.clear();
    if (previous_cam0_) for (auto& [id, track] : tracks_) {
        const KltResult temporal = track_feature(*previous_cam0_, *current0, track.pixel, track.pixel, options_.temporal);
        if (!passes_forward_backward(*previous_cam0_, *current0, track.pixel, temporal, options_.temporal)) { lost.push_back(id); continue; }
        const StereoMatch stereo = match_stereo(*current0, *current1, temporal.pixel, track.disparity, options_.stereo);
        if (!stereo.valid) { lost.push_back(id); continue; }
        track.pixel = stereo.pixel_cam0; track.disparity = stereo.disparity; ++track.age;
        frame.stereo_row_residuals.push_back(stereo.pixel_cam0.y() - stereo.pixel_cam1.y());
    }
    for (LandmarkId id : lost) { if (tracks_.at(id).state == TrackState::kMapped) frame.dead_landmarks.push_back(id); tracks_.erase(id); }
    std::vector<Eigen::Vector2d> occupied; for (const auto& [_, track] : tracks_) occupied.push_back(track.pixel);
    for (const Corner& corner : detect_corners(*image0, occupied, options_.detector)) {
        const StereoMatch stereo = match_stereo(*current0, *current1, corner.pixel, 0.0, options_.stereo); if (!stereo.valid) continue;
        frame.stereo_row_residuals.push_back(stereo.pixel_cam0.y() - stereo.pixel_cam1.y());
        tracks_.emplace(next_id_, Track{.id = next_id_, .pixel = corner.pixel, .disparity = stereo.disparity}); ++next_id_;
    }
    std::size_t mapped = 0;
    for (const auto& [id, track] : tracks_) {
        const StereoObservation observation{.id = id, .pixel_cam0 = track.pixel, .pixel_cam1 = {track.pixel.x() - track.disparity, track.pixel.y()}};
        if (track.state == TrackState::kMapped) { if (mapped++ < options_.max_mapped_landmarks) frame.mapped_observations.push_back(observation); }
        else if (track.age >= options_.min_track_age_for_mapping) frame.birth_candidates.push_back(observation);
    }
    frame.active_track_count = static_cast<int>(tracks_.size()); previous_cam0_ = std::move(*current0); return frame;
}

void FeatureFrontend::report_augmentation(std::span<const LandmarkId> ids) { for (LandmarkId id : ids) if (auto it = tracks_.find(id); it != tracks_.end()) it->second.state = TrackState::kMapped; }
void FeatureFrontend::report_outcomes(std::span<const LandmarkUpdateDiagnostics> outcomes) { for (const auto& outcome : outcomes) if (auto it = tracks_.find(outcome.id); it != tracks_.end()) it->second.gated = outcome.outcome == ObservationOutcome::kGated ? it->second.gated + 1 : 0; }
