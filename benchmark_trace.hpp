#pragma once

#include "evaluation.hpp"
#include "measurement_update.hpp"

#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string_view>

// CSV writer for the MH_01 benchmark. It records only the 15-state robot
// covariance; the joint landmark covariance is intentionally excluded because
// it grows with the map and is not needed for trajectory visualizations.
class BenchmarkTraceWriter {
public:
    static ParseResult<BenchmarkTraceWriter> create(const std::filesystem::path& output_directory,
        std::string_view git_revision, std::size_t landmark_budget, double pixel_sigma);

    ParseResult<void> write_imu(TimestampNs timestamp, double timestep_seconds, const NominalState& state,
        const ImuStateCovariance& covariance, const std::optional<GroundTruthState>& truth);
    ParseResult<void> write_camera(TimestampNs timestamp, const NominalState& prior, const ImuStateCovariance& prior_covariance,
        const NominalState& posterior, const ImuStateCovariance& posterior_covariance,
        int active_tracks, int applied, int gated, int augmented, const std::optional<GroundTruthState>& truth);
    ParseResult<void> write_observations(TimestampNs timestamp, std::span<const StereoObservation> observations,
        std::span<const LandmarkUpdateDiagnostics> diagnostics);

private:
    BenchmarkTraceWriter() = default;

    std::unique_ptr<std::ofstream> owned_imu_stream_;
    std::unique_ptr<std::ofstream> owned_camera_stream_;
    std::unique_ptr<std::ofstream> owned_observation_stream_;
};
