#include "benchmark_trace.hpp"

#include <fstream>
#include <iomanip>
#include <memory>
#include <unordered_map>

namespace {

ParseResult<void> write_state(std::ostream& stream, const NominalState& state, const ImuStateCovariance& covariance) {
    const Eigen::Quaterniond orientation{state.orientation.unit_quaternion()};
    const auto vector = [&](const Eigen::Vector3d& value) {
        stream << ',' << value.x() << ',' << value.y() << ',' << value.z();
    };
    vector(state.position);
    vector(state.velocity);
    stream << ',' << orientation.w() << ',' << orientation.x() << ',' << orientation.y() << ',' << orientation.z();
    vector(state.accelerometer_bias);
    vector(state.gyroscope_bias);
    for (int row = 0; row < kImuErrorStateSize; ++row) {
        for (int column = 0; column < kImuErrorStateSize; ++column) stream << ',' << covariance(row, column);
    }
    return stream ? ParseResult<void>{} : std::unexpected(std::string{"failed while writing state trace row"});
}

void write_state_header(std::ostream& stream, std::string_view prefix) {
    const auto field = [&](std::string_view name) { stream << ',' << prefix << '_' << name; };
    field("px"); field("py"); field("pz"); field("vx"); field("vy"); field("vz");
    field("qw"); field("qx"); field("qy"); field("qz");
    field("bax"); field("bay"); field("baz"); field("bgx"); field("bgy"); field("bgz");
    for (int row = 0; row < kImuErrorStateSize; ++row) {
        for (int column = 0; column < kImuErrorStateSize; ++column) stream << ',' << prefix << "_P" << row << column;
    }
}

void write_truth(std::ostream& stream, const std::optional<GroundTruthState>& truth) {
    if (!truth) {
        for (int index = 0; index < 10; ++index) stream << ',';
        return;
    }
    stream << ',' << truth->position.x() << ',' << truth->position.y() << ',' << truth->position.z();
    stream << ',' << truth->orientation.w() << ',' << truth->orientation.x() << ',' << truth->orientation.y() << ',' << truth->orientation.z();
    stream << ',' << truth->velocity.x() << ',' << truth->velocity.y() << ',' << truth->velocity.z();
}

void write_truth_header(std::ostream& stream) {
    stream << ",truth_px,truth_py,truth_pz,truth_qw,truth_qx,truth_qy,truth_qz,truth_vx,truth_vy,truth_vz";
}

std::string_view outcome_name(ObservationOutcome outcome) {
    switch (outcome) {
        case ObservationOutcome::kApplied: return "applied";
        case ObservationOutcome::kGated: return "gated";
        case ObservationOutcome::kUnknownLandmark: return "unknown_landmark";
        case ObservationOutcome::kBehindCamera: return "behind_camera";
    }
    return "unknown";
}

}  // namespace

ParseResult<BenchmarkTraceWriter> BenchmarkTraceWriter::create(const std::filesystem::path& output_directory,
    std::string_view git_revision, std::size_t landmark_budget, double pixel_sigma) {
    std::error_code error;
    if (!std::filesystem::create_directories(output_directory, error) && error) {
        return std::unexpected("could not create trace directory " + output_directory.string() + ": " + error.message());
    }
    BenchmarkTraceWriter writer;
    writer.owned_imu_stream_ = std::make_unique<std::ofstream>(output_directory / "imu_trace.csv");
    writer.owned_camera_stream_ = std::make_unique<std::ofstream>(output_directory / "camera_trace.csv");
    writer.owned_observation_stream_ = std::make_unique<std::ofstream>(output_directory / "observation_trace.csv");
    if (!*writer.owned_imu_stream_ || !*writer.owned_camera_stream_ || !*writer.owned_observation_stream_) {
        return std::unexpected("could not open one or more trace CSV files in " + output_directory.string());
    }
    std::ofstream metadata(output_directory / "metadata.json");
    if (!metadata) return std::unexpected("could not open trace metadata file in " + output_directory.string());
    metadata << "{\n  \"sequence\": \"MH_01_easy\",\n  \"git_revision\": \"" << git_revision
             << "\",\n  \"landmark_budget\": " << landmark_budget << ",\n  \"pixel_sigma_px\": " << pixel_sigma
             << ",\n  \"initialization\": \"first_ground_truth_state\",\n"
                "  \"camera_calibration\": \"dataset cam0/cam1, rectified internally\",\n"
                "  \"timestamp_unit\": \"ns\",\n  \"covariance\": \"15-state robot covariance, row-major\"\n}\n";
    if (!metadata) return std::unexpected("failed while writing trace metadata in " + output_directory.string());

    *writer.owned_imu_stream_ << std::setprecision(17) << "timestamp_ns,dt_seconds";
    write_state_header(*writer.owned_imu_stream_, "estimate");
    write_truth_header(*writer.owned_imu_stream_);
    *writer.owned_imu_stream_ << '\n';
    *writer.owned_camera_stream_ << std::setprecision(17) << "timestamp_ns,active_tracks,applied_count,gated_count,augmented_count";
    write_state_header(*writer.owned_camera_stream_, "prior");
    write_state_header(*writer.owned_camera_stream_, "posterior");
    write_truth_header(*writer.owned_camera_stream_);
    *writer.owned_camera_stream_ << '\n';
    *writer.owned_observation_stream_ << std::setprecision(17)
        << "timestamp_ns,landmark_id,outcome,mahalanobis_distance,observed_u0,observed_v0,observed_u1,observed_v1,"
           "predicted_u0,predicted_v0,predicted_u1,predicted_v1,innovation_u0,innovation_v0,innovation_u1,innovation_v1\n";
    return writer;
}

ParseResult<void> BenchmarkTraceWriter::write_imu(TimestampNs timestamp, double timestep_seconds, const NominalState& state,
    const ImuStateCovariance& covariance, const std::optional<GroundTruthState>& truth) {
    *owned_imu_stream_ << timestamp << ',' << timestep_seconds;
    if (const auto result = write_state(*owned_imu_stream_, state, covariance); !result) return result;
    write_truth(*owned_imu_stream_, truth);
    *owned_imu_stream_ << '\n';
    return *owned_imu_stream_ ? ParseResult<void>{} : std::unexpected(std::string{"failed while writing IMU trace row"});
}

ParseResult<void> BenchmarkTraceWriter::write_camera(TimestampNs timestamp, const NominalState& prior,
    const ImuStateCovariance& prior_covariance, const NominalState& posterior, const ImuStateCovariance& posterior_covariance,
    int active_tracks, int applied, int gated, int augmented, const std::optional<GroundTruthState>& truth) {
    *owned_camera_stream_ << timestamp << ',' << active_tracks << ',' << applied << ',' << gated << ',' << augmented;
    if (const auto result = write_state(*owned_camera_stream_, prior, prior_covariance); !result) return result;
    if (const auto result = write_state(*owned_camera_stream_, posterior, posterior_covariance); !result) return result;
    write_truth(*owned_camera_stream_, truth);
    *owned_camera_stream_ << '\n';
    return *owned_camera_stream_ ? ParseResult<void>{} : std::unexpected(std::string{"failed while writing camera trace row"});
}

ParseResult<void> BenchmarkTraceWriter::write_observations(TimestampNs timestamp, std::span<const StereoObservation> observations,
    std::span<const LandmarkUpdateDiagnostics> diagnostics) {
    std::unordered_map<LandmarkId, const StereoObservation*> by_id;
    for (const StereoObservation& observation : observations) by_id.emplace(observation.id, &observation);
    for (const LandmarkUpdateDiagnostics& diagnostic : diagnostics) {
        const auto iterator = by_id.find(diagnostic.id);
        if (iterator == by_id.end()) return std::unexpected(std::string{"missing observation for diagnostic landmark id"});
        const StereoObservation& observation = *iterator->second;
        const Eigen::Vector4d observed{observation.pixel_cam0.x(), observation.pixel_cam0.y(), observation.pixel_cam1.x(), observation.pixel_cam1.y()};
        const Eigen::Vector4d predicted = observed - diagnostic.innovation;
        *owned_observation_stream_ << timestamp << ',' << diagnostic.id << ',' << outcome_name(diagnostic.outcome) << ','
                             << diagnostic.mahalanobis_distance;
        for (int index = 0; index < kStereoMeasurementDim; ++index) *owned_observation_stream_ << ',' << observed(index);
        for (int index = 0; index < kStereoMeasurementDim; ++index) *owned_observation_stream_ << ',' << predicted(index);
        for (int index = 0; index < kStereoMeasurementDim; ++index) *owned_observation_stream_ << ',' << diagnostic.innovation(index);
        *owned_observation_stream_ << '\n';
    }
    return *owned_observation_stream_ ? ParseResult<void>{} : std::unexpected(std::string{"failed while writing observation trace row"});
}
