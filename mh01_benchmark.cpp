#include "benchmark_trace.hpp"
#include "evaluation.hpp"
#include "feature_frontend.hpp"
#include "image_io.hpp"
#include "landmark_augmentation.hpp"
#include "propagation.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>

namespace {

int fail(std::string_view message) {
    std::cerr << "mh01_benchmark: " << message << '\n';
    return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char** argv) {
    constexpr std::string_view kUsage = "usage: mh01_benchmark [max_frames] [--sequence NAME] [--trace-dir directory]";
    std::size_t max_frames = std::numeric_limits<std::size_t>::max();
    std::string sequence_name = "MH_01_easy";
    std::optional<std::filesystem::path> trace_directory;
    for (int argument_index = 1; argument_index < argc; ++argument_index) {
        const std::string_view argument{argv[argument_index]};
        if (argument == "--trace-dir") {
            if (++argument_index >= argc || trace_directory) return fail(kUsage);
            trace_directory = argv[argument_index];
            continue;
        }
        if (argument == "--sequence") {
            if (++argument_index >= argc) return fail(kUsage);
            sequence_name = argv[argument_index];
            continue;
        }
        const auto parsed = std::from_chars(argument.data(), argument.data() + argument.size(), max_frames);
        if (parsed.ec != std::errc{} || parsed.ptr != argument.data() + argument.size() || max_frames == 0) {
            return fail(kUsage);
        }
    }
    const auto dataset = parse_dataset(std::filesystem::path{EKF_SLAM_SOURCE_DIR} / "datasets/machine_hall" / sequence_name);
    if (!dataset) return fail(dataset.error());
    const auto rectification = make_stereo_rectification(dataset->cam0_calibration, dataset->cam1_calibration);
    if (!rectification) return fail(rectification.error());
    auto frontend_result = FeatureFrontend::create(*rectification);
    if (!frontend_result) return fail(frontend_result.error());
    FeatureFrontend frontend = std::move(*frontend_result);
    const GroundTruthState& initial = dataset->ground_truth_states.front();
    const auto state_result = SlamState::create(100, NominalState{
        .position = initial.position, .velocity = initial.velocity, .orientation = Sophus::SO3d{initial.orientation},
        .accelerometer_bias = initial.accelerometer_bias, .gyroscope_bias = initial.gyroscope_bias,
    }, 1e-4 * ImuStateCovariance::Identity());
    if (!state_result) return fail(state_result.error());
    SlamState state = std::move(*state_result);
    // Measured per-observation residual sigma is 0.25 px (MAD, so the bulk
    // rather than the outlier tail) and per-landmark residuals autocorrelate at
    // 0.784 at lag 1, which by the AR(1) factor (1 + rho) / (1 - rho) = 8.26
    // argues for an effective 0.72 px. Measured, that trade is a wash: NEES
    // improves 10% and ATE degrades 10%, because R is not the binding
    // constraint on consistency -- unobservable-direction drift is. Left at
    // 0.5 px and worth revisiting once FEJ lands. See BENCHMARKS.md.
    constexpr double kPixelSigma = 0.5;
    const Eigen::Matrix4d pixel_covariance = kPixelSigma * kPixelSigma * Eigen::Matrix4d::Identity();
    std::optional<BenchmarkTraceWriter> trace;
    if (trace_directory) {
        auto writer = BenchmarkTraceWriter::create(*trace_directory, EKF_SLAM_GIT_REVISION, state.max_landmarks(), kPixelSigma);
        if (!writer) return fail(writer.error());
        trace = std::move(*writer);
    }
    std::size_t imu_index = 0;
    TimestampNs last_imu_timestamp = initial.timestamp;
    int applied = 0, gated = 0, augmented = 0, peak_tracks = 0;
    std::size_t processed_frames = 0;
    std::chrono::nanoseconds frontend_time{}, update_time{}, decode_time{};
    std::vector<TrajectoryEstimate> estimates;
    estimates.reserve(dataset->stereo_pairs.size());

    for (const StereoPair& pair : dataset->stereo_pairs) {
        if (processed_frames >= max_frames) break;
        while (imu_index < dataset->imu_measurements.size() && dataset->imu_measurements[imu_index].timestamp <= pair.timestamp) {
            const ImuMeasurement& measurement = dataset->imu_measurements[imu_index++];
            if (measurement.timestamp <= last_imu_timestamp) continue;
            const double dt = static_cast<double>(measurement.timestamp - last_imu_timestamp) * 1e-9;
            const auto propagation = propagate_slam(state, measurement, dataset->imu_calibration, dt);
            if (!propagation) return fail(propagation.error());
            last_imu_timestamp = measurement.timestamp;
            if (trace) {
                const auto ground_truth = interpolate_ground_truth(dataset->ground_truth_states, measurement.timestamp);
                if (const auto written = trace->write_imu(measurement.timestamp, dt, state.robot, state.robot_covariance(),
                        ground_truth ? std::optional<GroundTruthState>{*ground_truth} : std::nullopt); !written) {
                    return fail(written.error());
                }
            }
        }
        const auto decode_start = std::chrono::steady_clock::now();
        const auto cam0 = load_grayscale_png(pair.cam0_image_path);
        const auto cam1 = load_grayscale_png(pair.cam1_image_path);
        decode_time += std::chrono::steady_clock::now() - decode_start;
        if (!cam0) return fail(cam0.error());
        if (!cam1) return fail(cam1.error());
        const auto frontend_start = std::chrono::steady_clock::now();
        const auto frame = frontend.process(*cam0, *cam1, pair.timestamp);
        frontend_time += std::chrono::steady_clock::now() - frontend_start;
        if (!frame) return fail(frame.error());
        peak_tracks = std::max(peak_tracks, frame->active_track_count);
        const NominalState prior = state.robot;
        const ImuStateCovariance prior_covariance = state.robot_covariance();
        const auto update_start = std::chrono::steady_clock::now();
        const auto update = update_stereo_frame(state, frame->mapped_observations, rectification->cam0_rectified,
            rectification->cam1_rectified, pixel_covariance);
        update_time += std::chrono::steady_clock::now() - update_start;
        if (!update) return fail(update.error());
        applied += update->applied_count;
        gated += update->gated_count;
        frontend.report_outcomes(update->diagnostics);
        std::vector<LandmarkId> births;
        for (const StereoObservation& candidate : frame->birth_candidates) {
            if (state.active_landmarks() >= state.max_landmarks()) break;
            const auto augmented_landmark = augment_landmark(state, candidate.id, candidate.pixel_cam0, candidate.pixel_cam1,
                rectification->cam0_rectified, rectification->cam1_rectified, pixel_covariance, 1.0);
            if (!augmented_landmark) return fail(augmented_landmark.error());
            births.push_back(candidate.id);
            ++augmented;
        }
        frontend.report_augmentation(births);
        if (trace) {
            const auto ground_truth = interpolate_ground_truth(dataset->ground_truth_states, pair.timestamp);
            if (const auto observations_written = trace->write_observations(pair.timestamp, frame->mapped_observations, update->diagnostics); !observations_written) {
                return fail(observations_written.error());
            }
            if (const auto camera_written = trace->write_camera(pair.timestamp, prior, prior_covariance, state.robot,
                    state.robot_covariance(), frame->active_track_count, update->applied_count, update->gated_count,
                    static_cast<int>(births.size()), ground_truth ? std::optional<GroundTruthState>{*ground_truth} : std::nullopt); !camera_written) {
                return fail(camera_written.error());
            }
        }
        const auto removal = state.remove_landmarks(frame->dead_landmarks);
        if (!removal) return fail(removal.error());
        // EuRoC camera capture continues slightly beyond the ground-truth
        // stream.  Metrics are defined only on their common timestamp range.
        if (pair.timestamp >= dataset->ground_truth_states.front().timestamp &&
            pair.timestamp <= dataset->ground_truth_states.back().timestamp) {
            estimates.push_back(TrajectoryEstimate{.timestamp = pair.timestamp, .state = state.robot,
                .covariance = state.robot_covariance()});
        }
        ++processed_frames;
    }
    const auto metrics = evaluate_trajectory(estimates, dataset->ground_truth_states);
    if (!metrics) return fail(metrics.error());
    // Timing is per processed frame; accuracy metrics cover only the camera
    // timestamps shared with ground truth, so the two counts differ.
    const double frames = static_cast<double>(processed_frames);
    const FrontendStageTimings& stages = frontend.stage_timings();
    const double frontend_ns = static_cast<double>(stages.total.count());
    const auto stage_line = [&](std::string_view name, std::chrono::nanoseconds total) {
        std::cout << "  " << name << '=' << static_cast<double>(total.count()) / frames / 1e6 << " ms ("
                  << 100.0 * static_cast<double>(total.count()) / frontend_ns << "%)\n";
    };
    const std::chrono::nanoseconds accounted = stages.rectify + stages.pyramid + stages.temporal_klt
        + stages.forward_backward + stages.stereo_tracked + stages.detect + stages.stereo_new;
    std::cout << sequence_name << " benchmark\n"
              << "frames=" << metrics->sample_count << " rpe_pairs=" << metrics->rpe_pair_count << '\n'
              << "ate_position_rmse_m=" << metrics->ate_position_rmse_m << '\n'
              << "rpe_translation_rmse_m_per_s=" << metrics->rpe_translation_rmse_m_per_s << '\n'
              << "rpe_rotation_rmse_rad_per_s=" << metrics->rpe_rotation_rmse_rad_per_s << '\n'
              << "mean_robot_nees_15dof=" << metrics->mean_robot_nees << '\n'
              << "peak_tracks=" << peak_tracks << " augmented=" << augmented << " applied=" << applied << " gated=" << gated << '\n'
              << "processed_frames=" << processed_frames << '\n'
              << "png_decode_ms_per_frame=" << decode_time.count() / frames / 1e6 << '\n'
              << "frontend_ms_per_frame=" << frontend_time.count() / frames / 1e6 << '\n'
              << "update_ms_per_frame=" << update_time.count() / frames / 1e6 << '\n'
              << "frontend stage breakdown\n";
    stage_line("rectify", stages.rectify);
    stage_line("pyramid", stages.pyramid);
    stage_line("temporal_klt", stages.temporal_klt);
    stage_line("forward_backward", stages.forward_backward);
    stage_line("stereo_tracked", stages.stereo_tracked);
    stage_line("detect", stages.detect);
    stage_line("stereo_new", stages.stereo_new);
    stage_line("bookkeeping", stages.total - accounted);
    const auto per_call_us = [](std::chrono::nanoseconds total, std::size_t calls) {
        return calls == 0 ? 0.0 : static_cast<double>(total.count()) / static_cast<double>(calls) / 1e3;
    };
    std::cout << "temporal_klt_calls_per_frame=" << static_cast<double>(stages.temporal_calls) / frames
              << " us_per_call=" << per_call_us(stages.temporal_klt, stages.temporal_calls) << '\n'
              << "stereo_new_calls_per_frame=" << static_cast<double>(stages.stereo_new_calls) / frames
              << " us_per_call=" << per_call_us(stages.stereo_new, stages.stereo_new_calls) << '\n';
    return EXIT_SUCCESS;
}
