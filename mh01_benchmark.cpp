#include "evaluation.hpp"
#include "feature_frontend.hpp"
#include "image_io.hpp"
#include "landmark_augmentation.hpp"
#include "propagation.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>

namespace {

int fail(std::string_view message) {
    std::cerr << "mh01_benchmark: " << message << '\n';
    return EXIT_FAILURE;
}

}  // namespace

int main() {
    const auto dataset = parse_dataset(std::filesystem::path{EKF_SLAM_SOURCE_DIR} / "datasets/machine_hall/MH_01_easy");
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
    constexpr double kPixelSigma = 0.5;
    const Eigen::Matrix4d pixel_covariance = kPixelSigma * kPixelSigma * Eigen::Matrix4d::Identity();
    std::size_t imu_index = 0;
    TimestampNs last_imu_timestamp = initial.timestamp;
    int applied = 0, gated = 0, augmented = 0, peak_tracks = 0;
    std::chrono::nanoseconds frontend_time{}, update_time{};
    std::vector<TrajectoryEstimate> estimates;
    estimates.reserve(dataset->stereo_pairs.size());

    for (const StereoPair& pair : dataset->stereo_pairs) {
        while (imu_index < dataset->imu_measurements.size() && dataset->imu_measurements[imu_index].timestamp <= pair.timestamp) {
            const ImuMeasurement& measurement = dataset->imu_measurements[imu_index++];
            if (measurement.timestamp <= last_imu_timestamp) continue;
            const double dt = static_cast<double>(measurement.timestamp - last_imu_timestamp) * 1e-9;
            const auto propagation = propagate_slam(state, measurement, dataset->imu_calibration, dt);
            if (!propagation) return fail(propagation.error());
            last_imu_timestamp = measurement.timestamp;
        }
        const auto cam0 = load_grayscale_png(pair.cam0_image_path);
        const auto cam1 = load_grayscale_png(pair.cam1_image_path);
        if (!cam0) return fail(cam0.error());
        if (!cam1) return fail(cam1.error());
        const auto frontend_start = std::chrono::steady_clock::now();
        const auto frame = frontend.process(*cam0, *cam1, pair.timestamp);
        frontend_time += std::chrono::steady_clock::now() - frontend_start;
        if (!frame) return fail(frame.error());
        peak_tracks = std::max(peak_tracks, frame->active_track_count);
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
        const auto removal = state.remove_landmarks(frame->dead_landmarks);
        if (!removal) return fail(removal.error());
        // EuRoC camera capture continues slightly beyond the ground-truth
        // stream.  Metrics are defined only on their common timestamp range.
        if (pair.timestamp >= dataset->ground_truth_states.front().timestamp &&
            pair.timestamp <= dataset->ground_truth_states.back().timestamp) {
            estimates.push_back(TrajectoryEstimate{.timestamp = pair.timestamp, .state = state.robot,
                .covariance = state.robot_covariance()});
        }
    }
    const auto metrics = evaluate_trajectory(estimates, dataset->ground_truth_states);
    if (!metrics) return fail(metrics.error());
    const double frames = static_cast<double>(estimates.size());
    std::cout << "MH_01_easy benchmark\n"
              << "frames=" << metrics->sample_count << " rpe_pairs=" << metrics->rpe_pair_count << '\n'
              << "ate_position_rmse_m=" << metrics->ate_position_rmse_m << '\n'
              << "rpe_translation_rmse_m_per_s=" << metrics->rpe_translation_rmse_m_per_s << '\n'
              << "rpe_rotation_rmse_rad_per_s=" << metrics->rpe_rotation_rmse_rad_per_s << '\n'
              << "mean_robot_nees_15dof=" << metrics->mean_robot_nees << '\n'
              << "peak_tracks=" << peak_tracks << " augmented=" << augmented << " applied=" << applied << " gated=" << gated << '\n'
              << "frontend_ms_per_frame=" << frontend_time.count() / frames / 1e6 << '\n'
              << "update_ms_per_frame=" << update_time.count() / frames / 1e6 << '\n';
    return EXIT_SUCCESS;
}
