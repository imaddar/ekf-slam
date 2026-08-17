#include "evaluation.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>

namespace {

constexpr double kNanosecondsPerSecond = 1e9;
constexpr int kPositionIndex = 0;
constexpr int kVelocityIndex = 3;
constexpr int kOrientationIndex = 6;
constexpr int kAccelerometerBiasIndex = 9;
constexpr int kGyroscopeBiasIndex = 12;

ParseResult<void> validate_ground_truth(std::span<const GroundTruthState> truth) {
    if (truth.size() < 2) return std::unexpected("ground_truth: need at least two states");
    for (std::size_t index = 1; index < truth.size(); ++index) {
        if (truth[index].timestamp <= truth[index - 1].timestamp) {
            return std::unexpected("ground_truth: timestamps must be strictly increasing");
        }
    }
    return {};
}

Eigen::Matrix<double, kRobotDim, 1> robot_error(
    const NominalState& estimate, const GroundTruthState& truth) {
    Eigen::Matrix<double, kRobotDim, 1> error;
    error.segment<3>(kPositionIndex) = truth.position - estimate.position;
    error.segment<3>(kVelocityIndex) = truth.velocity - estimate.velocity;
    error.segment<3>(kOrientationIndex) =
        (estimate.orientation.inverse() * Sophus::SO3d{truth.orientation}).log();
    error.segment<3>(kAccelerometerBiasIndex) = truth.accelerometer_bias - estimate.accelerometer_bias;
    error.segment<3>(kGyroscopeBiasIndex) = truth.gyroscope_bias - estimate.gyroscope_bias;
    return error;
}

}  // namespace

ParseResult<GroundTruthState> interpolate_ground_truth(
    std::span<const GroundTruthState> ground_truth, TimestampNs timestamp) {
    if (const auto valid = validate_ground_truth(ground_truth); !valid) return std::unexpected(valid.error());
    if (timestamp < ground_truth.front().timestamp || timestamp > ground_truth.back().timestamp) {
        return std::unexpected("ground_truth: timestamp is outside available range");
    }
    const auto upper = std::lower_bound(ground_truth.begin(), ground_truth.end(), timestamp,
        [](const GroundTruthState& state, TimestampNs value) { return state.timestamp < value; });
    if (upper->timestamp == timestamp) return *upper;
    const GroundTruthState& after = *upper;
    const GroundTruthState& before = *(upper - 1);
    const double alpha = static_cast<double>(timestamp - before.timestamp) /
        static_cast<double>(after.timestamp - before.timestamp);
    GroundTruthState result{
        .timestamp = timestamp,
        .position = (1.0 - alpha) * before.position + alpha * after.position,
        .orientation = before.orientation.slerp(alpha, after.orientation),
        .velocity = (1.0 - alpha) * before.velocity + alpha * after.velocity,
        .gyroscope_bias = (1.0 - alpha) * before.gyroscope_bias + alpha * after.gyroscope_bias,
        .accelerometer_bias = (1.0 - alpha) * before.accelerometer_bias + alpha * after.accelerometer_bias,
    };
    return result;
}

ParseResult<TrajectoryMetrics> evaluate_trajectory(
    std::span<const TrajectoryEstimate> estimates,
    std::span<const GroundTruthState> ground_truth,
    double rpe_interval_seconds) {
    if (const auto valid = validate_ground_truth(ground_truth); !valid) return std::unexpected(valid.error());
    if (estimates.size() < 2) return std::unexpected("estimates: need at least two states");
    if (!(rpe_interval_seconds > 0.0) || !std::isfinite(rpe_interval_seconds)) {
        return std::unexpected("rpe_interval_seconds: must be finite and positive");
    }
    TrajectoryMetrics metrics{.sample_count = estimates.size()};
    double ate_squared = 0.0, nees_total = 0.0, rpe_translation_squared = 0.0, rpe_rotation_squared = 0.0;
    std::vector<GroundTruthState> associated_truth;
    associated_truth.reserve(estimates.size());
    for (std::size_t index = 0; index < estimates.size(); ++index) {
        if (index > 0 && estimates[index].timestamp <= estimates[index - 1].timestamp) {
            return std::unexpected("estimates: timestamps must be strictly increasing");
        }
        if (!estimates[index].covariance.allFinite()) return std::unexpected("estimates: covariance is non-finite");
        const auto truth = interpolate_ground_truth(ground_truth, estimates[index].timestamp);
        if (!truth) return std::unexpected(truth.error());
        associated_truth.push_back(*truth);
        ate_squared += (estimates[index].state.position - truth->position).squaredNorm();
        const Eigen::LDLT<ImuStateCovariance> decomposition(estimates[index].covariance);
        if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) {
            return std::unexpected("estimates: covariance is not positive definite");
        }
        const auto error = robot_error(estimates[index].state, *truth);
        nees_total += error.dot(decomposition.solve(error));
    }
    for (std::size_t first = 0; first + 1 < estimates.size(); ++first) {
        const TimestampNs target = estimates[first].timestamp + static_cast<TimestampNs>(rpe_interval_seconds * kNanosecondsPerSecond);
        const auto second = std::lower_bound(estimates.begin() + static_cast<std::ptrdiff_t>(first + 1), estimates.end(), target,
            [](const TrajectoryEstimate& estimate, TimestampNs value) { return estimate.timestamp < value; });
        if (second == estimates.end()) continue;
        const std::size_t second_index = static_cast<std::size_t>(second - estimates.begin());
        const double dt = static_cast<double>(second->timestamp - estimates[first].timestamp) / kNanosecondsPerSecond;
        const Eigen::Vector3d estimated_delta = estimates[first].state.orientation.inverse() *
            (second->state.position - estimates[first].state.position);
        const Sophus::SO3d estimated_rotation = estimates[first].state.orientation.inverse() * second->state.orientation;
        const Eigen::Vector3d truth_delta = Sophus::SO3d{associated_truth[first].orientation}.inverse() *
            (associated_truth[second_index].position - associated_truth[first].position);
        const Sophus::SO3d truth_rotation = Sophus::SO3d{associated_truth[first].orientation}.inverse() *
            Sophus::SO3d{associated_truth[second_index].orientation};
        rpe_translation_squared += ((estimated_delta - truth_delta) / dt).squaredNorm();
        rpe_rotation_squared += std::pow((truth_rotation.inverse() * estimated_rotation).log().norm() / dt, 2.0);
        ++metrics.rpe_pair_count;
    }
    metrics.ate_position_rmse_m = std::sqrt(ate_squared / static_cast<double>(metrics.sample_count));
    metrics.mean_robot_nees = nees_total / static_cast<double>(metrics.sample_count);
    if (metrics.rpe_pair_count > 0) {
        metrics.rpe_translation_rmse_m_per_s = std::sqrt(rpe_translation_squared / metrics.rpe_pair_count);
        metrics.rpe_rotation_rmse_rad_per_s = std::sqrt(rpe_rotation_squared / metrics.rpe_pair_count);
    }
    return metrics;
}
