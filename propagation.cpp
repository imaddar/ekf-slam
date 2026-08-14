#include "propagation.hpp"

#include "slam_state.hpp"

#include <cmath>
#include <format>
#include <new>

namespace {

const Eigen::Vector3d kGravity{0.0, 0.0, -9.81};

constexpr int kPositionIndex = 0;
constexpr int kVelocityIndex = 3;
constexpr int kOrientationIndex = 6;
constexpr int kAccelerometerBiasIndex = 9;
constexpr int kGyroscopeBiasIndex = 12;
constexpr int kBlockSize = 3;
constexpr int kNoiseSize = 12;
constexpr int kAccelNoiseIndex = 0;
constexpr int kGyroNoiseIndex = 3;
constexpr int kAccelBiasNoiseIndex = 6;
constexpr int kGyroBiasNoiseIndex = 9;

using NoiseJacobian = Eigen::Matrix<double, kErrorStateSize, kNoiseSize>;
using RawNoiseCovariance = Eigen::Matrix<double, kNoiseSize, kNoiseSize>;

struct RobotPropagationTerms {
    NominalState updated_state;
    StateCovariance discrete_transition;
    StateCovariance process_noise;
};

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d skew;
    skew << 0.0, -vector.z(), vector.y(),
        vector.z(), 0.0, -vector.x(),
        -vector.y(), vector.x(), 0.0;
    return skew;
}

ParseResult<RobotPropagationTerms> build_robot_propagation_terms(
    const NominalState& nominal_state,
    const ImuMeasurement& measurement,
    const ImuCalibration& imu_calibration,
    double timestep_seconds) {
    // Out-of-order IMU samples would run the covariance update backwards.
    if (!std::isfinite(timestep_seconds) || timestep_seconds < 0.0) {
        return std::unexpected(std::format(
            "timestep_seconds: expected a finite non-negative timestep, found {}", timestep_seconds));
    }

    const Eigen::Vector3d acceleration = measurement.acceleration - nominal_state.accelerometer_bias;
    const Eigen::Vector3d angular_velocity = measurement.angular_velocity - nominal_state.gyroscope_bias;
    // IMU acceleration is body-frame specific force; rotate it before adding gravity.
    const Eigen::Vector3d world_acceleration = nominal_state.orientation * acceleration + kGravity;
    const Eigen::Matrix3d rotation = nominal_state.orientation.matrix();

    // Constant-input integration matches the existing IMU-only propagation path.
    NominalState updated_state{
        // TODO: Benchmark the second-order position term against first-order integration.
        .position = nominal_state.position + nominal_state.velocity * timestep_seconds
            + 0.5 * world_acceleration * timestep_seconds * timestep_seconds,
        .velocity = nominal_state.velocity + world_acceleration * timestep_seconds,
        .orientation = nominal_state.orientation * Sophus::SO3d::exp(angular_velocity * timestep_seconds),
        .accelerometer_bias = nominal_state.accelerometer_bias,
        .gyroscope_bias = nominal_state.gyroscope_bias,
    };

    // F is continuous-time; each 3x3 block maps one small error component into another.
    StateCovariance continuous_transition = StateCovariance::Zero();
    continuous_transition.block<kBlockSize, kBlockSize>(kPositionIndex, kVelocityIndex) =
        Eigen::Matrix3d::Identity();
    continuous_transition.block<kBlockSize, kBlockSize>(kVelocityIndex, kOrientationIndex) =
        -rotation * skew_symmetric(acceleration);
    continuous_transition.block<kBlockSize, kBlockSize>(kVelocityIndex, kAccelerometerBiasIndex) = -rotation;
    continuous_transition.block<kBlockSize, kBlockSize>(kOrientationIndex, kOrientationIndex) =
        -skew_symmetric(angular_velocity);
    continuous_transition.block<kBlockSize, kBlockSize>(kOrientationIndex, kGyroscopeBiasIndex) =
        -Eigen::Matrix3d::Identity();

    // First-order discretization; higher-order alternatives remain future work.
    const StateCovariance discrete_transition =
        StateCovariance::Identity() + continuous_transition * timestep_seconds;

    NoiseJacobian noise_jacobian = NoiseJacobian::Zero();
    noise_jacobian.block<kBlockSize, kBlockSize>(kVelocityIndex, kAccelNoiseIndex) = -rotation;
    noise_jacobian.block<kBlockSize, kBlockSize>(kOrientationIndex, kGyroNoiseIndex) =
        -Eigen::Matrix3d::Identity();
    noise_jacobian.block<kBlockSize, kBlockSize>(kAccelerometerBiasIndex, kAccelBiasNoiseIndex) =
        Eigen::Matrix3d::Identity();
    noise_jacobian.block<kBlockSize, kBlockSize>(kGyroscopeBiasIndex, kGyroBiasNoiseIndex) =
        Eigen::Matrix3d::Identity();

    RawNoiseCovariance raw_noise = RawNoiseCovariance::Zero();
    raw_noise.block<kBlockSize, kBlockSize>(kAccelNoiseIndex, kAccelNoiseIndex) =
        imu_calibration.accelerometer_noise_density * imu_calibration.accelerometer_noise_density
        * Eigen::Matrix3d::Identity();
    raw_noise.block<kBlockSize, kBlockSize>(kGyroNoiseIndex, kGyroNoiseIndex) =
        imu_calibration.gyroscope_noise_density * imu_calibration.gyroscope_noise_density
        * Eigen::Matrix3d::Identity();
    raw_noise.block<kBlockSize, kBlockSize>(kAccelBiasNoiseIndex, kAccelBiasNoiseIndex) =
        imu_calibration.accelerometer_random_walk * imu_calibration.accelerometer_random_walk
        * Eigen::Matrix3d::Identity();
    raw_noise.block<kBlockSize, kBlockSize>(kGyroBiasNoiseIndex, kGyroBiasNoiseIndex) =
        imu_calibration.gyroscope_random_walk * imu_calibration.gyroscope_random_walk
        * Eigen::Matrix3d::Identity();

    // This first-order Qd keeps the current noise model inexpensive.
    const StateCovariance process_noise =
        noise_jacobian * raw_noise * noise_jacobian.transpose() * timestep_seconds;

    return RobotPropagationTerms{
        .updated_state = updated_state,
        .discrete_transition = discrete_transition,
        .process_noise = process_noise,
    };
}

}  // namespace

ParseResult<PropagationResult> propagate(
    const NominalState& nominal_state,
    const ImuMeasurement& measurement,
    const ImuCalibration& imu_calibration,
    double timestep_seconds,
    const StateCovariance& covariance) {
    const auto terms = build_robot_propagation_terms(
        nominal_state, measurement, imu_calibration, timestep_seconds);
    if (!terms) {
        return std::unexpected(terms.error());
    }

    const StateCovariance updated_covariance =
        terms->discrete_transition * covariance * terms->discrete_transition.transpose()
        + terms->process_noise;
    return PropagationResult{
        .nominal_state = terms->updated_state,
        .covariance = updated_covariance,
    };
}

ParseResult<void> propagate_slam(
    SlamState& slam_state,
    const ImuMeasurement& measurement,
    const ImuCalibration& imu_calibration,
    double timestep_seconds) {
    const auto terms = build_robot_propagation_terms(
        slam_state.robot, measurement, imu_calibration, timestep_seconds);
    if (!terms) {
        return std::unexpected(terms.error());
    }

    try {
        const ImuStateCovariance updated_robot_covariance =
            terms->discrete_transition * slam_state.robot_covariance()
            * terms->discrete_transition.transpose() + terms->process_noise;
        auto robot_landmark_covariance = slam_state.robot_landmark_covariance();
        for (int offset = 0; offset < robot_landmark_covariance.cols(); offset += kLandmarkDim) {
            const Eigen::Matrix<double, kRobotDim, kLandmarkDim> old_block =
                robot_landmark_covariance.middleCols<kLandmarkDim>(offset);
            robot_landmark_covariance.middleCols<kLandmarkDim>(offset) =
                terms->discrete_transition * old_block;
        }

        slam_state.robot = terms->updated_state;
        slam_state.robot_covariance() = updated_robot_covariance;
        const auto covariance_result = slam_state.set_robot_landmark_covariance(robot_landmark_covariance);
        if (!covariance_result) {
            return std::unexpected(covariance_result.error());
        }
    } catch (const std::bad_alloc&) {
        return std::unexpected("slam propagation: covariance allocation failed");
    }

    return {};
}
