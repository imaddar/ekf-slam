#include "propagation.hpp"

#include <cmath>
#include <format>

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

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d skew;
    skew << 0.0, -vector.z(), vector.y(),
        vector.z(), 0.0, -vector.x(),
        -vector.y(), vector.x(), 0.0;
    return skew;
}

}  // namespace

ParseResult<PropagationResult> propagate(
    const NominalState& nominal_state,
    const ImuMeasurement& measurement,
    const ImuCalibration& imu_calibration,
    double timestep_seconds,
    const StateCovariance& covariance) {

    // Out-of-order IMU samples produce a negative dt, which would subtract
    // information from P rather than add it. Reject rather than silently drift.
    if (!std::isfinite(timestep_seconds) || timestep_seconds < 0.0) {
        return std::unexpected(std::format(
            "timestep_seconds: expected a finite non-negative timestep, found {}", timestep_seconds));
    }

    // Remove the current bias estimate before using the IMU sample in either update.
    const Eigen::Vector3d acceleration = measurement.acceleration - nominal_state.accelerometer_bias;
    const Eigen::Vector3d angular_velocity = measurement.angular_velocity - nominal_state.gyroscope_bias;
    // IMU acceleration is measured in the body frame; rotate it before adding world gravity.
    const Eigen::Vector3d world_acceleration = nominal_state.orientation * acceleration + kGravity;
    const Eigen::Matrix3d rotation = nominal_state.orientation.matrix();

    // This is the simple constant-input integration step used between adjacent IMU samples.
    NominalState updated_state{
        // TODO: Benchmark the second-order acceleration term against first-order position integration.
        .position = nominal_state.position + nominal_state.velocity * timestep_seconds
            + 0.5 * world_acceleration * timestep_seconds * timestep_seconds,
        .velocity = nominal_state.velocity + world_acceleration * timestep_seconds,
        .orientation = nominal_state.orientation * Sophus::SO3d::exp(angular_velocity * timestep_seconds),
        .accelerometer_bias = nominal_state.accelerometer_bias,
        .gyroscope_bias = nominal_state.gyroscope_bias,
    };

    // F is continuous-time here; each 3x3 block maps one small error component into another.
    StateCovariance continuous_transition = StateCovariance::Zero();
    continuous_transition.block<kBlockSize, kBlockSize>(kPositionIndex, kVelocityIndex) = Eigen::Matrix3d::Identity();
    continuous_transition.block<kBlockSize, kBlockSize>(kVelocityIndex, kOrientationIndex) =
        -rotation * skew_symmetric(acceleration);
    continuous_transition.block<kBlockSize, kBlockSize>(kVelocityIndex, kAccelerometerBiasIndex) = -rotation;
    continuous_transition.block<kBlockSize, kBlockSize>(kOrientationIndex, kOrientationIndex) =
        -skew_symmetric(angular_velocity);
    continuous_transition.block<kBlockSize, kBlockSize>(kOrientationIndex, kGyroscopeBiasIndex) =
        -Eigen::Matrix3d::Identity();

    // Start with Phi = I + F dt; higher-order discretization can be evaluated with Q later.
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
    // EuRoC stores noise densities/random walks, so Q_raw uses their variances.
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

    // This first-order Qd keeps the noise model cheap while the exact integral remains future work.
    const StateCovariance process_noise =
        noise_jacobian * raw_noise * noise_jacobian.transpose() * timestep_seconds;
    const StateCovariance updated_covariance =
        discrete_transition * covariance * discrete_transition.transpose() + process_noise;

    return PropagationResult{
        .nominal_state = updated_state,
        .covariance = updated_covariance,
    };
}
