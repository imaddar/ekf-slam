#include "propagation.hpp"

#include "parser.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <numbers>
#include <random>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

namespace {

constexpr double kTolerance = 1e-9;
constexpr int kPositionIndex = 0;
constexpr int kVelocityIndex = 3;
constexpr int kOrientationIndex = 6;
constexpr int kAccelerometerBiasIndex = 9;
constexpr int kGyroscopeBiasIndex = 12;
constexpr std::uint64_t kOneSecondNs = 1'000'000'000;

NominalState make_state() {
    return {
        .position = Eigen::Vector3d::Zero(),
        .velocity = Eigen::Vector3d::Zero(),
        .orientation = Sophus::SO3d{},
        .accelerometer_bias = Eigen::Vector3d::Zero(),
        .gyroscope_bias = Eigen::Vector3d::Zero(),
    };
}

NominalState make_state_from_ground_truth(const GroundTruthState& ground_truth) {
    return {
        .position = ground_truth.position,
        .velocity = ground_truth.velocity,
        .orientation = Sophus::SO3d{ground_truth.orientation},
        .accelerometer_bias = ground_truth.accelerometer_bias,
        .gyroscope_bias = ground_truth.gyroscope_bias,
    };
}

ImuCalibration make_imu_calibration(
    double accelerometer_noise_density = 0.0,
    double gyroscope_noise_density = 0.0,
    double accelerometer_random_walk = 0.0,
    double gyroscope_random_walk = 0.0) {
    return {
        .t_bs = Eigen::Matrix4d::Identity(),
        .rate_hz = 200.0,
        .gyroscope_noise_density = gyroscope_noise_density,
        .gyroscope_random_walk = gyroscope_random_walk,
        .accelerometer_noise_density = accelerometer_noise_density,
        .accelerometer_random_walk = accelerometer_random_walk,
    };
}

const GroundTruthState& nearest_ground_truth(
    const std::vector<GroundTruthState>& ground_truth_states,
    TimestampNs timestamp) {
    const auto nearest = std::ranges::min_element(
        ground_truth_states,
        [timestamp](const GroundTruthState& lhs, const GroundTruthState& rhs) {
            const auto lhs_delta = lhs.timestamp > timestamp ? lhs.timestamp - timestamp : timestamp - lhs.timestamp;
            const auto rhs_delta = rhs.timestamp > timestamp ? rhs.timestamp - timestamp : timestamp - rhs.timestamp;
            return lhs_delta < rhs_delta;
        });

    return *nearest;
}

}  // namespace

TEST(PropagationTest, KeepsStationaryStateWhenSpecificForceBalancesGravity) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };
    const StateCovariance covariance = StateCovariance::Zero();

    const auto propagated = propagate(state, measurement, imu_calibration, 0.005, covariance);
    ASSERT_TRUE(propagated) << propagated.error();
    const auto& result = *propagated;

    EXPECT_TRUE(result.nominal_state.position.isApprox(state.position, kTolerance));
    EXPECT_TRUE(result.nominal_state.velocity.isApprox(state.velocity, kTolerance));
    EXPECT_TRUE(result.nominal_state.orientation.unit_quaternion().isApprox(state.orientation.unit_quaternion(), kTolerance));
    EXPECT_TRUE(result.nominal_state.accelerometer_bias.isApprox(state.accelerometer_bias, kTolerance));
    EXPECT_TRUE(result.nominal_state.gyroscope_bias.isApprox(state.gyroscope_bias, kTolerance));
    EXPECT_TRUE(result.covariance.isApprox(covariance, kTolerance));
}

TEST(PropagationTest, IntegratesPositionAndVelocityFromWorldAcceleration) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{1.0, 2.0, 12.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };
    const StateCovariance covariance = StateCovariance::Identity();

    const auto propagated = propagate(state, measurement, imu_calibration, 0.1, covariance);
    ASSERT_TRUE(propagated) << propagated.error();
    const auto& result = *propagated;

    EXPECT_TRUE(result.nominal_state.position.isApprox(Eigen::Vector3d{0.005, 0.01, 0.015}, kTolerance));
    EXPECT_TRUE(result.nominal_state.velocity.isApprox(Eigen::Vector3d{0.1, 0.2, 0.3}, kTolerance));
    EXPECT_TRUE(result.nominal_state.orientation.unit_quaternion().isApprox(state.orientation.unit_quaternion(), kTolerance));
}

TEST(PropagationTest, IntegratesOrientationFromAngularVelocity) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d{0.0, 0.0, std::numbers::pi},
    };
    const StateCovariance covariance = StateCovariance::Identity();

    const auto propagated = propagate(state, measurement, imu_calibration, 0.5, covariance);
    ASSERT_TRUE(propagated) << propagated.error();
    const auto& result = *propagated;
    const Sophus::SO3d expected_orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.0, 0.0, std::numbers::pi / 2.0});

    EXPECT_TRUE(result.nominal_state.orientation.unit_quaternion().isApprox(expected_orientation.unit_quaternion(), kTolerance));
}

TEST(PropagationTest, RemovesAccelerometerAndGyroscopeBiasesBeforeIntegration) {
    NominalState state{
        .position = Eigen::Vector3d{1.0, 2.0, 3.0},
        .velocity = Eigen::Vector3d{4.0, 5.0, 6.0},
        .orientation = Sophus::SO3d{},
        .accelerometer_bias = Eigen::Vector3d{0.1, 0.2, 0.3},
        .gyroscope_bias = Eigen::Vector3d{0.4, 0.5, 0.6},
    };
    const auto imu_calibration = make_imu_calibration();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{1.1, 0.2, 10.11},
        .angular_velocity = Eigen::Vector3d{0.4, 0.5, 1.6},
    };
    const StateCovariance covariance = StateCovariance::Zero();

    const auto propagated = propagate(state, measurement, imu_calibration, 0.1, covariance);
    ASSERT_TRUE(propagated) << propagated.error();
    const auto& result = *propagated;
    const Sophus::SO3d expected_orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.0, 0.0, 0.1});

    EXPECT_TRUE(result.nominal_state.position.isApprox(Eigen::Vector3d{1.405, 2.5, 3.6}, kTolerance));
    EXPECT_TRUE(result.nominal_state.velocity.isApprox(Eigen::Vector3d{4.1, 5.0, 6.0}, kTolerance));
    EXPECT_TRUE(result.nominal_state.orientation.unit_quaternion().isApprox(expected_orientation.unit_quaternion(), kTolerance));
    EXPECT_TRUE(result.nominal_state.accelerometer_bias.isApprox(state.accelerometer_bias, kTolerance));
    EXPECT_TRUE(result.nominal_state.gyroscope_bias.isApprox(state.gyroscope_bias, kTolerance));
    EXPECT_TRUE(result.covariance.isApprox(covariance, kTolerance));
}

TEST(PropagationTest, UpdatesCovarianceWithStateTransitionMatrix) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };
    const StateCovariance covariance = StateCovariance::Identity();

    const auto propagated = propagate(state, measurement, imu_calibration, 0.1, covariance);
    ASSERT_TRUE(propagated) << propagated.error();
    const auto& result = *propagated;

    EXPECT_TRUE((result.covariance.block<3, 3>(kPositionIndex, kPositionIndex).isApprox(1.01 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kPositionIndex, kVelocityIndex).isApprox(0.1 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kVelocityIndex, kAccelerometerBiasIndex).isApprox(-0.1 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kOrientationIndex, kGyroscopeBiasIndex).isApprox(-0.1 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kVelocityIndex, kOrientationIndex).isApprox(
        (Eigen::Matrix3d{} << 0.0, 0.981, 0.0, -0.981, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
        kTolerance)));
    EXPECT_TRUE(result.covariance.isApprox(result.covariance.transpose(), kTolerance));
}

TEST(PropagationTest, AddsFirstOrderProcessNoiseFromImuCalibration) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration(
        2.0,
        3.0,
        4.0,
        5.0);
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };
    const StateCovariance covariance = StateCovariance::Zero();

    const auto propagated = propagate(state, measurement, imu_calibration, 0.1, covariance);
    ASSERT_TRUE(propagated) << propagated.error();
    const auto& result = *propagated;

    EXPECT_TRUE((result.covariance.block<3, 3>(kPositionIndex, kPositionIndex).isZero(kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kVelocityIndex, kVelocityIndex).isApprox(0.4 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kOrientationIndex, kOrientationIndex).isApprox(0.9 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kAccelerometerBiasIndex, kAccelerometerBiasIndex).isApprox(1.6 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE((result.covariance.block<3, 3>(kGyroscopeBiasIndex, kGyroscopeBiasIndex).isApprox(2.5 * Eigen::Matrix3d::Identity(), kTolerance)));
    EXPECT_TRUE(result.covariance.isApprox(result.covariance.transpose(), kTolerance));
}

TEST(PropagationTest, ProcessNoiseKeepsZeroInitialCovariancePositiveSemiDefinite) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration(
        0.02,
        0.03,
        0.004,
        0.005);
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{1.0, 2.0, 12.81},
        .angular_velocity = Eigen::Vector3d{0.1, 0.2, 0.3},
    };
    const StateCovariance covariance = StateCovariance::Zero();

    const auto propagated = propagate(state, measurement, imu_calibration, 0.005, covariance);
    ASSERT_TRUE(propagated) << propagated.error();
    const auto& result = *propagated;
    const Eigen::SelfAdjointEigenSolver<StateCovariance> eigen_solver(result.covariance);

    EXPECT_TRUE(result.covariance.isApprox(result.covariance.transpose(), kTolerance));
    EXPECT_GE(eigen_solver.eigenvalues().minCoeff(), -kTolerance);
}

TEST(PropagationTest, SmokePropagatesShortEuRocWindowNearGroundTruth) {
    const auto sequence_root = std::filesystem::path(EKF_SLAM_SOURCE_DIR) / "datasets/machine_hall/MH_01_easy";
    ASSERT_TRUE(std::filesystem::exists(sequence_root))
        << "MH_01_easy dataset fixture is required for this smoke test: " << sequence_root;

    const auto dataset = parse_dataset(sequence_root);
    ASSERT_TRUE(dataset) << dataset.error();
    ASSERT_FALSE(dataset->imu_measurements.empty());
    ASSERT_FALSE(dataset->ground_truth_states.empty());

    const auto& initial_ground_truth = dataset->ground_truth_states.front();
    NominalState state = make_state_from_ground_truth(initial_ground_truth);
    StateCovariance covariance = 1e-6 * StateCovariance::Identity();
    TimestampNs previous_timestamp = initial_ground_truth.timestamp;
    const TimestampNs end_timestamp = initial_ground_truth.timestamp + kOneSecondNs;

    const auto first_imu = std::ranges::lower_bound(
        dataset->imu_measurements,
        initial_ground_truth.timestamp,
        {},
        &ImuMeasurement::timestamp);
    ASSERT_NE(first_imu, dataset->imu_measurements.end());

    TimestampNs propagated_timestamp = previous_timestamp;
    for (auto imu = first_imu; imu != dataset->imu_measurements.end() && imu->timestamp <= end_timestamp; ++imu) {
        ASSERT_GE(imu->timestamp, previous_timestamp);
        const double dt_seconds = static_cast<double>(imu->timestamp - previous_timestamp) * 1e-9;
        const auto result = propagate(state, *imu, dataset->imu_calibration, dt_seconds, covariance);
        ASSERT_TRUE(result) << result.error();
        state = result->nominal_state;
        covariance = result->covariance;
        previous_timestamp = imu->timestamp;
        propagated_timestamp = imu->timestamp;
    }

    const auto& final_ground_truth = nearest_ground_truth(dataset->ground_truth_states, propagated_timestamp);
    const double position_error_m = (state.position - final_ground_truth.position).norm();
    const double velocity_error_mps = (state.velocity - final_ground_truth.velocity).norm();
    const double orientation_error_rad =
        (Sophus::SO3d{final_ground_truth.orientation}.inverse() * state.orientation).log().norm();
    const Eigen::SelfAdjointEigenSolver<StateCovariance> eigen_solver(covariance);

    EXPECT_LT(position_error_m, 2.0);
    EXPECT_LT(velocity_error_mps, 3.0);
    EXPECT_LT(orientation_error_rad, 0.5);
    EXPECT_TRUE(covariance.allFinite());
    EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-8));
    EXPECT_GE(eigen_solver.eigenvalues().minCoeff(), -1e-8);
}

TEST(PropagationTest, RejectsNegativeTimestep) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };

    const auto result = propagate(state, measurement, imu_calibration, -0.005, StateCovariance::Identity());

    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("timestep_seconds"), std::string::npos) << result.error();
}

TEST(PropagationTest, RejectsNonFiniteTimestep) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration();
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.0, 0.0, 9.81},
        .angular_velocity = Eigen::Vector3d::Zero(),
    };

    for (const double timestep_seconds : {std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::infinity()}) {
        const auto result =
            propagate(state, measurement, imu_calibration, timestep_seconds, StateCovariance::Identity());

        ASSERT_FALSE(result) << "accepted timestep " << timestep_seconds;
        EXPECT_NE(result.error().find("timestep_seconds"), std::string::npos) << result.error();
    }
}

// A zero timestep is a legitimate no-op (duplicate IMU timestamps), so it is
// accepted rather than rejected alongside the negative case.
TEST(PropagationTest, AcceptsZeroTimestepAsNoOp) {
    const auto state = make_state();
    const auto imu_calibration = make_imu_calibration(0.02, 0.03, 0.004, 0.005);
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{1.0, 2.0, 12.81},
        .angular_velocity = Eigen::Vector3d{0.1, 0.2, 0.3},
    };
    const StateCovariance covariance = StateCovariance::Identity();

    const auto result = propagate(state, measurement, imu_calibration, 0.0, covariance);

    ASSERT_TRUE(result) << result.error();
    EXPECT_TRUE(result->nominal_state.position.isApprox(state.position, kTolerance));
    EXPECT_TRUE(result->nominal_state.velocity.isApprox(state.velocity, kTolerance));
    EXPECT_TRUE(result->covariance.isApprox(covariance, kTolerance));
}

namespace {

// Monte Carlo consistency fixture.
//
// Block-value assertions above run with an identity orientation and zero angular
// velocity, which makes R indistinguishable from R^T and zeroes [w]x — so they
// cannot detect a transposed or sign-flipped block in F. This test instead
// compares the propagated covariance against the sample covariance of perturbed
// trajectories, using the nominal integrator (validated analytically in
// tests/synthetic_test.cpp) as the truth surrogate.
constexpr int kMonteCarloSamples = 6000;
constexpr int kMonteCarloSteps = 200;
constexpr double kMonteCarloTimestep = 0.005;
constexpr std::uint64_t kMonteCarloSeed = 1;
// Correct code lands at ~0.04 across seeds; the floor is Phi = I + F dt truncation,
// not sampling noise, so more samples will not lower it. Corrupting any block of F
// scores >= 0.23.
constexpr double kMonteCarloTolerance = 0.15;

// Rotation and angular velocity are deliberately non-trivial: an identity
// orientation hides R vs R^T, and zero angular velocity hides the [w]x block.
NominalState make_excited_state() {
    return {
        .position = Eigen::Vector3d{1.0, -2.0, 0.5},
        .velocity = Eigen::Vector3d{0.3, 0.1, -0.2},
        .orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.4, -0.3, 0.7}),
        .accelerometer_bias = Eigen::Vector3d{0.02, -0.01, 0.03},
        .gyroscope_bias = Eigen::Vector3d{-0.005, 0.004, -0.003},
    };
}

// Non-zero bias uncertainty is what gives the bias-coupling blocks of F any
// weight; starting from P = 0 leaves a sign flip in them invisible.
Eigen::Matrix<double, kErrorStateSize, 1> make_initial_stddev() {
    Eigen::Matrix<double, kErrorStateSize, 1> stddev;
    stddev.segment<3>(kPositionIndex).setConstant(0.10);
    stddev.segment<3>(kVelocityIndex).setConstant(0.10);
    stddev.segment<3>(kOrientationIndex).setConstant(0.05);
    stddev.segment<3>(kAccelerometerBiasIndex).setConstant(0.05);
    stddev.segment<3>(kGyroscopeBiasIndex).setConstant(0.01);
    return stddev;
}

// Local error convention: R_sample = R_nominal * Exp(dtheta), matching the
// right-multiplied nominal update in propagate(...).
Eigen::Matrix<double, kErrorStateSize, 1> error_state_of(
    const NominalState& sample,
    const NominalState& nominal) {
    Eigen::Matrix<double, kErrorStateSize, 1> error;
    error.segment<3>(kPositionIndex) = sample.position - nominal.position;
    error.segment<3>(kVelocityIndex) = sample.velocity - nominal.velocity;
    error.segment<3>(kOrientationIndex) = (nominal.orientation.inverse() * sample.orientation).log();
    error.segment<3>(kAccelerometerBiasIndex) = sample.accelerometer_bias - nominal.accelerometer_bias;
    error.segment<3>(kGyroscopeBiasIndex) = sample.gyroscope_bias - nominal.gyroscope_bias;
    return error;
}

}  // namespace

TEST(PropagationTest, PropagatedCovarianceMatchesMonteCarloErrorCovariance) {
    const auto imu_calibration = make_imu_calibration(0.02, 0.003, 0.004, 0.0005);
    const ImuMeasurement measurement{
        .timestamp = 1403636579758555392,
        .acceleration = Eigen::Vector3d{0.6, -0.9, 9.5},
        .angular_velocity = Eigen::Vector3d{0.35, -0.25, 0.45},
    };
    const auto initial_state = make_excited_state();
    const auto initial_stddev = make_initial_stddev();

    NominalState nominal = initial_state;
    StateCovariance covariance = initial_stddev.cwiseProduct(initial_stddev).asDiagonal();
    for (int step = 0; step < kMonteCarloSteps; ++step) {
        const auto result = propagate(nominal, measurement, imu_calibration, kMonteCarloTimestep, covariance);
        ASSERT_TRUE(result) << result.error();
        nominal = result->nominal_state;
        covariance = result->covariance;
    }

    // EuRoC reports continuous-time densities, so convert to per-step deviations.
    const double accel_stddev = imu_calibration.accelerometer_noise_density / std::sqrt(kMonteCarloTimestep);
    const double gyro_stddev = imu_calibration.gyroscope_noise_density / std::sqrt(kMonteCarloTimestep);
    const double accel_bias_stddev = imu_calibration.accelerometer_random_walk * std::sqrt(kMonteCarloTimestep);
    const double gyro_bias_stddev = imu_calibration.gyroscope_random_walk * std::sqrt(kMonteCarloTimestep);

    std::mt19937_64 rng{kMonteCarloSeed};
    std::normal_distribution<double> standard_normal{0.0, 1.0};
    const auto sample_vector = [&rng, &standard_normal] {
        return Eigen::Vector3d{standard_normal(rng), standard_normal(rng), standard_normal(rng)};
    };

    Eigen::Matrix<double, kErrorStateSize, 1> mean_error = Eigen::Matrix<double, kErrorStateSize, 1>::Zero();
    StateCovariance empirical = StateCovariance::Zero();
    for (int sample = 0; sample < kMonteCarloSamples; ++sample) {
        Eigen::Matrix<double, kErrorStateSize, 1> initial_error;
        for (int i = 0; i < kErrorStateSize; ++i) {
            initial_error(i) = standard_normal(rng) * initial_stddev(i);
        }

        NominalState state = initial_state;
        state.position += initial_error.segment<3>(kPositionIndex);
        state.velocity += initial_error.segment<3>(kVelocityIndex);
        state.orientation = initial_state.orientation
            * Sophus::SO3d::exp(initial_error.segment<3>(kOrientationIndex));
        state.accelerometer_bias += initial_error.segment<3>(kAccelerometerBiasIndex);
        state.gyroscope_bias += initial_error.segment<3>(kGyroscopeBiasIndex);

        StateCovariance ignored = StateCovariance::Zero();
        for (int step = 0; step < kMonteCarloSteps; ++step) {
            ImuMeasurement noisy = measurement;
            noisy.acceleration += sample_vector() * accel_stddev;
            noisy.angular_velocity += sample_vector() * gyro_stddev;

            const auto result = propagate(state, noisy, imu_calibration, kMonteCarloTimestep, ignored);
            ASSERT_TRUE(result) << result.error();
            state = result->nominal_state;
            // Bias random walk is not part of nominal propagation, so drive it here.
            state.accelerometer_bias += sample_vector() * accel_bias_stddev;
            state.gyroscope_bias += sample_vector() * gyro_bias_stddev;
        }

        const auto error = error_state_of(state, nominal);
        mean_error += error;
        empirical += error * error.transpose();
    }

    mean_error /= kMonteCarloSamples;
    empirical = empirical / kMonteCarloSamples - mean_error * mean_error.transpose();

    // Normalize by the filter's own standard deviations so every block is compared
    // on one scale. The comparison stays signed, so a flipped block cannot pass.
    const Eigen::Matrix<double, kErrorStateSize, 1> inverse_stddev =
        covariance.diagonal().cwiseSqrt().cwiseInverse();
    const StateCovariance normalizer = inverse_stddev * inverse_stddev.transpose();
    const double deviation = (empirical - covariance).cwiseProduct(normalizer).cwiseAbs().maxCoeff();

    EXPECT_LT(deviation, kMonteCarloTolerance)
        << "propagated covariance disagrees with the Monte Carlo error covariance";
}
