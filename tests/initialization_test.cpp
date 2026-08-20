#include "initialization.hpp"

#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace {

ImuMeasurement stationary_sample(
    const Sophus::SO3d& orientation, const Eigen::Vector3d& gyro_bias, double acceleration_magnitude = 9.81) {
    return ImuMeasurement{
        .timestamp = 0,
        .acceleration = orientation.inverse() * (acceleration_magnitude * Eigen::Vector3d::UnitZ()),
        .angular_velocity = gyro_bias,
    };
}

}  // namespace

TEST(StaticInitializationTest, RecoversTiltAndGyroscopeBiasFromStationaryMeasurements) {
    const Sophus::SO3d orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.2, -0.1, 0.5});
    const Eigen::Vector3d gyro_bias{0.01, -0.02, 0.03};
    std::vector<ImuMeasurement> measurements(100, stationary_sample(orientation, gyro_bias));

    const auto result = initialize_static_imu(measurements, StaticInitializationOptions{
        .initial_position_world = Eigen::Vector3d{1.0, 2.0, 3.0},
        .heading_reference = orientation,
    });

    ASSERT_TRUE(result) << result.error();
    EXPECT_TRUE(result->orientation.matrix().isApprox(orientation.matrix(), 1e-12));
    EXPECT_TRUE(result->gyroscope_bias.isApprox(gyro_bias, 1e-12));
    EXPECT_TRUE(result->velocity.isZero(1e-12));
    EXPECT_TRUE(result->position.isApprox(Eigen::Vector3d{1.0, 2.0, 3.0}, 1e-12));
}

TEST(StaticInitializationTest, RejectsTooFewMeasurements) {
    std::vector<ImuMeasurement> too_few(99, stationary_sample(Sophus::SO3d{}, Eigen::Vector3d::Zero()));

    const auto result = initialize_static_imu(too_few);

    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("stationary_measurements"), std::string::npos) << result.error();
}

TEST(StaticInitializationTest, RejectsNonFiniteMeasurements) {
    std::vector<ImuMeasurement> invalid(100, stationary_sample(Sophus::SO3d{}, Eigen::Vector3d::Zero()));
    invalid.front().acceleration.x() = std::numeric_limits<double>::quiet_NaN();

    const auto result = initialize_static_imu(invalid);

    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("finite"), std::string::npos) << result.error();
}

TEST(StaticInitializationTest, RejectsAMovingInterval) {
    std::vector<ImuMeasurement> measurements(
        100, stationary_sample(Sophus::SO3d{}, Eigen::Vector3d{0.1, 0.0, 0.0}));

    const auto result = initialize_static_imu(measurements);

    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("angular velocity"), std::string::npos) << result.error();
}

TEST(StaticInitializationTest, RejectsExcessiveAccelerationMagnitude) {
    // Free fall or a mis-scaled sensor: gyro looks stationary but the
    // measured specific force is nowhere near 1 g, so this is not a valid
    // static interval either.
    std::vector<ImuMeasurement> measurements(
        100, stationary_sample(Sophus::SO3d{}, Eigen::Vector3d::Zero(), /*acceleration_magnitude=*/5.0));

    const auto result = initialize_static_imu(measurements);

    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("acceleration magnitude"), std::string::npos) << result.error();
}
