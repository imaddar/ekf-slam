#include "slam_state.hpp"

#include <climits>
#include <cstddef>
#include <array>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

struct SlamStateTestAccess {
    static const Eigen::MatrixXd& covariance(const SlamState& state) {
        return state.covariance_;
    }
};

namespace {

NominalState make_initial_robot() {
    return NominalState{
        .position = Eigen::Vector3d{1.0, -2.0, 3.0},
        .velocity = Eigen::Vector3d{-4.0, 5.0, -6.0},
        .orientation = Sophus::SO3d::exp(Eigen::Vector3d{0.1, -0.2, 0.3}),
        .accelerometer_bias = Eigen::Vector3d{0.01, -0.02, 0.03},
        .gyroscope_bias = Eigen::Vector3d{-0.04, 0.05, -0.06},
    };
}

ImuStateCovariance make_initial_covariance() {
    ImuStateCovariance covariance = ImuStateCovariance::Zero();
    covariance.diagonal() = Eigen::Vector<double, kRobotDim>::LinSpaced(kRobotDim, 1.0, 15.0);
    return covariance;
}

auto make_state(std::size_t max_landmarks) {
    return SlamState::create(max_landmarks, make_initial_robot(), make_initial_covariance());
}

Eigen::MatrixXd make_landmark_covariance_column(const SlamState& state, double seed) {
    Eigen::MatrixXd covariance_column(state.active_dim() + kLandmarkDim, kLandmarkDim);
    for (int row = 0; row < covariance_column.rows(); ++row) {
        for (int column = 0; column < covariance_column.cols(); ++column) {
            covariance_column(row, column) = seed + row * 10.0 + column;
        }
    }
    return covariance_column;
}

}

TEST(SlamStateTest, PreservesExplicitRobotInitialization) {
    const NominalState expected_robot = make_initial_robot();
    const ImuStateCovariance expected_covariance = make_initial_covariance();
    const auto result = SlamState::create(0, expected_robot, expected_covariance);

    ASSERT_TRUE(result) << result.error();
    EXPECT_TRUE(result->robot.position.isApprox(expected_robot.position, 0.0));
    EXPECT_TRUE(result->robot.velocity.isApprox(expected_robot.velocity, 0.0));
    EXPECT_TRUE(result->robot.orientation.matrix().isApprox(expected_robot.orientation.matrix(), 0.0));
    EXPECT_TRUE(result->robot.accelerometer_bias.isApprox(expected_robot.accelerometer_bias, 0.0));
    EXPECT_TRUE(result->robot.gyroscope_bias.isApprox(expected_robot.gyroscope_bias, 0.0));
    EXPECT_TRUE(result->robot_covariance().isApprox(expected_covariance, 0.0));
    EXPECT_TRUE(result->active_covariance().isApprox(expected_covariance, 0.0));
}

TEST(SlamStateTest, PreallocatesFullCovarianceForLandmarkBudget) {
    constexpr std::size_t kMaxLandmarks = 4;
    const auto result = make_state(kMaxLandmarks);

    ASSERT_TRUE(result) << result.error();
    EXPECT_EQ(result->max_landmarks(), kMaxLandmarks);
    EXPECT_EQ(result->storage_dim(), kRobotDim + 4 * kLandmarkDim);
    EXPECT_EQ(result->active_landmarks(), 0U);
    EXPECT_EQ(result->active_dim(), kRobotDim);
    EXPECT_EQ(result->active_covariance().rows(), kRobotDim);
    EXPECT_EQ(result->active_covariance().cols(), kRobotDim);
}

TEST(SlamStateTest, RejectsLandmarkCapacityThatCannotFitInEigenDimension) {
    const auto result = make_state(static_cast<std::size_t>(INT_MAX));

    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("max_landmarks"), std::string::npos) << result.error();
}

TEST(SlamStateTest, RobotCovarianceBlockRoundTripsThroughAccessor) {
    const auto result = make_state(3);

    ASSERT_TRUE(result) << result.error();
    SlamState state = std::move(*result);
    const Eigen::Matrix<double, kRobotDim, kRobotDim> expected =
        Eigen::Matrix<double, kRobotDim, kRobotDim>::Identity();

    state.robot_covariance() = expected;

    EXPECT_TRUE(state.robot_covariance().isApprox(expected, 0.0));
    EXPECT_TRUE(state.active_covariance().isApprox(expected, 0.0));
}

TEST(SlamStateTest, PoisonsInactiveCovarianceRegion) {
    const auto result = make_state(2);

    ASSERT_TRUE(result) << result.error();
    const auto& covariance = SlamStateTestAccess::covariance(*result);
    const int storage_dim = covariance.rows();
    const int inactive_dim = storage_dim - kRobotDim;

    ASSERT_GT(inactive_dim, 0);
    EXPECT_TRUE(covariance.topLeftCorner(kRobotDim, kRobotDim).allFinite());
    EXPECT_TRUE(covariance.block(kRobotDim, 0, inactive_dim, storage_dim).array().isNaN().all());
    EXPECT_TRUE(covariance.block(0, kRobotDim, kRobotDim, inactive_dim).array().isNaN().all());
}

TEST(SlamStateTest, LandmarkBlockAccessorUsesCapacityAndCompileTimeDimensions) {
    const auto result = make_state(3);

    ASSERT_TRUE(result) << result.error();
    SlamState state = std::move(*result);
    const Eigen::Matrix3d expected =
        (Eigen::Matrix3d{} << 1.0, 2.0, 3.0,
            4.0, 5.0, 6.0,
            7.0, 8.0, 9.0)
            .finished();

    auto block = state.landmark_block(2);
    ASSERT_TRUE(block) << block.error();
    (*block) = expected;

    const auto readback = std::as_const(state).landmark_block(2);
    ASSERT_TRUE(readback) << readback.error();
    EXPECT_TRUE((*readback).isApprox(expected, 0.0));

    const auto out_of_capacity = state.landmark_block(3);
    ASSERT_FALSE(out_of_capacity);
    EXPECT_NE(out_of_capacity.error().find("landmark_block"), std::string::npos);
}

TEST(SlamStateTest, AddingLandmarksBuildsFiniteActiveStorageWithoutReallocation) {
    const auto result = make_state(3);

    ASSERT_TRUE(result) << result.error();
    SlamState state = std::move(*result);
    const auto* allocation = state.active_covariance().data();
    const std::array<Eigen::Vector3d, 2> positions{
        Eigen::Vector3d{1.0, 2.0, 3.0},
        Eigen::Vector3d{-4.0, 5.0, -6.0}};

    ASSERT_TRUE(state.add_landmark(10, positions[0], make_landmark_covariance_column(state, 100.0)));
    EXPECT_EQ(state.active_landmarks(), 1U);
    EXPECT_EQ(state.active_dim(), kRobotDim + kLandmarkDim);
    EXPECT_EQ(state.active_covariance().data(), allocation);
    EXPECT_TRUE(state.active_covariance().allFinite());

    ASSERT_TRUE(state.add_landmark(20, positions[1], make_landmark_covariance_column(state, 200.0)));
    EXPECT_EQ(state.active_landmarks(), 2U);
    EXPECT_EQ(state.active_covariance().data(), allocation);
    EXPECT_TRUE(state.active_covariance().allFinite());
}

TEST(SlamStateTest, LandmarkIdsResolveToOffsetsAndPositions) {
    const auto result = make_state(3);

    ASSERT_TRUE(result) << result.error();
    SlamState state = std::move(*result);
    const Eigen::Vector3d position{7.0, -8.0, 9.0};

    ASSERT_TRUE(state.add_landmark(42, position, make_landmark_covariance_column(state, 100.0)));
    const auto offset = state.landmark_offset(42);
    const auto stored_position = state.landmark_position(42);

    ASSERT_TRUE(offset) << offset.error();
    ASSERT_TRUE(stored_position) << stored_position.error();
    EXPECT_EQ(*offset, kRobotDim);
    EXPECT_TRUE(stored_position->isApprox(position, 0.0));

    const auto missing_offset = state.landmark_offset(99);
    ASSERT_FALSE(missing_offset);
    EXPECT_NE(missing_offset.error().find("landmark_id"), std::string::npos);
}

TEST(SlamStateTest, RejectsDuplicateMissingAndOverCapacityLandmarks) {
    const auto result = make_state(1);

    ASSERT_TRUE(result) << result.error();
    SlamState state = std::move(*result);
    const Eigen::Vector3d position{1.0, 2.0, 3.0};

    ASSERT_TRUE(state.add_landmark(1, position, make_landmark_covariance_column(state, 100.0)));
    const auto duplicate = state.add_landmark(
        1, position, make_landmark_covariance_column(state, 200.0));
    ASSERT_FALSE(duplicate);
    EXPECT_NE(duplicate.error().find("duplicate"), std::string::npos);

    const auto full = state.add_landmark(2, position, make_landmark_covariance_column(state, 300.0));
    ASSERT_FALSE(full);
    EXPECT_NE(full.error().find("capacity"), std::string::npos);

    const std::array<LandmarkId, 1> missing{99};
    const auto remove_missing = state.remove_landmarks(missing);
    ASSERT_FALSE(remove_missing);
    EXPECT_NE(remove_missing.error().find("unknown"), std::string::npos);
}

TEST(SlamStateTest, BatchRemovalCompactsSurvivorsAndRebuildsOffsets) {
    const auto result = make_state(4);

    ASSERT_TRUE(result) << result.error();
    SlamState state = std::move(*result);
    const std::array<LandmarkId, 4> ids{10, 20, 30, 40};
    const std::array<Eigen::Vector3d, 4> positions{
        Eigen::Vector3d{1.0, 2.0, 3.0},
        Eigen::Vector3d{4.0, 5.0, 6.0},
        Eigen::Vector3d{7.0, 8.0, 9.0},
        Eigen::Vector3d{10.0, 11.0, 12.0}};

    for (std::size_t index = 0; index < ids.size(); ++index) {
        ASSERT_TRUE(state.add_landmark(
            ids[index], positions[index], make_landmark_covariance_column(state, 100.0 + index)));
    }

    const int old_active_dim = state.active_dim();
    Eigen::MatrixXd before = state.active_covariance();
    for (int row = 0; row < old_active_dim; ++row) {
        for (int column = 0; column < old_active_dim; ++column) {
            before(row, column) = static_cast<double>(row * old_active_dim + column + 1);
        }
    }
    state.active_covariance() = before;

    const std::array<LandmarkId, 2> removed{20, 40};
    ASSERT_TRUE(state.remove_landmarks(removed));
    EXPECT_EQ(state.active_landmarks(), 2U);
    EXPECT_EQ(state.active_dim(), kRobotDim + 2 * kLandmarkDim);
    EXPECT_TRUE(state.active_covariance().allFinite());

    const auto first_offset = state.landmark_offset(10);
    const auto second_offset = state.landmark_offset(30);
    ASSERT_TRUE(first_offset) << first_offset.error();
    ASSERT_TRUE(second_offset) << second_offset.error();
    EXPECT_EQ(*first_offset, kRobotDim);
    EXPECT_EQ(*second_offset, kRobotDim + kLandmarkDim);

    const auto first_position = state.landmark_position(10);
    const auto second_position = state.landmark_position(30);
    ASSERT_TRUE(first_position) << first_position.error();
    ASSERT_TRUE(second_position) << second_position.error();
    EXPECT_TRUE(first_position->isApprox(positions[0], 0.0));
    EXPECT_TRUE(second_position->isApprox(positions[2], 0.0));

    Eigen::MatrixXd expected = Eigen::MatrixXd::Zero(state.active_dim(), state.active_dim());
    const std::array<int, 2> survivor_offsets{0, 2};
    expected.topLeftCorner<kRobotDim, kRobotDim>() = before.topLeftCorner<kRobotDim, kRobotDim>();
    for (std::size_t new_index = 0; new_index < survivor_offsets.size(); ++new_index) {
        const int new_offset = kRobotDim + static_cast<int>(new_index) * kLandmarkDim;
        const int old_offset = kRobotDim + survivor_offsets[new_index] * kLandmarkDim;
        expected.block<kRobotDim, kLandmarkDim>(0, new_offset) = before.block<kRobotDim, kLandmarkDim>(0, old_offset);
        expected.block<kLandmarkDim, kRobotDim>(new_offset, 0) = before.block<kLandmarkDim, kRobotDim>(old_offset, 0);
        expected.block<kLandmarkDim, kLandmarkDim>(new_offset, new_offset) =
            before.block<kLandmarkDim, kLandmarkDim>(old_offset, old_offset);
        for (std::size_t previous = 0; previous < new_index; ++previous) {
            const int previous_new_offset = kRobotDim + static_cast<int>(previous) * kLandmarkDim;
            const int previous_old_offset = kRobotDim + survivor_offsets[previous] * kLandmarkDim;
            expected.block<kLandmarkDim, kLandmarkDim>(previous_new_offset, new_offset) =
                before.block<kLandmarkDim, kLandmarkDim>(previous_old_offset, old_offset);
            expected.block<kLandmarkDim, kLandmarkDim>(new_offset, previous_new_offset) =
                before.block<kLandmarkDim, kLandmarkDim>(old_offset, previous_old_offset);
        }
    }
    EXPECT_TRUE(state.active_covariance().isApprox(expected, 0.0));

    const auto& storage = SlamStateTestAccess::covariance(state);
    const int inactive_dim = state.storage_dim() - state.active_dim();
    ASSERT_GT(inactive_dim, 0);
    EXPECT_TRUE(storage.block(state.active_dim(), 0, inactive_dim, state.storage_dim()).array().isNaN().all());
    EXPECT_TRUE(storage.block(0, state.active_dim(), state.active_dim(), inactive_dim).array().isNaN().all());
}

TEST(SlamStateTest, RequiresFiniteCompleteLandmarkCovarianceColumn) {
    const auto result = make_state(1);

    ASSERT_TRUE(result) << result.error();
    SlamState state = std::move(*result);
    const Eigen::Vector3d position{1.0, 2.0, 3.0};
    const Eigen::MatrixXd wrong_shape = Eigen::MatrixXd::Zero(kRobotDim, kLandmarkDim);

    const auto shape_error = state.add_landmark(1, position, wrong_shape);
    ASSERT_FALSE(shape_error);
    EXPECT_NE(shape_error.error().find("covariance_column"), std::string::npos);
    EXPECT_EQ(state.active_landmarks(), 0U);

    Eigen::MatrixXd nonfinite = make_landmark_covariance_column(state, 100.0);
    nonfinite(0, 0) = std::numeric_limits<double>::quiet_NaN();
    const auto finite_error = state.add_landmark(1, position, nonfinite);
    ASSERT_FALSE(finite_error);
    EXPECT_NE(finite_error.error().find("finite"), std::string::npos);
    EXPECT_EQ(state.active_landmarks(), 0U);
}

TEST(SlamStateTest, AddRemoveAddCompactsDeterministically) {
    const auto result = make_state(2);

    ASSERT_TRUE(result) << result.error();
    SlamState state = std::move(*result);
    ASSERT_TRUE(state.add_landmark(
        1, Eigen::Vector3d{1.0, 0.0, 0.0}, make_landmark_covariance_column(state, 100.0)));
    ASSERT_TRUE(state.add_landmark(
        2, Eigen::Vector3d{2.0, 0.0, 0.0}, make_landmark_covariance_column(state, 200.0)));

    const std::array<LandmarkId, 1> removed{1};
    ASSERT_TRUE(state.remove_landmarks(removed));
    ASSERT_TRUE(state.add_landmark(
        3, Eigen::Vector3d{3.0, 0.0, 0.0}, make_landmark_covariance_column(state, 300.0)));

    const auto offset_two = state.landmark_offset(2);
    const auto offset_three = state.landmark_offset(3);
    ASSERT_TRUE(offset_two) << offset_two.error();
    ASSERT_TRUE(offset_three) << offset_three.error();
    EXPECT_EQ(*offset_two, kRobotDim);
    EXPECT_EQ(*offset_three, kRobotDim + kLandmarkDim);
}
