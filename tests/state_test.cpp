#include "state.hpp"

#include <type_traits>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <sophus/so3.hpp>

TEST(StateTypesTest, DeclaresNominalState) {
    EXPECT_TRUE(std::is_class_v<NominalState>);
}

TEST(StateTypesTest, NominalStateContainsPoseVelocityAndBiases) {
    EXPECT_TRUE((std::is_same_v<decltype(&NominalState::position), Eigen::Vector3d NominalState::*>));
    EXPECT_TRUE((std::is_same_v<decltype(&NominalState::velocity), Eigen::Vector3d NominalState::*>));
    EXPECT_TRUE((std::is_same_v<decltype(&NominalState::orientation), Sophus::SO3d NominalState::*>));
    EXPECT_TRUE((std::is_same_v<decltype(&NominalState::accelerometer_bias), Eigen::Vector3d NominalState::*>));
    EXPECT_TRUE((std::is_same_v<decltype(&NominalState::gyroscope_bias), Eigen::Vector3d NominalState::*>));
}
