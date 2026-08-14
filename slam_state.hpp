#pragma once

#include "parser.hpp"
#include "state.hpp"

#include <cstddef>
#include <format>
#include <limits>

#include <Eigen/Core>

inline constexpr int kRobotDim = kImuErrorStateSize;
inline constexpr int kLandmarkDim = 3;

class SlamState {
public:
    static ParseResult<SlamState> create(
        std::size_t max_landmarks,
        const ImuStateCovariance& initial_robot_covariance) {
        constexpr std::size_t kMaxLandmarks =
            (static_cast<std::size_t>(std::numeric_limits<int>::max()) - kRobotDim) / kLandmarkDim;
        if (max_landmarks > kMaxLandmarks) {
            return std::unexpected(std::format(
                "max_landmarks: expected <= {}, found {}",
                kMaxLandmarks,
                max_landmarks));
        }

        return SlamState{
            max_landmarks,
            storage_dim_for(max_landmarks),
            initial_robot_covariance};
    }

    std::size_t max_landmarks() const { return max_landmarks_; }
    std::size_t active_landmarks() const { return active_landmarks_; }

    int active_dim() const {
        return kRobotDim + static_cast<int>(active_landmarks_) * kLandmarkDim;
    }

    int storage_dim() const { return storage_dim_; }

    auto robot_covariance() {
        return covariance_.template topLeftCorner<kRobotDim, kRobotDim>();
    }

    auto robot_covariance() const {
        return covariance_.template topLeftCorner<kRobotDim, kRobotDim>();
    }

    auto active_covariance() {
        return covariance_.topLeftCorner(active_dim(), active_dim());
    }

    auto active_covariance() const {
        return covariance_.topLeftCorner(active_dim(), active_dim());
    }

    NominalState robot{};

private:
    SlamState(
        std::size_t max_landmarks,
        int storage_dim,
        const ImuStateCovariance& initial_robot_covariance)
        : covariance_{storage_dim, storage_dim},
          storage_dim_{storage_dim},
          max_landmarks_{max_landmarks} {
        robot_covariance() = initial_robot_covariance;
    }

    static int storage_dim_for(std::size_t max_landmarks) {
        return kRobotDim + static_cast<int>(max_landmarks) * kLandmarkDim;
    }

    Eigen::MatrixXd covariance_;
    int storage_dim_ = kRobotDim;
    std::size_t max_landmarks_ = 0;
    std::size_t active_landmarks_ = 0;
};
