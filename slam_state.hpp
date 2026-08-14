#pragma once

#include "parser.hpp"
#include "state.hpp"

#include <cstddef>
#include <format>

#include <Eigen/Core>

inline constexpr int kRobotDim = kImuErrorStateSize;
inline constexpr int kLandmarkDim = 3;

struct SlamState {
    explicit SlamState(std::size_t max_landmarks)
        : covariance{Eigen::MatrixXd::Zero(storage_dim_for(max_landmarks), storage_dim_for(max_landmarks))},
          max_landmarks_{max_landmarks} {}

    std::size_t max_landmarks() const { return max_landmarks_; }
    std::size_t active_landmarks() const { return active_landmarks_; }

    int active_dim() const {
        return kRobotDim + static_cast<int>(active_landmarks_) * kLandmarkDim;
    }

    int storage_dim() const {
        return kRobotDim + static_cast<int>(max_landmarks_) * kLandmarkDim;
    }

    ParseResult<void> set_active_landmarks(std::size_t active_landmarks) {
        if (active_landmarks > max_landmarks_) {
            return std::unexpected(std::format(
                "active_landmarks: expected <= max_landmarks {}, found {}",
                max_landmarks_,
                active_landmarks));
        }

        active_landmarks_ = active_landmarks;
        return {};
    }

    int landmark_offset(std::size_t active_landmark_index) const {
        return kRobotDim + static_cast<int>(active_landmark_index) * kLandmarkDim;
    }

    auto active_covariance() {
        return covariance.topLeftCorner(active_dim(), active_dim());
    }

    auto active_covariance() const {
        return covariance.topLeftCorner(active_dim(), active_dim());
    }

    NominalState robot{};
    Eigen::MatrixXd covariance;

private:
    static int storage_dim_for(std::size_t max_landmarks) {
        return kRobotDim + static_cast<int>(max_landmarks) * kLandmarkDim;
    }

    std::size_t max_landmarks_ = 0;
    std::size_t active_landmarks_ = 0;
};
