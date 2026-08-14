#pragma once

#include "parser.hpp"
#include "state.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

inline constexpr int kRobotDim = kImuErrorStateSize;
inline constexpr int kLandmarkDim = 3;
using LandmarkId = std::int64_t;

struct SlamStateTestAccess;

class SlamState {
public:
    using RobotBlock = Eigen::Block<Eigen::MatrixXd, kRobotDim, kRobotDim>;
    using ConstRobotBlock = Eigen::Block<const Eigen::MatrixXd, kRobotDim, kRobotDim>;
    using ActiveBlock = Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>;
    using ConstActiveBlock = Eigen::Block<const Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>;
    using RobotLandmarkBlock = Eigen::Block<Eigen::MatrixXd, kRobotDim, Eigen::Dynamic>;
    using ConstRobotLandmarkBlock = Eigen::Block<const Eigen::MatrixXd, kRobotDim, Eigen::Dynamic>;
    using LandmarkLandmarkBlock = Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>;
    using ConstLandmarkLandmarkBlock = Eigen::Block<const Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>;
    using LandmarkBlock = Eigen::Block<Eigen::MatrixXd, kLandmarkDim, kLandmarkDim>;
    using ConstLandmarkBlock = Eigen::Block<const Eigen::MatrixXd, kLandmarkDim, kLandmarkDim>;

    static ParseResult<SlamState> create(
        std::size_t max_landmarks,
        const NominalState& initial_robot,
        const ImuStateCovariance& initial_robot_covariance);

    std::size_t max_landmarks() const;
    std::size_t active_landmarks() const;
    int active_dim() const;
    int storage_dim() const;

    ParseResult<LandmarkBlock> landmark_block(std::size_t storage_index);
    ParseResult<ConstLandmarkBlock> landmark_block(std::size_t storage_index) const;

    RobotBlock robot_covariance();
    ConstRobotBlock robot_covariance() const;
    ActiveBlock active_covariance();
    ConstActiveBlock active_covariance() const;
    RobotLandmarkBlock robot_landmark_covariance();
    ConstRobotLandmarkBlock robot_landmark_covariance() const;
    LandmarkLandmarkBlock landmark_landmark_covariance();
    ConstLandmarkLandmarkBlock landmark_landmark_covariance() const;
    ParseResult<void> set_robot_landmark_covariance(
        const Eigen::Ref<const Eigen::MatrixXd>& covariance);
    ParseResult<void> apply_robot_transition(const ImuStateCovariance& transition);

    ParseResult<void> add_landmark(
        LandmarkId id,
        const Eigen::Vector3d& position,
        const Eigen::MatrixXd& covariance_column);
    ParseResult<int> landmark_offset(LandmarkId id) const;
    ParseResult<Eigen::Vector3d> landmark_position(LandmarkId id) const;
    ParseResult<void> remove_landmarks(std::span<const LandmarkId> ids);

    NominalState robot;

private:
    friend struct SlamStateTestAccess;

    SlamState(
        std::size_t max_landmarks,
        int storage_dim,
        const NominalState& initial_robot,
        const ImuStateCovariance& initial_robot_covariance);

    static int storage_dim_for(std::size_t max_landmarks);
    static int landmark_offset_for_storage_index(std::size_t storage_index);

    std::vector<bool> make_removal_mask(const std::vector<std::size_t>& removed_indices) const;

    Eigen::MatrixXd covariance_;
    Eigen::Matrix<double, kLandmarkDim, Eigen::Dynamic> landmark_positions_;
    std::unordered_map<LandmarkId, std::size_t> landmark_indices_;
    std::vector<LandmarkId> storage_index_to_id_;
    int storage_dim_ = kRobotDim;
    std::size_t max_landmarks_ = 0;
    std::size_t active_landmarks_ = 0;
};
