#include "slam_state.hpp"

#include <algorithm>
#include <format>
#include <limits>
#include <new>

ParseResult<SlamState> SlamState::create(
    std::size_t max_landmarks,
    const NominalState& initial_robot,
    const ImuStateCovariance& initial_robot_covariance) {
    constexpr std::size_t kMaxLandmarks =
        (static_cast<std::size_t>(std::numeric_limits<int>::max()) - kRobotDim) / kLandmarkDim;
    if (max_landmarks > kMaxLandmarks) {
        return std::unexpected(std::format(
            "max_landmarks: expected <= {}, found {}", kMaxLandmarks, max_landmarks));
    }

    try {
        return SlamState{max_landmarks, storage_dim_for(max_landmarks), initial_robot, initial_robot_covariance};
    } catch (const std::bad_alloc&) {
        return std::unexpected("max_landmarks: covariance allocation failed");
    }
}

std::size_t SlamState::max_landmarks() const { return max_landmarks_; }

std::size_t SlamState::active_landmarks() const { return active_landmarks_; }

int SlamState::active_dim() const {
    return kRobotDim + static_cast<int>(active_landmarks_) * kLandmarkDim;
}

int SlamState::storage_dim() const { return storage_dim_; }

ParseResult<SlamState::LandmarkBlock> SlamState::landmark_block(std::size_t storage_index) {
    if (storage_index >= max_landmarks_) {
        return std::unexpected(std::format(
            "landmark_block: expected index < {}, found {}", max_landmarks_, storage_index));
    }
    const int offset = landmark_offset_for_storage_index(storage_index);
    return covariance_.block<kLandmarkDim, kLandmarkDim>(offset, offset);
}

ParseResult<SlamState::ConstLandmarkBlock> SlamState::landmark_block(std::size_t storage_index) const {
    if (storage_index >= max_landmarks_) {
        return std::unexpected(std::format(
            "landmark_block: expected index < {}, found {}", max_landmarks_, storage_index));
    }
    const int offset = landmark_offset_for_storage_index(storage_index);
    return covariance_.block<kLandmarkDim, kLandmarkDim>(offset, offset);
}

SlamState::RobotBlock SlamState::robot_covariance() {
    return covariance_.topLeftCorner<kRobotDim, kRobotDim>();
}

SlamState::ConstRobotBlock SlamState::robot_covariance() const {
    return covariance_.topLeftCorner<kRobotDim, kRobotDim>();
}

SlamState::ActiveBlock SlamState::active_covariance() {
    return covariance_.topLeftCorner(active_dim(), active_dim());
}

SlamState::ConstActiveBlock SlamState::active_covariance() const {
    return covariance_.topLeftCorner(active_dim(), active_dim());
}

ParseResult<void> SlamState::add_landmark(
    LandmarkId id,
    const Eigen::Vector3d& position,
    const Eigen::MatrixXd& covariance_column) {
    if (!position.allFinite()) {
        return std::unexpected("landmark_position: expected finite XYZ coordinates");
    }
    if (landmark_indices_.contains(id)) {
        return std::unexpected(std::format("landmark_id: duplicate id {}", id));
    }
    if (active_landmarks_ >= max_landmarks_) {
        return std::unexpected(std::format(
            "landmark_id: expected capacity {}, found full storage", max_landmarks_));
    }

    const int expected_rows = active_dim() + kLandmarkDim;
    if (covariance_column.rows() != expected_rows || covariance_column.cols() != kLandmarkDim) {
        return std::unexpected(std::format(
            "covariance_column: expected {}x{}, found {}x{}",
            expected_rows,
            kLandmarkDim,
            covariance_column.rows(),
            covariance_column.cols()));
    }
    if (!covariance_column.allFinite()) {
        return std::unexpected("covariance_column: expected finite values");
    }

    const std::size_t storage_index = active_landmarks_;
    try {
        landmark_indices_.emplace(id, storage_index);
    } catch (const std::bad_alloc&) {
        return std::unexpected("landmark_id: registry allocation failed");
    }

    landmark_positions_.col(static_cast<Eigen::Index>(storage_index)) = position;
    const int offset = landmark_offset_for_storage_index(storage_index);
    const int new_active_dim = offset + kLandmarkDim;
    covariance_.block(0, offset, new_active_dim, kLandmarkDim) = covariance_column;
    covariance_.block(offset, 0, kLandmarkDim, new_active_dim) = covariance_column.transpose();
    storage_index_to_id_[storage_index] = id;
    ++active_landmarks_;
    return {};
}

ParseResult<int> SlamState::landmark_offset(LandmarkId id) const {
    const auto iterator = landmark_indices_.find(id);
    if (iterator == landmark_indices_.end()) {
        return std::unexpected(std::format("landmark_id: unknown id {}", id));
    }
    return landmark_offset_for_storage_index(iterator->second);
}

ParseResult<Eigen::Vector3d> SlamState::landmark_position(LandmarkId id) const {
    const auto iterator = landmark_indices_.find(id);
    if (iterator == landmark_indices_.end()) {
        return std::unexpected(std::format("landmark_id: unknown id {}", id));
    }
    return landmark_positions_.col(static_cast<Eigen::Index>(iterator->second));
}

ParseResult<void> SlamState::remove_landmarks(std::span<const LandmarkId> ids) {
    if (ids.empty()) {
        return {};
    }

    std::vector<std::size_t> removed_indices;
    try {
        removed_indices.reserve(ids.size());
        for (const LandmarkId id : ids) {
            const auto iterator = landmark_indices_.find(id);
            if (iterator == landmark_indices_.end()) {
                return std::unexpected(std::format("landmark_id: unknown id {}", id));
            }
            removed_indices.push_back(iterator->second);
        }

        std::sort(removed_indices.begin(), removed_indices.end());
        if (std::adjacent_find(removed_indices.begin(), removed_indices.end()) != removed_indices.end()) {
            return std::unexpected("landmark_id: duplicate id in removal batch");
        }

        const std::vector<bool> removed = make_removal_mask(removed_indices);
        const std::size_t survivor_count = active_landmarks_ - removed_indices.size();
        std::vector<std::size_t> survivor_indices;
        survivor_indices.reserve(survivor_count);
        std::vector<LandmarkId> compacted_ids;
        compacted_ids.reserve(survivor_count);
        std::unordered_map<LandmarkId, std::size_t> compacted_indices;
        compacted_indices.reserve(survivor_count);

        std::size_t new_index = 0;
        for (std::size_t old_index = 0; old_index < active_landmarks_; ++old_index) {
            if (removed[old_index]) {
                continue;
            }
            const LandmarkId id = storage_index_to_id_[old_index];
            survivor_indices.push_back(old_index);
            compacted_ids.push_back(id);
            compacted_indices.emplace(id, new_index);
            ++new_index;
        }

        // Survivors move only toward the robot block. Compact columns first,
        // then rows; forward order keeps each source block available.
        for (std::size_t survivor = 0; survivor < survivor_count; ++survivor) {
            const std::size_t old_index = survivor_indices[survivor];
            landmark_positions_.col(static_cast<Eigen::Index>(survivor)) =
                landmark_positions_.col(static_cast<Eigen::Index>(old_index));

            const int new_offset = landmark_offset_for_storage_index(survivor);
            const int old_offset = landmark_offset_for_storage_index(old_index);
            const Eigen::Matrix<double, kRobotDim, kLandmarkDim> robot_cross_covariance =
                covariance_.block<kRobotDim, kLandmarkDim>(0, old_offset);
            covariance_.block<kRobotDim, kLandmarkDim>(0, new_offset) = robot_cross_covariance;
            for (std::size_t row = 0; row < active_landmarks_; ++row) {
                const int row_offset = landmark_offset_for_storage_index(row);
                const Eigen::Matrix3d landmark_covariance =
                    covariance_.block<kLandmarkDim, kLandmarkDim>(row_offset, old_offset);
                covariance_.block<kLandmarkDim, kLandmarkDim>(row_offset, new_offset) = landmark_covariance;
            }
        }

        for (std::size_t survivor = 0; survivor < survivor_count; ++survivor) {
            const std::size_t old_index = survivor_indices[survivor];
            const int new_offset = landmark_offset_for_storage_index(survivor);
            const int old_offset = landmark_offset_for_storage_index(old_index);
            const Eigen::Matrix<double, kLandmarkDim, kRobotDim> robot_cross_covariance =
                covariance_.block<kLandmarkDim, kRobotDim>(old_offset, 0);
            covariance_.block<kLandmarkDim, kRobotDim>(new_offset, 0) = robot_cross_covariance;

            for (std::size_t column = 0; column < survivor_count; ++column) {
                const int column_offset = landmark_offset_for_storage_index(column);
                const Eigen::Matrix3d landmark_covariance =
                    covariance_.block<kLandmarkDim, kLandmarkDim>(old_offset, column_offset);
                covariance_.block<kLandmarkDim, kLandmarkDim>(new_offset, column_offset) = landmark_covariance;
            }
        }

        const int new_active_dim = kRobotDim + static_cast<int>(survivor_count) * kLandmarkDim;
        const int tail_dim = storage_dim_ - new_active_dim;
        if (tail_dim > 0) {
            covariance_.block(new_active_dim, 0, tail_dim, storage_dim_)
                .setConstant(std::numeric_limits<double>::quiet_NaN());
            covariance_.block(0, new_active_dim, new_active_dim, tail_dim)
                .setConstant(std::numeric_limits<double>::quiet_NaN());
            landmark_positions_.rightCols(static_cast<Eigen::Index>(max_landmarks_ - survivor_count))
                .setConstant(std::numeric_limits<double>::quiet_NaN());
        }
        for (std::size_t index = 0; index < survivor_count; ++index) {
            storage_index_to_id_[index] = compacted_ids[index];
        }

        landmark_indices_.swap(compacted_indices);
        active_landmarks_ = survivor_count;
    } catch (const std::bad_alloc&) {
        return std::unexpected("landmark_id: compaction allocation failed");
    }

    return {};
}

SlamState::SlamState(
    std::size_t max_landmarks,
    int storage_dim,
    const NominalState& initial_robot,
    const ImuStateCovariance& initial_robot_covariance)
    : robot{initial_robot},
      covariance_{storage_dim, storage_dim},
      landmark_positions_{kLandmarkDim, static_cast<Eigen::Index>(max_landmarks)},
      storage_index_to_id_(max_landmarks),
      storage_dim_{storage_dim},
      max_landmarks_{max_landmarks} {
    covariance_.setConstant(std::numeric_limits<double>::quiet_NaN());
    landmark_positions_.setConstant(std::numeric_limits<double>::quiet_NaN());
    robot_covariance() = initial_robot_covariance;
}

int SlamState::storage_dim_for(std::size_t max_landmarks) {
    return kRobotDim + static_cast<int>(max_landmarks) * kLandmarkDim;
}

int SlamState::landmark_offset_for_storage_index(std::size_t storage_index) {
    return kRobotDim + static_cast<int>(storage_index) * kLandmarkDim;
}

std::vector<bool> SlamState::make_removal_mask(
    const std::vector<std::size_t>& removed_indices) const {
    std::vector<bool> removed(active_landmarks_, false);
    for (const std::size_t index : removed_indices) {
        removed[index] = true;
    }
    return removed;
}
