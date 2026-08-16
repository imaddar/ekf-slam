#include "measurement_update.hpp"

#include <algorithm>
#include <array>
#include <format>
#include <limits>

#include <Eigen/Cholesky>

namespace {

constexpr int kPositionIndex = 0;
constexpr int kOrientationIndex = 6;

using MeasurementRow = Eigen::Matrix<double, kStereoMeasurementDim, Eigen::Dynamic>;
using GainColumn = Eigen::Matrix<double, Eigen::Dynamic, kStereoMeasurementDim>;
using SparseJacobian = Eigen::Matrix<double, kStereoMeasurementDim, kMeasurementSparseColumns>;
using SparseBlock = Eigen::Matrix<double, kMeasurementSparseColumns, kMeasurementSparseColumns>;

// The 9 nonzero columns of a stereo row block, in state-column order.
SparseJacobian compact_jacobian(const StereoMeasurementBlocks& blocks) {
    SparseJacobian sparse;
    sparse.leftCols<kLandmarkDim>() = blocks.pose.leftCols<kLandmarkDim>();
    sparse.middleCols<kLandmarkDim>(kLandmarkDim) = blocks.pose.rightCols<kLandmarkDim>();
    sparse.rightCols<kLandmarkDim>() = blocks.landmark;
    return sparse;
}

template <typename Derived>
SparseJacobian gather_columns(const Eigen::MatrixBase<Derived>& rows, int landmark_offset) {
    SparseJacobian gathered;
    gathered.leftCols<kLandmarkDim>() = rows.template middleCols<kLandmarkDim>(kPositionIndex);
    gathered.middleCols<kLandmarkDim>(kLandmarkDim) =
        rows.template middleCols<kLandmarkDim>(kOrientationIndex);
    gathered.rightCols<kLandmarkDim>() = rows.template middleCols<kLandmarkDim>(landmark_offset);
    return gathered;
}

// Gating needs only H P H^T, which touches the 9x9 sub-block of P at the
// observation's columns. Gathering it avoids snapshotting the full covariance.
template <typename Derived>
SparseBlock gather_prior_block(const Eigen::MatrixBase<Derived>& covariance, int landmark_offset) {
    const std::array<int, 3> starts{kPositionIndex, kOrientationIndex, landmark_offset};
    SparseBlock block;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            block.block<kLandmarkDim, kLandmarkDim>(kLandmarkDim * row, kLandmarkDim * column) =
                covariance.template block<kLandmarkDim, kLandmarkDim>(starts[row], starts[column]);
        }
    }
    return block;
}

// H_i P without materializing H_i: three 4x3 blocks times three row strips of P.
template <typename Derived>
void sparse_h_times_p(
    const StereoMeasurementBlocks& blocks,
    const Eigen::MatrixBase<Derived>& covariance,
    MeasurementRow& out) {
    out.noalias() = blocks.pose.leftCols<kLandmarkDim>()
        * covariance.template middleRows<kLandmarkDim>(kPositionIndex);
    out.noalias() += blocks.pose.rightCols<kLandmarkDim>()
        * covariance.template middleRows<kLandmarkDim>(kOrientationIndex);
    out.noalias() +=
        blocks.landmark * covariance.template middleRows<kLandmarkDim>(blocks.landmark_offset);
}

// M H^T, again touching only the 9 columns the observation reaches.
template <typename Derived>
void sparse_m_times_h_transpose(
    const StereoMeasurementBlocks& blocks,
    const SparseJacobian& sparse,
    const Eigen::MatrixBase<Derived>& covariance,
    GainColumn& out) {
    out.noalias() = covariance.template middleCols<kLandmarkDim>(kPositionIndex)
        * sparse.leftCols<kLandmarkDim>().transpose();
    out.noalias() += covariance.template middleCols<kLandmarkDim>(kOrientationIndex)
        * sparse.middleCols<kLandmarkDim>(kLandmarkDim).transpose();
    out.noalias() += covariance.template middleCols<kLandmarkDim>(blocks.landmark_offset)
        * sparse.rightCols<kLandmarkDim>().transpose();
}

template <typename Derived>
void symmetrize(Eigen::MatrixBase<Derived>& covariance) {
    const int dimension = static_cast<int>(covariance.rows());
    for (int row = 0; row < dimension; ++row) {
        for (int column = row + 1; column < dimension; ++column) {
            const double average = 0.5 * (covariance(row, column) + covariance(column, row));
            covariance(row, column) = average;
            covariance(column, row) = average;
        }
    }
}

struct PreparedObservation {
    StereoMeasurementBlocks blocks;
    SparseJacobian sparse;
    Eigen::Vector4d residual;
    LandmarkId id;
};

}  // namespace

ParseResult<StereoMeasurementBlocks> make_stereo_measurement_blocks(
    const NominalState& robot,
    const Eigen::Vector3d& landmark_world,
    const CameraCalibration& cam0,
    const CameraCalibration& cam1,
    int landmark_offset) {
    const auto cam0_prediction =
        predict_pinhole_pixel(robot.orientation, robot.position, landmark_world, cam0);
    if (!cam0_prediction) {
        return std::unexpected(cam0_prediction.error());
    }
    const auto cam1_prediction =
        predict_pinhole_pixel(robot.orientation, robot.position, landmark_world, cam1);
    if (!cam1_prediction) {
        return std::unexpected(cam1_prediction.error());
    }

    StereoMeasurementBlocks stereo{
        .pose = Eigen::Matrix<double, kStereoMeasurementDim, 6>::Zero(),
        .landmark = Eigen::Matrix<double, kStereoMeasurementDim, kLandmarkDim>::Zero(),
        .prediction = Eigen::Vector4d::Zero(),
        .depth_cam0 = cam0_prediction->landmark_camera.z(),
        .depth_cam1 = cam1_prediction->landmark_camera.z(),
        .landmark_offset = landmark_offset,
        .visible = cam0_prediction->landmark_camera.z() > 0.0
            && cam1_prediction->landmark_camera.z() > 0.0,
    };
    stereo.prediction << cam0_prediction->pixel, cam1_prediction->pixel;
    // A landmark behind either camera still projects to a finite pixel, so the
    // Jacobians are left zero rather than linearized about an invalid point.
    if (!stereo.visible) {
        return stereo;
    }

    const auto cam0_blocks = make_measurement_jacobian_blocks(
        *cam0_prediction, robot.orientation, cam0, landmark_offset);
    if (!cam0_blocks) {
        return std::unexpected(cam0_blocks.error());
    }
    const auto cam1_blocks = make_measurement_jacobian_blocks(
        *cam1_prediction, robot.orientation, cam1, landmark_offset);
    if (!cam1_blocks) {
        return std::unexpected(cam1_blocks.error());
    }

    stereo.pose.topRows<2>() = cam0_blocks->pose;
    stereo.pose.bottomRows<2>() = cam1_blocks->pose;
    stereo.landmark.topRows<2>() = cam0_blocks->landmark;
    stereo.landmark.bottomRows<2>() = cam1_blocks->landmark;
    return stereo;
}

ParseResult<StereoUpdateResult> update_stereo_frame(
    SlamState& state,
    std::span<const StereoObservation> observations,
    const CameraCalibration& cam0,
    const CameraCalibration& cam1,
    const Eigen::Matrix4d& pixel_covariance,
    const UpdateOptions& options) {
    if (!pixel_covariance.allFinite()) {
        return std::unexpected("pixel_covariance: expected finite values");
    }
    // Written as a negated comparison so a NaN threshold is rejected too.
    if (!(options.chi_square_threshold >= 0.0)) {
        return std::unexpected(std::format(
            "chi_square_threshold: expected a non-negative value, found {}",
            options.chi_square_threshold));
    }

    StereoUpdateResult result;
    result.diagnostics.reserve(observations.size());
    std::vector<PreparedObservation> prepared;
    prepared.reserve(observations.size());

    const int dimension = state.active_dim();

    // Phase A: predict, linearize, and gate against the prior. Nothing here
    // writes to P, so the accept/reject partition cannot depend on sweep order
    // and cannot tighten as earlier updates shrink the covariance.
    for (const auto& observation : observations) {
        LandmarkUpdateDiagnostics diagnostics{
            .id = observation.id,
            .outcome = ObservationOutcome::kUnknownLandmark,
            .mahalanobis_distance = std::numeric_limits<double>::quiet_NaN(),
            .innovation = Eigen::Vector4d::Zero(),
        };

        const auto offset = state.landmark_offset(observation.id);
        if (!offset) {
            ++result.skipped_count;
            result.diagnostics.push_back(diagnostics);
            continue;
        }
        const auto position = state.landmark_position(observation.id);
        if (!position) {
            return std::unexpected(position.error());
        }
        const auto blocks =
            make_stereo_measurement_blocks(state.robot, *position, cam0, cam1, *offset);
        if (!blocks) {
            return std::unexpected(blocks.error());
        }
        if (!blocks->visible) {
            diagnostics.outcome = ObservationOutcome::kBehindCamera;
            ++result.skipped_count;
            result.diagnostics.push_back(diagnostics);
            continue;
        }

        Eigen::Vector4d measurement;
        measurement << observation.pixel_cam0, observation.pixel_cam1;
        if (!measurement.allFinite()) {
            return std::unexpected(std::format(
                "observation {}: expected finite stereo pixels", observation.id));
        }
        const Eigen::Vector4d residual = measurement - blocks->prediction;
        diagnostics.innovation = residual;

        const SparseJacobian sparse = compact_jacobian(*blocks);
        const Eigen::Matrix4d innovation_covariance =
            sparse * gather_prior_block(state.active_covariance(), *offset) * sparse.transpose()
            + pixel_covariance;
        const Eigen::LLT<Eigen::Matrix4d> factorization(innovation_covariance);
        if (factorization.info() != Eigen::Success) {
            return std::unexpected(std::format(
                "observation {}: prior innovation covariance is not positive definite",
                observation.id));
        }
        diagnostics.mahalanobis_distance = residual.dot(factorization.solve(residual));

        if (!(diagnostics.mahalanobis_distance < options.chi_square_threshold)) {
            diagnostics.outcome = ObservationOutcome::kGated;
            ++result.gated_count;
            result.diagnostics.push_back(diagnostics);
            continue;
        }

        diagnostics.outcome = ObservationOutcome::kApplied;
        result.diagnostics.push_back(diagnostics);
        prepared.push_back(PreparedObservation{
            .blocks = *blocks,
            .sparse = sparse,
            .residual = residual,
            .id = observation.id,
        });
    }

    if (prepared.empty()) {
        return result;
    }

    // The sweep is order-independent in exact arithmetic but not bitwise, so fix
    // the order to keep runs reproducible.
    std::sort(prepared.begin(), prepared.end(), [](const auto& left, const auto& right) {
        return left.blocks.landmark_offset < right.blocks.landmark_offset;
    });

    // Phase B: sequential updates against a nominal state that stays frozen, so
    // every Jacobian keeps the linearization point captured in phase A.
    auto covariance = state.active_covariance();
    Eigen::VectorXd error_state = Eigen::VectorXd::Zero(dimension);
    MeasurementRow row_block(kStereoMeasurementDim, dimension);
    GainColumn gain(dimension, kStereoMeasurementDim);
    GainColumn joseph_column(dimension, kStereoMeasurementDim);

    for (const auto& entry : prepared) {
        const int offset = entry.blocks.landmark_offset;

        // The innovation subtracts the error already absorbed by earlier
        // landmarks. Without this term the sweep double-counts those
        // corrections and stops matching a batch update.
        Eigen::Matrix<double, kMeasurementSparseColumns, 1> compact_error;
        compact_error.head<kLandmarkDim>() = error_state.segment<kLandmarkDim>(kPositionIndex);
        compact_error.segment<kLandmarkDim>(kLandmarkDim) =
            error_state.segment<kLandmarkDim>(kOrientationIndex);
        compact_error.tail<kLandmarkDim>() = error_state.segment<kLandmarkDim>(offset);
        const Eigen::Vector4d innovation = entry.residual - entry.sparse * compact_error;

        sparse_h_times_p(entry.blocks, covariance, row_block);
        const Eigen::Matrix4d innovation_covariance =
            gather_columns(row_block, offset) * entry.sparse.transpose() + pixel_covariance;
        const Eigen::LLT<Eigen::Matrix4d> factorization(innovation_covariance);
        if (factorization.info() != Eigen::Success) {
            return std::unexpected(std::format(
                "observation {}: innovation covariance is not positive definite", entry.id));
        }

        // K = P H^T S^-1 = (H P)^T S^-1, using the symmetry of P and S.
        gain = factorization.solve(row_block).transpose();
        error_state.noalias() += gain * innovation;

        covariance.noalias() -= gain * row_block;
        if (options.use_joseph_form) {
            // Factored Joseph, with M = P - K (H P) already written in place:
            // P <- M - (M H^T) K^T + K R K^T. Do not algebraically simplify this
            // back to P - K (H P); under the optimal gain the two are equal in
            // exact arithmetic, and computing it this way is the entire point.
            sparse_m_times_h_transpose(entry.blocks, entry.sparse, covariance, joseph_column);
            covariance.noalias() -= joseph_column * gain.transpose();
            covariance.noalias() += gain * pixel_covariance * gain.transpose();
        }
        symmetrize(covariance);
        ++result.applied_count;
    }

    // Phase C: one injection and one covariance reset for the whole frame.
    if (const auto injected = state.inject_error_state(error_state); !injected) {
        return std::unexpected(injected.error());
    }
    result.error_state = std::move(error_state);
    return result;
}
