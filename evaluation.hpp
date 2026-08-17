#pragma once

#include "parser.hpp"
#include "slam_state.hpp"

#include <span>
#include <vector>

// One posterior robot estimate associated with a camera timestamp.  The
// covariance is the 15-state robot block after all measurements at that time.
struct TrajectoryEstimate {
    TimestampNs timestamp;
    NominalState state;
    ImuStateCovariance covariance;
};

struct TrajectoryMetrics {
    // Both trajectory errors are root-mean-square values.  ATE uses the raw
    // EuRoC world frame: no SE(3) alignment is applied, because this VIO run
    // is initialized in that frame and alignment would hide drift.
    double ate_position_rmse_m = 0.0;
    double rpe_translation_rmse_m_per_s = 0.0;
    double rpe_rotation_rmse_rad_per_s = 0.0;
    double mean_robot_nees = 0.0;
    std::size_t sample_count = 0;
    std::size_t rpe_pair_count = 0;
};

// Linearly interpolates EuRoC translation, velocity, and biases, and slerps
// orientation.  It rejects timestamps outside the ground-truth range.
ParseResult<GroundTruthState> interpolate_ground_truth(
    std::span<const GroundTruthState> ground_truth, TimestampNs timestamp);

// Computes raw-frame ATE, one-second RPE rates, and 15-dof robot NEES.  The
// error vector obeys CONVENTIONS.md: position/velocity/bias truth-minus-
// estimate in W, and orientation log(R_est^-1 R_truth) in the estimate body.
ParseResult<TrajectoryMetrics> evaluate_trajectory(
    std::span<const TrajectoryEstimate> estimates,
    std::span<const GroundTruthState> ground_truth,
    double rpe_interval_seconds = 1.0);
