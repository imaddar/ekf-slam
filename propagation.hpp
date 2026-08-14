#pragma once

#include "parser.hpp"
#include "state.hpp"
#include "types.hpp"

class SlamState;

struct PropagationResult {
    NominalState nominal_state;
    StateCovariance covariance;
};

// Rejects non-finite and negative timesteps. A negative dt runs the covariance
// update backwards, which removes information instead of adding it and drives P
// indefinite; a zero dt is allowed because it is an exact no-op.
ParseResult<PropagationResult> propagate(
    const NominalState& nominal_state,
    const ImuMeasurement& measurement,
    const ImuCalibration& imu_calibration,
    double timestep_seconds,
    const StateCovariance& covariance);

// Propagates the robot state and the active joint covariance in place. Landmark
// covariance is static during prediction; only P_rr and P_rl change.
ParseResult<void> propagate_slam(
    SlamState& slam_state,
    const ImuMeasurement& measurement,
    const ImuCalibration& imu_calibration,
    double timestep_seconds);
