#pragma once

#include "parser.hpp"
#include "state.hpp"
#include "types.hpp"

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
