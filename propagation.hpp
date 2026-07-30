#pragma once

#include "state.hpp"
#include "types.hpp"

struct PropagationResult {
    NominalState nominal_state;
    StateCovariance covariance;
};

PropagationResult propagate(
    const NominalState& nominal_state,
    const ImuMeasurement& measurement,
    double timestep_seconds,
    const StateCovariance& covariance);
