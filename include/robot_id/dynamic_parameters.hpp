#pragma once

#include <Eigen/Core>

namespace robot_id {

// Build the frozen deterministic nominal model mismatch while preserving a
// physically meaningful (m, c, I_C) representation for every moving link.
Eigen::VectorXd makeNominalMismatchedParameters(const Eigen::VectorXd& theta_true);

}  // namespace robot_id
