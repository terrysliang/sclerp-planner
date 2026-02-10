#pragma once

#include "sclerp/core/common/status.hpp"

#include <Eigen/Dense>

namespace sclerp::collision {

// Lemke algorithm for Linear Complementarity Problems (LCP):
//   w = q + M z,  w >= 0,  z >= 0,  w^T z = 0
//
// Used by the avoidance layer to compute contact impulses. This is a local numeric method and can
// fail via ray termination or unstable pivots on ill-conditioned problems.
struct LemkeResult {
  Eigen::VectorXd w;
  Eigen::VectorXd z;
  int iterations = 0;
  bool ray_termination = false;
};

sclerp::core::Status lemkeSolve(const Eigen::VectorXd& q,
                               const Eigen::MatrixXd& M,
                               LemkeResult* out);

}  // namespace sclerp::collision
