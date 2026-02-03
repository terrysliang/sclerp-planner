#pragma once

#include "sclerp/core/common/status.hpp"

#include <Eigen/Dense>

namespace sclerp::collision {

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
