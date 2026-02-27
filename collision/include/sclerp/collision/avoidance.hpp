#pragma once

#include "sclerp/core/common/status.hpp"
#include "sclerp/collision/types.hpp"

#include <Eigen/Dense>

namespace sclerp::collision {

// Collision avoidance step used by `planMotionSclerpWithCollision`.
//
// This implements a local, linearized correction: given a proposed step q -> q_next, we compute
// closest contacts and solve a small LCP to find complementarity impulses that push the robot
// away from obstacles to satisfy `distance >= safe_dist` when possible.
struct CollisionAvoidanceOptions {
  // Hard boundary around obstacles (paper: col_tol / epsilon)
  double safe_dist = 0.005;

  // Efficiency-only: when to include constraint in the LCP (paper: comp_activate_tol)
  // Typical: comp_activate_tol >= safe_dist
  double comp_activate_tol = 0.01;

  // Discretization timestep h
  double dt = 0.001;

  // Optional damping for analytic pseudo-inverse:
  // J^T (J J^T + lambda I)^-1, still “closed form” and only a 3×3 solve.
  double pinv_lambda = 1e-9;
};

sclerp::core::Status adjustJoints(
    const CollisionAvoidanceOptions& opt,
    const ContactSet& contacts,
    const Eigen::VectorXd& current_joint_values,
    const Eigen::VectorXd& next_joint_values,
    Eigen::VectorXd* adjusted_joint_values);

}  // namespace sclerp::collision
