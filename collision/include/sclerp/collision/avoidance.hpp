#pragma once

#include "sclerp/core/common/status.hpp"
#include "sclerp/collision/types.hpp"

#include <Eigen/Dense>

namespace sclerp::collision {

struct CollisionAvoidanceOptions {
  double safe_dist = 0.01;
  double dt = 0.001;
};

sclerp::core::Status adjustJoints(
    const CollisionAvoidanceOptions& opt,
    const ContactSet& contacts,
    const Eigen::VectorXd& current_joint_values,
    const Eigen::VectorXd& next_joint_values,
    Eigen::VectorXd* adjusted_joint_values);

}  // namespace sclerp::collision
