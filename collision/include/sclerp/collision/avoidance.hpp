#pragma once

#include "sclerp/core/common/status.hpp"
#include "sclerp/collision/types.hpp"

#include <Eigen/Dense>
#include <vector>

namespace sclerp::collision {

sclerp::core::Status adjustJoints(
    double h,
    const std::vector<double>& dist_array,
    const Eigen::MatrixXd& contact_normal_array,
    double safe_dist,
    const Eigen::VectorXd& current_joint_values,
    const Eigen::VectorXd& next_joint_values,
    const std::vector<Eigen::MatrixXd>& j_contact_array,
    Eigen::VectorXd* adjusted_joint_values);

sclerp::core::Status adjustJoints(
    double h,
    double safe_dist,
    const ContactSet& contacts,
    const Eigen::VectorXd& current_joint_values,
    const Eigen::VectorXd& next_joint_values,
    Eigen::VectorXd* adjusted_joint_values);

}  // namespace sclerp::collision
