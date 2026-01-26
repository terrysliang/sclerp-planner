#pragma once
#include <Eigen/Core>
#include <string>
#include <vector>

namespace sclerp::core {

// ROS-free joint trajectory container
struct JointTrajectory {
  std::vector<std::string> joint_names;      // optional, but useful
  std::vector<Eigen::VectorXd> positions;    // size N, each is dof()
  std::vector<double> time_from_start;       // optional, size N

  bool has_time() const {
    return !time_from_start.empty() && time_from_start.size() == positions.size();
  }
  std::size_t size() const { return positions.size(); }
};

}  // namespace sclerp::core
