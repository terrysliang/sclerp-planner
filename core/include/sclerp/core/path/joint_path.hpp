#pragma once
#include <Eigen/Core>
#include <string>
#include <vector>

namespace sclerp::core {

// Kinematic joint path (no timing).
struct JointPath {
  std::vector<std::string> joint_names;      // optional, but useful
  std::vector<Eigen::VectorXd> positions;    // size N, each is dof()

  std::size_t size() const { return positions.size(); }
};

}  // namespace sclerp::core
