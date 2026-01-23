#pragma once
#include "types.hpp"
#include <vector>

namespace sclerp::core {

struct PoseTrajectory {
  std::vector<SE3d> poses;
  std::vector<double> t;  // optional (can be empty)

  bool has_time() const { return !t.empty() && t.size() == poses.size(); }
};

}  // namespace sclerp::core
