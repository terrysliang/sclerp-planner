#pragma once
#include "export.hpp"
#include "trajectory.hpp"
#include "constraints.hpp"

namespace sclerp::core {

struct PlannerOptions {
  int num_samples{50};          // samples including endpoints
  bool include_endpoints{true};
};

struct PlanningRequest {
  SE3d start;
  SE3d goal;
  Constraint constraint;
};

struct PlanningResult {
  bool success{false};
  PoseTrajectory traj;
};

class SCLERP_CORE_API TaskSpacePlanner {
public:
  PlanningResult plan(const PlanningRequest& req, const PlannerOptions& opt) const;
};

}  // namespace sclerp::core
