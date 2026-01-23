#include "sclerp/core/planner.hpp"
#include "sclerp/core/interpolate.hpp"
#include <algorithm>

namespace sclerp::core {

PlanningResult TaskSpacePlanner::plan(const PlanningRequest& req, const PlannerOptions& opt) const {
  PlanningResult out;

  const int N = std::max(2, opt.num_samples);
  out.traj.poses.reserve(static_cast<size_t>(N));

  for (int i = 0; i < N; ++i) {
    const double t = (N == 1) ? 0.0 : (static_cast<double>(i) / (N - 1));
    out.traj.poses.push_back(interpolate(req.start, req.goal, req.constraint, t));
  }

  out.success = true;
  return out;
}

}  // namespace sclerp::core
