#include "sclerp/core/planning/failure_dump_guard.hpp"

#include "sclerp/core/planning/motion_plan.hpp"

#include <utility>

namespace sclerp::core {

MotionPlanFailureDumpGuard::MotionPlanFailureDumpGuard(std::string planner_name,
                                                       const MotionPlanRequest& req,
                                                       const KinematicsSolver& solver,
                                                       const FailureLogOptions& log_opt,
                                                       MotionPlanResult* out) noexcept
    : planner_name_(std::move(planner_name)),
      req_(&req),
      solver_(&solver),
      log_opt_(&log_opt),
      out_(out) {}

MotionPlanFailureDumpGuard::~MotionPlanFailureDumpGuard() noexcept {
  if (!out_) return;
  if (ok(out_->status)) return;

  dumpMotionPlanFailure(
      planner_name_.empty() ? std::string("planMotion") : planner_name_,
      out_->status,
      out_->iters,
      out_->path,
      *log_opt_,
      req_ ? &req_->q_init : nullptr,
      solver_ ? &solver_->model().joint_names() : nullptr);
}

}  // namespace sclerp::core
