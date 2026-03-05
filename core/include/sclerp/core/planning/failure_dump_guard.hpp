#pragma once

#include "sclerp/core/planning/failure_log.hpp"

#include <string>

namespace sclerp::core {

class KinematicsSolver;
struct MotionPlanRequest;
struct MotionPlanResult;

// RAII guard that dumps a debug artifact (best effort) when a motion planner returns failure.
//
// This is used to ensure the dump runs on all early-return paths without duplicating logic across
// planners.
class MotionPlanFailureDumpGuard {
public:
  MotionPlanFailureDumpGuard(std::string planner_name,
                             const MotionPlanRequest& req,
                             const KinematicsSolver& solver,
                             const FailureLogOptions& log_opt,
                             MotionPlanResult* out) noexcept;

  ~MotionPlanFailureDumpGuard() noexcept;

  MotionPlanFailureDumpGuard(const MotionPlanFailureDumpGuard&) = delete;
  MotionPlanFailureDumpGuard& operator=(const MotionPlanFailureDumpGuard&) = delete;

private:
  std::string planner_name_;
  const MotionPlanRequest* req_{nullptr};
  const KinematicsSolver* solver_{nullptr};
  const FailureLogOptions* log_opt_{nullptr};
  MotionPlanResult* out_{nullptr};
};

}  // namespace sclerp::core
