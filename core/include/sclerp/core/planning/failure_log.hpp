#pragma once

#include "sclerp/core/common/status.hpp"
#include "sclerp/core/path/joint_path.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace sclerp::core {

struct FailureLogOptions {
  // When true, enables writing a timestamped joint-path dump (best effort).
  bool enabled{false};

  // Output directory for dumps. If empty, defaults to `./sclerp_failure_logs`.
  std::string dir{};
};

// Best-effort debug artifact dump for motion planner failures.
//
// When enabled, writes a timestamped CSV containing the partial `JointPath` (or at least `q_init`)
// under `./sclerp_failure_logs` (or `FailureLogOptions::dir`), and emits a summary message via the
// internal logger.
//
// This function never throws; errors are reported via the logger.
void dumpMotionPlanFailure(const std::string& planner_name,
                           Status status,
                           int iters,
                           const JointPath& path,
                           const FailureLogOptions& opt,
                           const Eigen::VectorXd* q_init = nullptr,
                           const std::vector<std::string>* joint_names = nullptr) noexcept;

}  // namespace sclerp::core
