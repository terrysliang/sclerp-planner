#pragma once
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/common/constants.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/path/joint_path.hpp"

namespace sclerp::core {

// Local motion planner that tracks an SE(3) ScLERP / dual-quat interpolation using RMRC.
//
// Sketch:
// - Start from `q_init` (validated and FK'ed).
// - Generate a small pose target step via dual-quat interpolation toward `g_f`.
// - Use damped RMRC (`KinematicsSolver::rmrcIncrement`) to compute a joint increment.
// - Back off the step size to satisfy joint limits.
// This is a local method: it can fail/stall in difficult regions (limits/singularities/clutter).
struct MotionPlanOptions {
  int max_iters = 10000;
  double q_init_tol = 0.0;

  // stop thresholds
  double pos_tol = 5e-4;  // meters
  double rot_tol = 5e-3;  // quaternion chordal distance (not angle in radians)

  // motion schedule parameters
  double beta = 0.5;
  double tau = 0.001;
  double tau_i = 0.01;
  double tau_max = 0.1;
  double tau_break = 0.9;

  // If joint deltas shrink below this, treat as joint-limit failure
  double joint_delta_min = 1e-4;

  // RMRC damping configuration
  RmrcOptions rmrc{};

  Thresholds thr = kDefaultThresholds;
};

struct MotionPlanRequest {
  // Initial joint configuration (size must match `solver.model().dof()`).
  Eigen::VectorXd q_init;

  // Initial end-effector pose (base->tool). This is used as a sanity check:
  // if it differs from FK(q_init), the planner will warn and internally trust FK(q_init).
  Transform g_i;

  // Target end-effector pose (base->tool).
  Transform g_f;
};

struct MotionPlanResult {
  // Overall status for the run. On success, `path.positions.back()` reaches the goal within tolerances.
  Status status{Status::Failure};

  // Joint-space path (no timing). The first entry is always `q_init`.
  JointPath path;
  int iters{0};
};

MotionPlanResult planMotionSclerp(const KinematicsSolver& solver,
                                  const MotionPlanRequest& req,
                                  const MotionPlanOptions& opt = {});

}  // namespace sclerp::core
