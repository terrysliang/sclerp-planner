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
  double pos_tol = 5e-4;
  double rot_tol = 5e-3;  // chordal distance (quat)

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
  Eigen::VectorXd q_init;
  Transform g_i;
  Transform g_f;
};

struct MotionPlanResult {
  Status status{Status::Failure};
  JointPath path;
  int iters{0};
};

MotionPlanResult planMotionSclerp(const KinematicsSolver& solver,
                                  const MotionPlanRequest& req,
                                  const MotionPlanOptions& opt = {});

}  // namespace sclerp::core
