#pragma once
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/common/constants.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/trajectory/joint_trajectory.hpp"

namespace sclerp::core {

// Refactored version of kinlib::getMotionPlan (no collision, no ROS msgs)
struct MotionPlanOptions {
  int max_iters = 2000;
  int num_waypoints = 100;

  double pos_tol = 1e-4;
  double rot_tol = 1e-3;

  // kinlib-like: if violating limits, shrink and retry
  int max_retries = 10;
  double shrink = 0.5;

  Thresholds thr = kDefaultThresholds;
  RmrcOptions rmrc{};
};

struct MotionPlanRequest {
  Eigen::VectorXd q_init;
  Transform g_i;
  Transform g_f;
};

struct MotionPlanResult {
  Status status{Status::Failure};
  JointTrajectory trajectory;
  int iters{0};
};

MotionPlanResult planMotionSclerp(const KinematicsSolver& solver,
                                  const MotionPlanRequest& req,
                                  const MotionPlanOptions& opt = {});

}  // namespace sclerp::core
