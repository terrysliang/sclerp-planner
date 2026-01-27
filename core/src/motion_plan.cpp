#include "sclerp/core/planning/motion_plan.hpp"

#include "sclerp/core/math/distance.hpp"
#include "sclerp/core/screw/screw.hpp"

#include <algorithm>
#include <cmath>

namespace sclerp::core {

static inline double clamp01(double t) {
  return std::max(0.0, std::min(1.0, t));
}

static inline bool poseCloseEnough(const Transform& a,
                                   const Transform& b,
                                   double pos_tol,
                                   double rot_tol) {
  return (positionDistance(a, b) <= pos_tol) && (rotationDistance(a, b) <= rot_tol);
}

MotionPlanResult planMotionSclerp(const KinematicsSolver& solver,
                                  const MotionPlanRequest& req,
                                  const MotionPlanOptions& opt) {
  MotionPlanResult out;

  const int n = solver.model().dof();
  if (n <= 0) {
    out.status = Status::InvalidParameter;
    return out;
  }
  if (req.q_init.size() != n) {
    out.status = Status::InvalidParameter;
    return out;
  }
  if (opt.num_waypoints < 2 || opt.max_iters <= 0) {
    out.status = Status::InvalidParameter;
    return out;
  }

  // Initialize q (clamp to limits if needed)
  Eigen::VectorXd q = req.q_init;
  if (!solver.model().within_limits(q, /*tol=*/0.0)) {
    q = solver.model().clamp_to_limits(q);
  }

  // Compute current FK
  Transform g_cur = Transform::Identity();
  {
    const Status st = solver.forwardKinematics(q, &g_cur);
    if (!ok(st)) {
      out.status = st;
      return out;
    }
  }

  // If provided g_i differs a lot from FK(q_init), prefer FK as start to avoid a jump.
  Transform g_start = req.g_i;
  {
    const double dp = positionDistance(g_cur, req.g_i);
    const double dr = rotationDistance(g_cur, req.g_i);
    if (dp > 1e-3 || dr > 1e-2) {
      g_start = g_cur;
    }
  }

  // Setup output trajectory
  out.trajectory.joint_names = solver.model().joint_names();
  out.trajectory.positions.clear();
  out.trajectory.time_from_start.clear();
  out.trajectory.positions.reserve(static_cast<std::size_t>(opt.max_iters) + 1);
  out.trajectory.positions.push_back(q);

  int iters = 0;

  // Iterate waypoint by waypoint
  for (int wp = 1; wp < opt.num_waypoints; ++wp) {
    const double tau = static_cast<double>(wp) / static_cast<double>(opt.num_waypoints - 1);
    const Transform g_goal_wp = sclerp(g_start, req.g_f, tau);

    // Move towards this waypoint with RMRC until within tolerance or we run out of iterations.
    while (!poseCloseEnough(g_cur, g_goal_wp, opt.pos_tol, opt.rot_tol)) {
      if (iters >= opt.max_iters) {
        out.status = Status::Failure;
        out.iters = iters;
        return out;
      }

      Eigen::VectorXd dq(n);
      Status st = solver.rmrcIncrement(g_cur, g_goal_wp, q, &dq, opt.rmrc);
      if (!ok(st)) {
        out.status = st;
        out.iters = iters;
        return out;
      }

      // Joint-limit handling: shrink step if needed
      bool accepted = false;
      Eigen::VectorXd q_next(n);

      for (int retry = 0; retry <= opt.max_retries; ++retry) {
        const double scale = (retry == 0) ? 1.0 : std::pow(opt.shrink, retry);
        q_next = q + dq * scale;

        if (solver.model().within_limits(q_next, /*tol=*/0.0)) {
          accepted = true;
          break;
        }
      }

      if (!accepted) {
        out.status = Status::JointLimit;
        out.iters = iters;
        return out;
      }

      // Apply
      q = q_next;

      st = solver.forwardKinematics(q, &g_cur);
      if (!ok(st)) {
        out.status = st;
        out.iters = iters;
        return out;
      }

      out.trajectory.positions.push_back(q);
      ++iters;
    }
  }

  // Final check to goal
  if (poseCloseEnough(g_cur, req.g_f, opt.pos_tol, opt.rot_tol)) {
    out.status = Status::Success;
  } else {
    out.status = Status::Failure;
  }
  out.iters = iters;
  return out;
}

}  // namespace sclerp::core
