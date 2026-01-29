#include "sclerp/core/planning/motion_plan.hpp"

#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/math/distance.hpp"
#include "sclerp/core/screw/screw.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace sclerp::core {

class Polynomial {
public:
  Polynomial(double tau_brake, double tau_max)
    : brake_start_(tau_brake), max_amp_(tau_max) {
    mat_(0,0) = std::pow(brake_start_, 3);
    mat_(0,1) = std::pow(brake_start_, 4);
    mat_(0,2) = std::pow(brake_start_, 5);

    mat_(1,0) = 3 * std::pow(brake_start_, 2);
    mat_(1,1) = 4 * std::pow(brake_start_, 3);
    mat_(1,2) = 5 * std::pow(brake_start_, 4);

    mat_(2,0) = 6 * brake_start_;
    mat_(2,1) = 12 * std::pow(brake_start_, 2);
    mat_(2,2) = 20 * std::pow(brake_start_, 3);

    r_val_(0) = max_amp_;
    r_val_(1) = 0;
    r_val_(2) = 0;

    coeff_ = mat_.inverse() * r_val_;
  }

  double getValue(double t) const {
    if (t >= brake_start_) {
      return max_amp_;
    }
    return ((coeff_(0) * std::pow(t, 3)) +
            (coeff_(1) * std::pow(t, 4)) +
            (coeff_(2) * std::pow(t, 5)));
  }

private:
  double brake_start_;
  double max_amp_;
  Eigen::Matrix3d mat_ = Eigen::Matrix3d::Zero();
  Eigen::Vector3d coeff_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_val_ = Eigen::Vector3d::Zero();
};


MotionPlanResult planMotionSclerp(const KinematicsSolver& solver,
                                  const MotionPlanRequest& req,
                                  const MotionPlanOptions& opt) {
  MotionPlanResult out;

  const int n = solver.model().dof();
  if (n <= 0) {
    log(LogLevel::Error, "planMotionSclerp: invalid dof");
    out.status = Status::InvalidParameter;
    return out;
  }
  if (req.q_init.size() != n) {
    log(LogLevel::Error, "planMotionSclerp: q_init size mismatch");
    out.status = Status::InvalidParameter;
    return out;
  }
  if (opt.max_iters <= 0) {
    log(LogLevel::Error, "planMotionSclerp: max_iters must be positive");
    out.status = Status::InvalidParameter;
    return out;
  }

  Transform g_fk = Transform::Identity();
  {
    const Status st = solver.forwardKinematics(req.q_init, &g_fk);
    if (!ok(st)) {
      log(LogLevel::Error, "planMotionSclerp: forwardKinematics failed");
      out.status = st;
      return out;
    }
  }
  const double dp_fk = positionDistance(g_fk, req.g_i);
  const double dr_fk = rotationDistance(g_fk, req.g_i);

  // Setup output trajectory
  out.trajectory.joint_names = solver.model().joint_names();
  out.trajectory.positions.clear();
  out.trajectory.time_from_start.clear();
  out.trajectory.positions.reserve(static_cast<std::size_t>(opt.max_iters) + 1);
  const bool fk_mismatch = (dp_fk > 1e-3 || dr_fk > 1e-2);

  if (!solver.model().within_limits(req.q_init, opt.q_init_tol)) {
    log(LogLevel::Error, "planMotionSclerp: q_init violates joint limits");
    out.status = Status::InvalidParameter;
    return out;
  }
  if (fk_mismatch) {
    if (shouldLog(LogLevel::Warn)) {
      std::ostringstream oss;
      oss << "g_i differs from FK(q_init) (dp=" << dp_fk << ", dr=" << dr_fk
          << "); using FK(q_init).";
      log(LogLevel::Warn, oss.str());
    }
  }

  Eigen::VectorXd q = req.q_init;
  out.trajectory.positions.push_back(q);

  // loop parameters
  double tau = opt.tau;
  const Polynomial tau_f(opt.tau_break, opt.tau_max);

  // Start state (safety-style: use FK(q_init))
  Transform g_current = g_fk;
  DualQuat dq_current(g_current);
  const DualQuat dq_f(req.g_f);

  double pos_dist = positionDistance(g_current, req.g_f);
  double rot_dist = rotationDistance(dq_current, dq_f);

  ScrewParameters base_screw;
  const Status st_screw = screwParameters(g_current, req.g_f, &base_screw, opt.thr);
  if (!ok(st_screw)) {
    log(LogLevel::Error, "planMotionSclerp: screwParameters failed");
    out.status = st_screw;
    return out;
  }

  int iters = 0;
  Eigen::VectorXd dq(n);
  Eigen::VectorXd q_next(n);
  Eigen::VectorXd joint_delta(n);
  while (!((pos_dist < opt.pos_tol) && (rot_dist < opt.rot_tol)) &&
         (iters < opt.max_iters)) {
    ++iters;

    double step_size = opt.beta;
    const DualQuat dq_next = DualQuat::interpolate(dq_current, dq_f, tau);

    if (tau < opt.tau_max) {
      const Transform g_next_temp = dq_next.toTransform();
      ScrewParameters curr_screw;
      if (ok(screwParameters(g_next_temp, req.g_f, &curr_screw, opt.thr)) &&
          base_screw.theta != 0.0) {
        const double motion_dist = std::abs(curr_screw.theta / base_screw.theta);
        const double tau_new = tau_f.getValue(motion_dist) + opt.tau_i;
        tau = std::min(opt.tau_max, tau_new);
      }
    }

    Status st = solver.rmrcIncrement(dq_current, dq_next, q, &dq);
    if (!ok(st)) {
      log(LogLevel::Error, "planMotionSclerp: rmrcIncrement failed");
      out.status = st;
      out.iters = iters;
      return out;
    }

    if (!dq.allFinite()) {
      log(LogLevel::Error, "planMotionSclerp: RMRC produced NaN/Inf");
      out.status = Status::Failure;
      out.iters = iters;
      return out;
    }

    while (true) {
      joint_delta = dq * step_size;
      q_next = q + joint_delta;

      if (solver.model().within_limits(q_next, /*tol=*/0.0)) {
        break;
      }

      const double max_val = joint_delta.cwiseAbs().maxCoeff();
      if (max_val <= opt.joint_delta_min) {
        log(LogLevel::Warn, "planMotionSclerp: joint limits reached");
        out.status = Status::JointLimit;
        out.iters = iters;
        return out;
      }

      step_size /= 10.0;
    }

    q = q_next;
    out.trajectory.positions.push_back(q);

    st = solver.forwardKinematics(q, &g_current);
    if (!ok(st)) {
      log(LogLevel::Error, "planMotionSclerp: forwardKinematics failed in loop");
      out.status = st;
      out.iters = iters;
      return out;
    }

    dq_current = DualQuat(g_current);
    pos_dist = positionDistance(g_current, req.g_f);
    rot_dist = rotationDistance(dq_current, dq_f);
  }

  out.iters = iters;
  if ((pos_dist < opt.pos_tol) && (rot_dist < opt.rot_tol)) {
    out.status = Status::Success;
  } else {
    if (shouldLog(LogLevel::Warn)) {
      std::ostringstream oss;
      oss << "planMotionSclerp reached max iters or stalled. "
          << "pos_dist=" << pos_dist << " (tol=" << opt.pos_tol << "), "
          << "rot_dist=" << rot_dist << " (tol=" << opt.rot_tol << "), "
          << "tau=" << tau << ", iters=" << iters;
      log(LogLevel::Warn, oss.str());
    }
    out.status = Status::Failure;
  }
  return out;
}

}  // namespace sclerp::core
