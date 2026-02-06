#include "sclerp/collision/motion_plan_collision.hpp"
#include "sclerp/collision/avoidance.hpp"

#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/math/distance.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/screw/screw.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace sclerp::collision {

using sclerp::core::KinematicsSolver;
using sclerp::core::LogLevel;
using sclerp::core::MotionPlanRequest;
using sclerp::core::MotionPlanResult;
using sclerp::core::Status;
using sclerp::core::Transform;
using sclerp::core::DualQuat;
using sclerp::core::positionDistance;
using sclerp::core::rotationDistance;
using sclerp::core::ok;
using sclerp::core::log;
using sclerp::core::shouldLog;
using sclerp::core::matrix4FromTransform;
using sclerp::core::Mat4;
using sclerp::core::ScrewParameters;
using sclerp::core::screwParameters;

namespace {

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

}  // namespace

MotionPlanResult planMotionSclerpWithCollision(
    const KinematicsSolver& solver,
    const MotionPlanRequest& req,
    CollisionScene& scene,
    const CollisionMotionPlanOptions& opt) {
  MotionPlanResult out;
  auto& link_meshes = scene.link_meshes;
  const auto& mesh_offset_transforms = scene.mesh_offset_transforms;
  const auto& obstacles = scene.obstacles;
  const auto& grasped_object = scene.grasped_object;

  const int n = solver.model().dof();
  if (n <= 0) {
    log(LogLevel::Error, "planMotionSclerpWithCollision: invalid dof");
    out.status = Status::InvalidParameter;
    return out;
  }
  if (req.q_init.size() != n) {
    log(LogLevel::Error, "planMotionSclerpWithCollision: q_init size mismatch");
    out.status = Status::InvalidParameter;
    return out;
  }
  if (opt.motion.max_iters <= 0) {
    log(LogLevel::Error, "planMotionSclerpWithCollision: max_iters must be positive");
    out.status = Status::InvalidParameter;
    return out;
  }
  if (opt.avoidance.safe_dist <= 0.0 || opt.avoidance.dt <= 0.0) {
    log(LogLevel::Error, "planMotionSclerpWithCollision: invalid collision params");
    out.status = Status::InvalidParameter;
    return out;
  }
  if (opt.query.num_links_ignore < 0 || opt.query.num_links_ignore >= n) {
    log(LogLevel::Error, "planMotionSclerpWithCollision: invalid num_links_ignore");
    out.status = Status::InvalidParameter;
    return out;
  }
  if (link_meshes.empty()) {
    log(LogLevel::Error, "planMotionSclerpWithCollision: link meshes are empty");
    out.status = Status::InvalidParameter;
    return out;
  }

  Transform g_fk = Transform::Identity();
  {
    const Status st = solver.forwardKinematics(req.q_init, &g_fk);
    if (!ok(st)) {
      log(LogLevel::Error, "planMotionSclerpWithCollision: forwardKinematics failed");
      out.status = st;
      return out;
    }
  }
  const double dp_fk = positionDistance(g_fk, req.g_i);
  const double dr_fk = rotationDistance(g_fk, req.g_i);

  out.path.joint_names = solver.model().joint_names();
  out.path.positions.clear();
  out.path.positions.reserve(static_cast<std::size_t>(opt.motion.max_iters) + 1);
  const bool fk_mismatch = (dp_fk > 1e-3 || dr_fk > 1e-2);

  if (!solver.model().within_limits(req.q_init, opt.motion.q_init_tol)) {
    log(LogLevel::Error, "planMotionSclerpWithCollision: q_init violates joint limits");
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
  out.path.positions.push_back(q);

  double tau = opt.motion.tau;
  const Polynomial tau_f(opt.motion.tau_break, opt.motion.tau_max);

  Transform g_current = g_fk;
  DualQuat dq_current(g_current);
  const DualQuat dq_f(req.g_f);

  double pos_dist = positionDistance(g_current, req.g_f);
  double rot_dist = rotationDistance(dq_current, dq_f);

  ScrewParameters base_screw;
  const Status st_screw = screwParameters(g_current, req.g_f, &base_screw, opt.motion.thr);
  if (!ok(st_screw)) {
    log(LogLevel::Error, "planMotionSclerpWithCollision: screwParameters failed");
    out.status = st_screw;
    return out;
  }

  int iters = 0;
  KinematicsSolver::RmrcWorkspace rmrc_ws;
  Eigen::VectorXd dq(n);
  Eigen::VectorXd q_next(n);
  Eigen::VectorXd joint_delta(n);
  Eigen::MatrixXd spatial_jacobian;
  std::vector<Transform> fk_intermediate;
  std::vector<Mat4> g_intermediate;

  while (!((pos_dist < opt.motion.pos_tol) && (rot_dist < opt.motion.rot_tol)) &&
         (iters < opt.motion.max_iters)) {
    ++iters;

    double step_size = opt.motion.beta;
    const DualQuat dq_next = DualQuat::interpolate(dq_current, dq_f, tau);

    if (tau < opt.motion.tau_max) {
      const Transform g_next_temp = dq_next.toTransform();
      ScrewParameters curr_screw;
      if (ok(screwParameters(g_next_temp, req.g_f, &curr_screw, opt.motion.thr)) &&
          base_screw.theta != 0.0) {
        const double motion_dist = std::abs(curr_screw.theta / base_screw.theta);
        const double tau_new = tau_f.getValue(motion_dist) + opt.motion.tau_i;
        tau = std::min(opt.motion.tau_max, tau_new);
      }
    }

    Status st = solver.rmrcIncrement(dq_current, dq_next, q, &dq, opt.motion.rmrc, &rmrc_ws);
    if (!ok(st)) {
      log(LogLevel::Error, "planMotionSclerpWithCollision: rmrcIncrement failed");
      out.status = st;
      out.iters = iters;
      return out;
    }

    if (!dq.allFinite()) {
      log(LogLevel::Error, "planMotionSclerpWithCollision: RMRC produced NaN/Inf");
      out.status = Status::Failure;
      out.iters = iters;
      return out;
    }

    while (true) {
      joint_delta = dq * step_size;
      q_next = q + joint_delta;

      // Collision avoidance adjustment
      fk_intermediate.clear();
      const Status st_fk_all = solver.forwardKinematicsAll(q_next, &fk_intermediate);
      if (!ok(st_fk_all)) {
        log(LogLevel::Error, "planMotionSclerpWithCollision: forwardKinematicsAll failed");
        out.status = st_fk_all;
        out.iters = iters;
        return out;
      }

      const std::size_t expected_transforms = fk_intermediate.size() + 1;
      if (expected_transforms != link_meshes.size()) {
        log(LogLevel::Error, "planMotionSclerpWithCollision: link mesh count mismatch");
        out.status = Status::InvalidParameter;
        out.iters = iters;
        return out;
      }
      if (mesh_offset_transforms.size() + 1 != expected_transforms) {
        log(LogLevel::Error, "planMotionSclerpWithCollision: mesh offset size mismatch");
        out.status = Status::InvalidParameter;
        out.iters = iters;
        return out;
      }

      g_intermediate.clear();
      g_intermediate.reserve(fk_intermediate.size() + 1);
      g_intermediate.push_back(Mat4::Identity());
      for (const auto& T : fk_intermediate) {
        g_intermediate.push_back(matrix4FromTransform(T));
      }

      st = updateLinkMeshTransforms(link_meshes,
                                    g_intermediate,
                                    mesh_offset_transforms);
      if (!ok(st)) {
        log(LogLevel::Error, "planMotionSclerpWithCollision: updateLinkMeshTransforms failed");
        out.status = st;
        out.iters = iters;
        return out;
      }

      if (grasped_object) {
        const Mat4& g_tool = g_intermediate.back();
        grasped_object->setTransform(g_tool.block<3,1>(0, 3), g_tool.block<3,3>(0, 0));
      }

      spatial_jacobian.resize(6, n);
      st = solver.spatialJacobian(q_next, spatial_jacobian);
      if (!ok(st)) {
        log(LogLevel::Error, "planMotionSclerpWithCollision: spatialJacobian failed");
        out.status = st;
        out.iters = iters;
        return out;
      }

      CollisionQueryOptions copt = opt.query;
      const CollisionContext cctx{
          link_meshes,
          obstacles,
          grasped_object,
          spatial_jacobian};
      ContactSet contacts;
      st = computeContacts(cctx, copt, &contacts);
      if (!ok(st)) {
        log(LogLevel::Error, "planMotionSclerpWithCollision: computeContacts failed");
        out.status = st;
        out.iters = iters;
        return out;
      }

      bool penetration = false;
      for (const auto& c : contacts.contacts) {
        if (c.distance < 0.0) {
          penetration = true;
          break;
        }
      }
      if (penetration) {
        if (shouldLog(LogLevel::Warn)) {
          log(LogLevel::Warn, "planMotionSclerpWithCollision: penetration detected, reducing step size");
        }
        const double max_val = joint_delta.cwiseAbs().maxCoeff();
        if (max_val <= opt.motion.joint_delta_min) {
          log(LogLevel::Warn, "planMotionSclerpWithCollision: penetration recovery failed");
          out.status = Status::Failure;
          out.iters = iters;
          return out;
        }
        step_size /= 10.0;
        continue;
      }

      Eigen::VectorXd adjusted;
      st = adjustJoints(opt.avoidance, contacts, q, q_next, &adjusted);
      if (!ok(st)) {
        log(LogLevel::Error, "planMotionSclerpWithCollision: adjustJoints failed");
        out.status = st;
        out.iters = iters;
        return out;
      }
      q_next = adjusted;

      if (solver.model().within_limits(q_next, /*tol=*/0.0)) {
        break;
      }

      const double max_val = joint_delta.cwiseAbs().maxCoeff();
      if (max_val <= opt.motion.joint_delta_min) {
        log(LogLevel::Warn, "planMotionSclerpWithCollision: joint limits reached");
        out.status = Status::JointLimit;
        out.iters = iters;
        return out;
      }

      step_size /= 10.0;
    }

    q = q_next;
    out.path.positions.push_back(q);

    st = solver.forwardKinematics(q, &g_current);
    if (!ok(st)) {
      log(LogLevel::Error, "planMotionSclerpWithCollision: forwardKinematics failed in loop");
      out.status = st;
      out.iters = iters;
      return out;
    }

    dq_current = DualQuat(g_current);
    pos_dist = positionDistance(g_current, req.g_f);
    rot_dist = rotationDistance(dq_current, dq_f);
  }

  out.iters = iters;
  if ((pos_dist < opt.motion.pos_tol) && (rot_dist < opt.motion.rot_tol)) {
    out.status = Status::Success;
  } else {
    if (shouldLog(LogLevel::Warn)) {
      std::ostringstream oss;
      oss << "planMotionSclerpWithCollision reached max iters or stalled. "
          << "pos_dist=" << pos_dist << " (tol=" << opt.motion.pos_tol << "), "
          << "rot_dist=" << rot_dist << " (tol=" << opt.motion.rot_tol << "), "
          << "tau=" << tau << ", iters=" << iters;
      log(LogLevel::Warn, oss.str());
    }
    out.status = Status::Failure;
  }
  return out;
}

}  // namespace sclerp::collision
