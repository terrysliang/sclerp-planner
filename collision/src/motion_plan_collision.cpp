#include "sclerp/collision/motion_plan_collision.hpp"
#include "sclerp/collision/avoidance.hpp"

#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/math/distance.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/screw/screw.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
  ContactSet contacts;

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

    // Collision avoidance linearization at the current configuration.
    fk_intermediate.clear();
    const Status st_fk_all = solver.forwardKinematicsAll(q, &fk_intermediate);
    if (!ok(st_fk_all)) {
      log(LogLevel::Error, "planMotionSclerpWithCollision: forwardKinematicsAll failed");
      out.status = st_fk_all;
      out.iters = iters;
      return out;
    }

    const bool fk_has_tool = (fk_intermediate.size() == static_cast<std::size_t>(n + 2));
    const std::size_t expected_transforms = fk_intermediate.size();
    if (expected_transforms != link_meshes.size()) {
      if (fk_has_tool && link_meshes.size() == static_cast<std::size_t>(n + 1)) {
        log(LogLevel::Warn,
            "planMotionSclerpWithCollision: tool frame detected but no tool mesh provided; "
            "add a tool mesh to link_meshes or set tip_link to the flange.");
      }
      log(LogLevel::Error, "planMotionSclerpWithCollision: link mesh count mismatch");
      out.status = Status::InvalidParameter;
      out.iters = iters;
      return out;
    }
    if (mesh_offset_transforms.size() != expected_transforms) {
      log(LogLevel::Error, "planMotionSclerpWithCollision: mesh offset size mismatch");
      out.status = Status::InvalidParameter;
      out.iters = iters;
      return out;
    }

    g_intermediate.clear();
    g_intermediate.reserve(fk_intermediate.size());
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

    const CollisionContext cctx{ link_meshes, obstacles, grasped_object };
    st = computeContacts(solver, q, cctx, opt.query, &contacts);

    if (!ok(st)) {
      log(LogLevel::Error, "planMotionSclerpWithCollision: computeContacts failed");
      out.status = st;
      out.iters = iters;
      return out;
    }
    if (shouldLog(LogLevel::Debug)) {
      double min_contact_dist = std::numeric_limits<double>::infinity();
      int penetration_count = 0;
      for (const auto& contact : contacts.contacts) {
        min_contact_dist = std::min(min_contact_dist, contact.distance);
        if (contact.distance < 0.0) {
          ++penetration_count;
        }
      }
      std::ostringstream oss;
      oss << "planMotionSclerpWithCollision: iter=" << iters
          << ", contacts=" << contacts.contacts.size()
          << ", penetration_contacts=" << penetration_count
          << ", min_contact_dist=" << min_contact_dist
          << ", step_size_init=" << step_size;
      log(LogLevel::Debug, oss.str());
    }

    int retry_count = 0;
    while (true) {
      joint_delta = dq * step_size;
      q_next = q + joint_delta;

      Eigen::VectorXd adjusted;
      st = adjustJoints(opt.avoidance, contacts, q, q_next, &adjusted);
      if (!ok(st)) {
        log(LogLevel::Error, "planMotionSclerpWithCollision: adjustJoints failed");
        out.status = st;
        out.iters = iters;
        return out;
      }
      q_next = adjusted;
      const Eigen::VectorXd adjusted_delta = q_next - q;
      const double raw_delta_norm = joint_delta.norm();
      const double adjusted_delta_norm = adjusted_delta.norm();
      double direction_alignment = 1.0;
      if (raw_delta_norm > 0.0 && adjusted_delta_norm > 0.0) {
        direction_alignment = joint_delta.dot(adjusted_delta) / (raw_delta_norm * adjusted_delta_norm);
      }
      if (shouldLog(LogLevel::Debug)) {
        std::ostringstream oss;
        oss << "planMotionSclerpWithCollision: iter=" << iters
            << ", retry=" << retry_count
            << ", step_size=" << step_size
            << ", raw_step_norm=" << raw_delta_norm
            << ", adjusted_step_norm=" << adjusted_delta_norm
            << ", direction_alignment=" << direction_alignment;
        if (direction_alignment < 0.0) {
          oss << " (backward correction)";
        }
        log(LogLevel::Debug, oss.str());
      }

      if (solver.model().within_limits(q_next, /*tol=*/0.0)) {
        break;
      }

      const double max_adjusted = adjusted_delta.cwiseAbs().maxCoeff();
      const double max_raw = joint_delta.cwiseAbs().maxCoeff();
      if (max_adjusted <= opt.motion.joint_delta_min || max_raw <= opt.motion.joint_delta_min) {
        log(LogLevel::Warn, "planMotionSclerpWithCollision: joint limits reached");
        out.status = Status::JointLimit;
        out.iters = iters;
        return out;
      }

      if (shouldLog(LogLevel::Debug)) {
        std::ostringstream oss;
        oss << "planMotionSclerpWithCollision: iter=" << iters
            << ", retry=" << retry_count
            << ", q_next violates limits, shrinking step from " << step_size
            << " to " << (step_size / 10.0)
            << ", max_raw=" << max_raw
            << ", max_adjusted=" << max_adjusted;
        log(LogLevel::Debug, oss.str());
      }
      step_size /= 10.0;
      ++retry_count;
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

  // Keep scene transforms consistent with the final accepted q.
  fk_intermediate.clear();
  const Status st_fk_all_end = solver.forwardKinematicsAll(q, &fk_intermediate);
  if (!ok(st_fk_all_end)) {
    log(LogLevel::Error, "planMotionSclerpWithCollision: forwardKinematicsAll failed at final sync");
    out.status = st_fk_all_end;
    out.iters = iters;
    return out;
  }

  const bool fk_has_tool_end = (fk_intermediate.size() == static_cast<std::size_t>(n + 2));
  const std::size_t expected_transforms_end = fk_intermediate.size();
  if (expected_transforms_end != link_meshes.size()) {
    if (fk_has_tool_end && link_meshes.size() == static_cast<std::size_t>(n + 1)) {
      log(LogLevel::Warn,
          "planMotionSclerpWithCollision: tool frame detected but no tool mesh provided; "
          "add a tool mesh to link_meshes or set tip_link to the flange.");
    }
    log(LogLevel::Error, "planMotionSclerpWithCollision: link mesh count mismatch at final sync");
    out.status = Status::InvalidParameter;
    out.iters = iters;
    return out;
  }
  if (mesh_offset_transforms.size() != expected_transforms_end) {
    log(LogLevel::Error, "planMotionSclerpWithCollision: mesh offset size mismatch at final sync");
    out.status = Status::InvalidParameter;
    out.iters = iters;
    return out;
  }

  g_intermediate.clear();
  g_intermediate.reserve(fk_intermediate.size());
  for (const auto& T : fk_intermediate) {
    g_intermediate.push_back(matrix4FromTransform(T));
  }

  Status st_final = updateLinkMeshTransforms(link_meshes,
                                             g_intermediate,
                                             mesh_offset_transforms);
  if (!ok(st_final)) {
    log(LogLevel::Error, "planMotionSclerpWithCollision: updateLinkMeshTransforms failed at final sync");
    out.status = st_final;
    out.iters = iters;
    return out;
  }
  if (grasped_object) {
    const Mat4& g_tool = g_intermediate.back();
    grasped_object->setTransform(g_tool.block<3,1>(0, 3), g_tool.block<3,3>(0, 0));
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
