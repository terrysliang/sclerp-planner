// Kinematics and differential kinematics (Jacobian) for `ManipulatorModel`.
//
// This file implements POE-style FK/Jacobians using the model's space screw axes, then
// optionally applies a fixed `base_offset` to express results in the world frame.
#include "sclerp/core/kinematics/kinematics_solver.hpp"

#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/math/so3.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>

namespace sclerp::core {

KinematicsSolver::KinematicsSolver(ManipulatorModel model)
  : model_(std::move(model)) {}

static Transform jointExp(const JointSpec& j, double q) {
  Transform T = Transform::Identity();

  if (j.type == JointType::Revolute) {
    const Vec3 w = j.axis;
    const Mat3 R = Eigen::AngleAxisd(q, w).toRotationMatrix();
    const Vec3 p = (Mat3::Identity() - R) * j.point;
    T.linear() = R;
    T.translation() = p;
  } else if (j.type == JointType::Prismatic) {
    T.translation() = j.axis * q;
  } else {
    // Fixed: identity
  }

  return T;
}

static AdjointMatrix adjoint(const Transform& T) {
  const Mat3 R = T.rotation();
  const Vec3 p = T.translation();

  AdjointMatrix Ad = AdjointMatrix::Zero();
  Ad.block<3,3>(0,0) = R;
  Ad.block<3,3>(3,3) = R;
  Ad.block<3,3>(0,3) = hat3(p) * R;  // ordering: [v; w]
  return Ad;
}

Status KinematicsSolver::forwardKinematics(const Eigen::VectorXd& q,
                                           Transform* g_base_tool) const {
  if (!g_base_tool) {
    log(LogLevel::Error, "forwardKinematics: null output pointer");
    return Status::InvalidParameter;
  }
  if (q.size() != model_.dof()) {
    log(LogLevel::Error, "forwardKinematics: q size mismatch");
    return Status::InvalidParameter;
  }

  const Transform& base = model_.base_offset();
  const int n = model_.dof();
  Transform prod = Transform::Identity();
  for (int i = 0; i < n; ++i) {
    prod = prod * jointExp(model_.joint(i), q(i));
  }

  *g_base_tool = base * prod * model_.ee_home();
  return Status::Success;
}

Status KinematicsSolver::forwardKinematicsAll(const Eigen::VectorXd& q,
                                              std::vector<Transform>* intermediate_transforms) const {
  if (!intermediate_transforms) {
    log(LogLevel::Error, "forwardKinematicsAll: null output pointer");
    return Status::InvalidParameter;
  }
  if (q.size() != model_.dof()) {
    log(LogLevel::Error, "forwardKinematicsAll: q size mismatch");
    return Status::InvalidParameter;
  }

  intermediate_transforms->clear();
  const std::size_t reserve_count =
      static_cast<std::size_t>(model_.dof()) + 1 + (model_.has_tool_frame() ? 1 : 0);
  intermediate_transforms->reserve(reserve_count);

  const Transform& base = model_.base_offset();
  const int n = model_.dof();
  Transform prod = Transform::Identity();

  // Base link transform (may be identity).
  intermediate_transforms->push_back(base);

  for (int i = 0; i < n; ++i) {
    prod = prod * jointExp(model_.joint(i), q(i));

    // each JointSpec has joint_tip_home (fixed home transform for that joint's tip frame)
    // so g_i(q) = (Π exp(Sk qk)) * M_i
    const Transform g_tip = base * prod * model_.joint(i).joint_tip_home;
    intermediate_transforms->push_back(g_tip);
  }

  if (model_.has_tool_frame()) {
    // Append EE as last element when a tool frame exists.
    intermediate_transforms->push_back(base * prod * model_.ee_home());
  }
  return Status::Success;
}

Status KinematicsSolver::spatialJacobian(const Eigen::VectorXd& q,
                                         Eigen::MatrixXd* J_space) const {
  if (!J_space) {
    log(LogLevel::Error, "spatialJacobian: null output pointer");
    return Status::InvalidParameter;
  }
  if (q.size() != model_.dof()) {
    log(LogLevel::Error, "spatialJacobian: q size mismatch");
    return Status::InvalidParameter;
  }

  const int n = model_.dof();
  J_space->resize(6, n);
  return spatialJacobian(q, Eigen::Ref<Eigen::MatrixXd>(*J_space));
}

Status KinematicsSolver::spatialJacobian(const Eigen::VectorXd& q,
                                         Eigen::Ref<Eigen::MatrixXd> J_space) const {
  if (q.size() != model_.dof()) {
    log(LogLevel::Error, "spatialJacobian: q size mismatch");
    return Status::InvalidParameter;
  }

  const int n = model_.dof();
  if (J_space.rows() != 6 || J_space.cols() != n) {
    log(LogLevel::Error, "spatialJacobian: output size mismatch");
    return Status::InvalidParameter;
  }

  const ScrewMatrix& S_space = model_.S_space(); // [v; w] columns

  if (n > 0) {
    J_space.col(0) = S_space.col(0);
  }

  Transform prod = Transform::Identity();  // Π_{k<i} exp(joint_k)
  for (int i = 1; i < n; ++i) {
    prod = prod * jointExp(model_.joint(i - 1), q(i - 1));
    const AdjointMatrix Ad = adjoint(prod);
    J_space.col(i) = Ad * S_space.col(i);
  }

  if (model_.has_base_offset()) {
    const AdjointMatrix Ad_base = adjoint(model_.base_offset());
    J_space = Ad_base * J_space;
  }
  return Status::Success;
}

// 1) Space Jacobian prefix (VW ordering) for joints [0..link_index].
//    Output is 6×(link_index+1). Columns are [v; w] in the space frame (after base_offset if any).
Status KinematicsSolver::spatialJacobianUpToLink(
    const Eigen::VectorXd& q,
    int link_index,
    Eigen::Ref<Eigen::MatrixXd> J_space_prefix) const {

  const int n = model_.dof();
  if (q.size() != n) {
    log(LogLevel::Error, "spatialJacobianUpToLink: q size mismatch");
    return Status::InvalidParameter;
  }
  if (link_index < 0 || link_index >= n) {
    log(LogLevel::Error, "spatialJacobianUpToLink: link_index out of range");
    return Status::InvalidParameter;
  }

  const int k = link_index + 1;
  if (J_space_prefix.rows() != 6 || J_space_prefix.cols() != k) {
    log(LogLevel::Error, "spatialJacobianUpToLink: output size mismatch (need 6×(link_index+1))");
    return Status::InvalidParameter;
  }

  const ScrewMatrix& S_space = model_.S_space();  // columns are [v; w]

  // Fill columns 0..link_index using the same recursion as spatialJacobian(), but stop at k.
  J_space_prefix.setZero();
  J_space_prefix.col(0) = S_space.col(0);

  Transform prod = Transform::Identity();  // Π_{m<i} exp(joint_m)
  for (int i = 1; i < k; ++i) {
    prod = prod * jointExp(model_.joint(i - 1), q(i - 1));
    const AdjointMatrix Ad = adjoint(prod);     // [v; w] convention
    J_space_prefix.col(i) = Ad * S_space.col(i);
  }

  if (model_.has_base_offset()) {
    const AdjointMatrix Ad_base = adjoint(model_.base_offset());
    J_space_prefix = Ad_base * J_space_prefix;
  }

  return Status::Success;
}

// 2) Contact-point linear Jacobian (prefix) for a point expressed in the same space/world frame.
//    Output is 3×(link_index+1): p_dot = J_point_prefix * q_dot
//    Using [v; w] convention: p_dot = v + w×p = v - hat(p)*w.
Status KinematicsSolver::pointJacobianUpToLink(
    const Eigen::VectorXd& q,
    int link_index,
    const Vec3& point_space,  // point position in the same "space" frame used by the Jacobian
    Eigen::Ref<Eigen::MatrixXd> J_point_prefix) const {

  const int n = model_.dof();
  if (q.size() != n) {
    log(LogLevel::Error, "pointJacobianUpToLink: q size mismatch");
    return Status::InvalidParameter;
  }
  if (link_index < 0 || link_index >= n) {
    log(LogLevel::Error, "pointJacobianUpToLink: link_index out of range");
    return Status::InvalidParameter;
  }

  const int k = link_index + 1;
  if (J_point_prefix.rows() != 3 || J_point_prefix.cols() != k) {
    log(LogLevel::Error, "pointJacobianUpToLink: output size mismatch (need 3×(link_index+1))");
    return Status::InvalidParameter;
  }

  // Compute prefix space Jacobian (6×k) then convert to point linear Jacobian (3×k).
  Eigen::MatrixXd J6(6, k);
  const Status st = spatialJacobianUpToLink(q, link_index, J6);
  if (!ok(st)) return st;

  const Eigen::MatrixXd Jv = J6.topRows(3);
  const Eigen::MatrixXd Jw = J6.bottomRows(3);

  const Mat3 P_hat = hat3(point_space);
  J_point_prefix = Jv - P_hat * Jw;

  return Status::Success;
}

Status KinematicsSolver::rmrcIncrement(const DualQuat& dq_i,
                                       const DualQuat& dq_f,
                                       const Eigen::VectorXd& q_current,
                                       Eigen::VectorXd* dq,
                                       const RmrcOptions& opt,
                                       RmrcWorkspace* ws) const {
  if (!dq) {
    log(LogLevel::Error, "rmrcIncrement: null output pointer");
    return Status::InvalidParameter;
  }
  if (q_current.size() != model_.dof()) {
    log(LogLevel::Error, "rmrcIncrement: q size mismatch");
    return Status::InvalidParameter;
  }

  // RMRC in dual-quat form. Normalize inputs to ensure valid quaternion algebra.
  const DualQuat dq_i_norm = dq_i.normalized();
  const DualQuat dq_f_norm = dq_f.normalized();
  const Transform g_i = dq_i_norm.toTransform();
  const Transform g_f = dq_f_norm.toTransform();

  Eigen::Matrix<double, 7, 1> gamma_i;
  Eigen::Matrix<double, 7, 1> gamma_f;

  const Vec3 p_i = g_i.translation();
  gamma_i.head<3>() = p_i;
  const Quat q_i = dq_i_norm.real();
  gamma_i(3) = q_i.w();
  gamma_i(4) = q_i.x();
  gamma_i(5) = q_i.y();
  gamma_i(6) = q_i.z();

  const Vec3 p_f = g_f.translation();
  gamma_f.head<3>() = p_f;
  const Quat q_f = dq_f_norm.real();
  gamma_f(3) = q_f.w();
  gamma_f(4) = q_f.x();
  gamma_f(5) = q_f.y();
  gamma_f(6) = q_f.z();

  Eigen::MatrixXd local_s_jac;
  Eigen::MatrixXd& s_jac = ws ? ws->s_jac : local_s_jac;
  const int n = model_.dof();
  if (s_jac.rows() != 6 || s_jac.cols() != n) {
    s_jac.resize(6, n);
  }
  const Status st = spatialJacobian(q_current, Eigen::Ref<Eigen::MatrixXd>(s_jac));
  if (!ok(st)) {
    log(LogLevel::Error, "rmrcIncrement: spatialJacobian failed");
    return st;
  }

  Eigen::Vector4d qv;
  qv << q_i.w(), q_i.x(), q_i.y(), q_i.z();

  Eigen::Matrix<double, 3, 4> J1 = Eigen::Matrix<double, 3, 4>::Zero();
  J1.block<3,1>(0,0) = -qv.tail<3>();
  J1(0,1) =  qv(0);
  J1(0,2) = -qv(3);
  J1(0,3) =  qv(2);
  J1(1,1) =  qv(3);
  J1(1,2) =  qv(0);
  J1(1,3) = -qv(1);
  J1(2,1) = -qv(2);
  J1(2,2) =  qv(1);
  J1(2,3) =  qv(0);

  Eigen::Matrix<double, 6, 7> J2 = Eigen::Matrix<double, 6, 7>::Zero();
  J2.block<3,3>(0,0) = Mat3::Identity();
  J2.block<3,4>(0,3) = 2.0 * hat3(p_i) * J1;
  J2.block<3,4>(3,3) = 2.0 * J1;

  Eigen::Matrix<double, 6, 6> temp = s_jac * s_jac.transpose();
  double lambda_sq = 0.0;
  if (opt.damping == RmrcDamping::Constant) {
    if (opt.lambda > 0.0) {
      lambda_sq = opt.lambda * opt.lambda;
    }
  } else if (opt.damping == RmrcDamping::Adaptive) {
    if (opt.lambda > 0.0 && opt.sigma_min > 0.0) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(temp);
      if (es.info() == Eigen::Success) {
        const double eig_min = std::max(0.0, es.eigenvalues().minCoeff());
        const double sigma_min = std::sqrt(eig_min);
        const double ratio = sigma_min / opt.sigma_min;
        if (ratio < 1.0) {
          const double r = std::max(0.0, std::min(1.0, ratio));
          // Smooth cubic ramp: 1 - smoothstep(r) = 1 - (3r^2 - 2r^3).
          const double scale = 1.0 - (3.0 * r * r - 2.0 * r * r * r);
          const double lambda = opt.lambda * scale;
          lambda_sq = lambda * lambda;
        }
      } else {
        lambda_sq = opt.lambda * opt.lambda;
      }
    }
  }
  if (lambda_sq > 0.0) {
    temp.diagonal().array() += lambda_sq;
  }
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(temp);
  if (ldlt.info() != Eigen::Success) {
    log(LogLevel::Error, "rmrcIncrement: LDLT factorization failed");
    return Status::Failure;
  }
  Eigen::Matrix<double, 6, 7> X = ldlt.solve(J2);
  const Eigen::Matrix<double, 7, 1> dgamma = (gamma_f - gamma_i);
  const Eigen::Matrix<double, 6, 1> y = X * dgamma;

  const Eigen::VectorXd dq_primary = (s_jac.transpose() * y).eval();

  Eigen::VectorXd dq_total = dq_primary;

  // Optional nullspace term for redundant manipulators (dof > 6).
  const bool ns_enabled = opt.nullspace.enabled &&
                          (opt.nullspace.joint_limits.enabled || opt.nullspace.posture.enabled);
  if (ns_enabled) {
    int movable = 0;
    for (int i = 0; i < n; ++i) {
      if (model_.joint(i).type != JointType::Fixed) ++movable;
    }

    if (movable > 6) {
      Eigen::VectorXd z = Eigen::VectorXd::Zero(n);

      // ---- joint-limit avoidance ----
      if (opt.nullspace.joint_limits.enabled) {
        const double w_lim = opt.nullspace.joint_limits.weight;
        const double m_in = opt.nullspace.joint_limits.margin_frac;
        const double m = std::clamp(m_in, 0.0, 0.49);

        for (int i = 0; i < n; ++i) {
          const JointSpec& j = model_.joint(i);
          if (j.type == JointType::Fixed) continue;

          const JointLimit& lim = j.limit;
          if (!lim.enabled) continue;
          const double range = lim.upper - lim.lower;
          if (!(range > 1e-9) || !std::isfinite(range)) continue;

          const double mid = 0.5 * (lim.lower + lim.upper);
          const double half = 0.5 * range;
          if (!(half > 0.0) || !std::isfinite(half)) continue;

          const double qi = q_current(i);
          const double x = (qi - mid) / half;
          const double absx = std::abs(x);

          double w_act = 0.0;
          if (m <= 0.0) {
            w_act = 1.0;
          } else {
            const double x0 = 1.0 - 2.0 * m;
            if (absx > x0) {
              const double denom = std::max(1e-12, 1.0 - x0);  // = 2m
              w_act = std::clamp((absx - x0) / denom, 0.0, 1.0);
            }
          }

          if (!(w_act > 0.0)) continue;

          const double grad = 2.0 * (qi - mid) / (half * half);
          z(i) += -w_lim * w_act * grad;
        }
      }

      // ---- nominal posture tracking ----
      Eigen::VectorXd q_nominal;
      if (opt.nullspace.posture.enabled) {
        if (opt.nullspace.posture.q_nominal.has_value()) {
          const Eigen::VectorXd& qn = opt.nullspace.posture.q_nominal.value();
          if (qn.size() != n) {
            log(LogLevel::Error, "rmrcIncrement: q_nominal size mismatch");
            return Status::InvalidParameter;
          }
          q_nominal = qn;
        } else {
          // Default: mid-range for limited joints; otherwise current (no contribution).
          q_nominal = q_current;
          for (int i = 0; i < n; ++i) {
            const JointSpec& j = model_.joint(i);
            if (j.type == JointType::Fixed) continue;

            const JointLimit& lim = j.limit;
            if (!lim.enabled) continue;
            const double range = lim.upper - lim.lower;
            if (!(range > 1e-9) || !std::isfinite(range)) continue;
            q_nominal(i) = 0.5 * (lim.lower + lim.upper);
          }
        }

        const double w_post = opt.nullspace.posture.weight;
        for (int i = 0; i < n; ++i) {
          const JointSpec& j = model_.joint(i);
          if (j.type == JointType::Fixed) continue;

          double half = 1.0;
          const JointLimit& lim = j.limit;
          if (lim.enabled) {
            const double range = lim.upper - lim.lower;
            if (range > 1e-9 && std::isfinite(range)) {
              half = 0.5 * range;
            }
          }

          const double grad = 2.0 * (q_current(i) - q_nominal(i)) / (half * half);
          z(i) += -w_post * grad;
        }
      }

      if (z.allFinite() && z.squaredNorm() > 0.0) {
        // Project into damped nullspace: (I - J^T M^{-1} J) z = z - J^T M^{-1} (J z)
        const Eigen::MatrixXd M_inv_J = ldlt.solve(s_jac);  // 6 x n
        const Eigen::MatrixXd P = s_jac.transpose() * M_inv_J;  // n x n
        Eigen::VectorXd dq_null = z - (P * z);

        if (dq_null.allFinite() && dq_null.squaredNorm() > 0.0) {
          dq_null *= opt.nullspace.gain;

          // Per-joint clamp.
          const double frac = opt.nullspace.max_joint_step_frac;
          const double abs_cap = opt.nullspace.max_joint_step_abs;
          for (int i = 0; i < n; ++i) {
            const JointSpec& j = model_.joint(i);
            if (j.type == JointType::Fixed) continue;

            double max_i = 0.0;
            const JointLimit& lim = j.limit;
            if (frac > 0.0 && lim.enabled) {
              const double range = lim.upper - lim.lower;
              if (range > 1e-9 && std::isfinite(range)) {
                max_i = frac * range;
              }
            }
            if (!(max_i > 0.0) && abs_cap > 0.0) {
              max_i = abs_cap;
            }

            if (max_i > 0.0 && std::isfinite(max_i)) {
              dq_null(i) = std::clamp(dq_null(i), -max_i, max_i);
            }
          }

          // Norm clamp.
          double cap = std::numeric_limits<double>::infinity();
          if (opt.nullspace.max_norm_abs > 0.0) {
            cap = std::min(cap, opt.nullspace.max_norm_abs);
          }
          if (opt.nullspace.max_norm_ratio > 0.0) {
            cap = std::min(cap, opt.nullspace.max_norm_ratio * (dq_primary.norm() + 1e-9));
          }
          if (std::isfinite(cap) && cap > 0.0) {
            const double nrm = dq_null.norm();
            if (nrm > cap && nrm > 0.0) {
              dq_null *= (cap / nrm);
            }
          }

          dq_total += dq_null;
        }
      }
    }
  }

  *dq = dq_total;
  if (!dq->allFinite()) {
    log(LogLevel::Error, "rmrcIncrement: non-finite joint increments");
    return Status::Failure;
  }
  return Status::Success;
}

}  // namespace sclerp::core
