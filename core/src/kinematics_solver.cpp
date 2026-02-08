#include "sclerp/core/kinematics/kinematics_solver.hpp"

#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/math/so3.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

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

static AdjointMatrix adjointVW(const Transform& T) {
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
    const AdjointMatrix Ad = adjointVW(prod);
    J_space.col(i) = Ad * S_space.col(i);
  }

  if (model_.has_base_offset()) {
    const AdjointMatrix Ad_base = adjointVW(model_.base_offset());
    J_space = Ad_base * J_space;
  }
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

  dq->noalias() = s_jac.transpose() * y;
  if (!dq->allFinite()) {
    log(LogLevel::Error, "rmrcIncrement: non-finite joint increments");
    return Status::Failure;
  }
  return Status::Success;
}

}  // namespace sclerp::core
