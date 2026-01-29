#include "sclerp/core/kinematics/kinematics_solver.hpp"

#include "sclerp/core/math/so3.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdexcept>

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
  if (!g_base_tool) return Status::InvalidParameter;
  if (q.size() != model_.dof()) return Status::InvalidParameter;

  const int n = model_.dof();
  Transform prod = Transform::Identity();
  for (int i = 0; i < n; ++i) {
    prod = prod * jointExp(model_.joint(i), q(i));
  }

  *g_base_tool = prod * model_.ee_home();
  return Status::Success;
}

Status KinematicsSolver::forwardKinematicsAll(const Eigen::VectorXd& q,
                                              std::vector<Transform>* intermediate_transforms) const {
  if (!intermediate_transforms) return Status::InvalidParameter;
  if (q.size() != model_.dof()) return Status::InvalidParameter;

  intermediate_transforms->clear();
  intermediate_transforms->reserve(static_cast<std::size_t>(model_.dof()) + 1);

  const int n = model_.dof();
  Transform prod = Transform::Identity();
  for (int i = 0; i < n; ++i) {
    prod = prod * jointExp(model_.joint(i), q(i));

    // each JointSpec has joint_tip_home (fixed home transform for that joint's tip frame)
    // so g_i(q) = (Π exp(Sk qk)) * M_i
    const Transform g_tip = prod * model_.joint(i).joint_tip_home;
    intermediate_transforms->push_back(g_tip);
  }

  // Append EE as last element (like many FK-all APIs)
  intermediate_transforms->push_back(prod * model_.ee_home());
  return Status::Success;
}

Status KinematicsSolver::spatialJacobian(const Eigen::VectorXd& q,
                                         Eigen::MatrixXd* J_space) const {
  if (!J_space) return Status::InvalidParameter;
  if (q.size() != model_.dof()) return Status::InvalidParameter;

  const int n = model_.dof();
  J_space->resize(6, n);

  const ScrewMatrix& S_space = model_.S_space(); // [w; v] columns

  if (n > 0) {
    Eigen::Matrix<double, 6, 1> twist;
    twist.head<3>() = S_space.block<3,1>(3, 0);  // v
    twist.tail<3>() = S_space.block<3,1>(0, 0);  // w
    J_space->col(0) = twist;
  }

  Eigen::Matrix<double, 6, 1> twist;
  Transform prod = Transform::Identity();  // Π_{k<i} exp(joint_k)
  for (int i = 1; i < n; ++i) {
    prod = prod * jointExp(model_.joint(i - 1), q(i - 1));
    const AdjointMatrix Ad = adjointVW(prod);
    twist.head<3>() = S_space.block<3,1>(3, i);  // v
    twist.tail<3>() = S_space.block<3,1>(0, i);  // w
    J_space->col(i) = Ad * twist;
  }

  return Status::Success;
}

Status KinematicsSolver::rmrcIncrement(const DualQuat& dq_i,
                                       const DualQuat& dq_f,
                                       const Eigen::VectorXd& q_current,
                                       Eigen::VectorXd* dq) const {
  if (!dq) return Status::InvalidParameter;
  if (q_current.size() != model_.dof()) return Status::InvalidParameter;

  // RMRC in dual-quat form.
  const Transform g_i = dq_i.toTransform();
  const Transform g_f = dq_f.toTransform();

  Eigen::Matrix<double, 7, 1> gamma_i;
  Eigen::Matrix<double, 7, 1> gamma_f;

  const Vec3 p_i = g_i.translation();
  gamma_i.head<3>() = p_i;
  const Quat q_i = dq_i.real();
  gamma_i(3) = q_i.w();
  gamma_i(4) = q_i.x();
  gamma_i(5) = q_i.y();
  gamma_i(6) = q_i.z();

  const Vec3 p_f = g_f.translation();
  gamma_f.head<3>() = p_f;
  const Quat q_f = dq_f.real();
  gamma_f(3) = q_f.w();
  gamma_f(4) = q_f.x();
  gamma_f(5) = q_f.y();
  gamma_f(6) = q_f.z();

  Eigen::MatrixXd s_jac;
  const Status st = spatialJacobian(q_current, &s_jac);
  if (!ok(st)) return st;

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
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(temp);
  Eigen::Matrix<double, 6, 7> X = ldlt.solve(J2);
  const Eigen::Matrix<double, 7, 1> dgamma = (gamma_f - gamma_i);
  const Eigen::Matrix<double, 6, 1> y = X * dgamma;

  dq->noalias() = s_jac.transpose() * y;
  return Status::Success;
}

}  // namespace sclerp::core
