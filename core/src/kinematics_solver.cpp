#include "sclerp/core/kinematics/kinematics_solver.hpp"

#include "sclerp/core/math/adjoint.hpp"
#include "sclerp/core/math/se3.hpp"
#include "sclerp/core/math/svd.hpp"

#include <Eigen/Dense>
#include <stdexcept>

namespace sclerp::core {

KinematicsSolver::KinematicsSolver(ManipulatorModel model)
  : model_(std::move(model)) {}

Status KinematicsSolver::forwardKinematics(const Eigen::VectorXd& q,
                                           Transform* g_base_tool) const {
  if (!g_base_tool) return Status::InvalidParameter;
  if (q.size() != model_.dof()) return Status::InvalidParameter;

  const int n = model_.dof();
  const ScrewMatrix& S = model_.S_space();

  Transform prod = Transform::Identity();
  for (int i = 0; i < n; ++i) {
    const Twist xi = S.col(i) * q(i);
    prod = prod * expSE3(xi);
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
  const ScrewMatrix& S = model_.S_space();

  Transform prod = Transform::Identity();
  for (int i = 0; i < n; ++i) {
    const Twist xi = S.col(i) * q(i);
    prod = prod * expSE3(xi);

    // Mimic kinlib “joint tip” idea:
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
  const ScrewMatrix& S = model_.S_space();

  J_space->resize(6, n);

  Transform prod = Transform::Identity();  // Π_{k<i} exp(Sk qk)
  for (int i = 0; i < n; ++i) {
    if (i == 0) {
      J_space->col(i) = S.col(i);
    } else {
      const AdjointMatrix Ad = adjoint(prod);
      J_space->col(i) = Ad * S.col(i);
    }

    // update prod for next column
    const Twist xi = S.col(i) * q(i);
    prod = prod * expSE3(xi);
  }

  return Status::Success;
}

Status KinematicsSolver::rmrcIncrement(const Transform& g_current,
                                       const Transform& g_goal,
                                       const Eigen::VectorXd& q_current,
                                       Eigen::VectorXd* dq,
                                       const RmrcOptions& opt) const {
  if (!dq) return Status::InvalidParameter;
  if (q_current.size() != model_.dof()) return Status::InvalidParameter;

  Eigen::MatrixXd J;
  const Status stJ = spatialJacobian(q_current, &J);
  if (!ok(stJ)) return stJ;

  // Error twist: g_rel = g_current^{-1} g_goal
  // logSE3(g_rel) gives a twist in the CURRENT/body frame.
  // Convert to spatial frame via Ad_g_current.
  const Transform g_rel = g_current.inverse() * g_goal;
  Twist xi_body = logSE3(g_rel);
  Twist xi_space = adjoint(g_current) * xi_body;

  // Apply gains (rot on omega, pos on v)
  xi_space.head<3>() *= opt.rot_gain;
  xi_space.tail<3>() *= opt.pos_gain;

  // Pseudoinverse solve
  const Eigen::MatrixXd J_pinv = svdPseudoInverse(J, opt.jac.svd_tol);
  Eigen::VectorXd step = J_pinv * xi_space;

  // Integrate step size (RMRC discrete step)
  *dq = step * opt.step;
  return Status::Success;
}

Status KinematicsSolver::rmrcIncrement(const DualQuat& dq_i,
                                       const DualQuat& dq_f,
                                       const Eigen::VectorXd& q_current,
                                       Eigen::VectorXd* dq,
                                       const RmrcOptions& opt) const {
  return rmrcIncrement(dq_i.toTransform(), dq_f.toTransform(), q_current, dq, opt);
}

}  // namespace sclerp::core
