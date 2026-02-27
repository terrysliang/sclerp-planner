// Collision avoidance correction via an LCP (complementarity) solve.
//
// High-level:
// - Build a per-contact linear constraint using closest-point normals and point Jacobians.
// - Solve an LCP for non-negative impulses z that activate only when constraints are tight.
// - Map those impulses back into joint space via analytic pseudoinverses (3×3 solves).
#include "sclerp/collision/avoidance.hpp"

#include "sclerp/collision/lemke.hpp"
#include "sclerp/core/common/logger.hpp"

#include <algorithm>

namespace sclerp::collision {

using sclerp::core::Status;
using sclerp::core::LogLevel;
using sclerp::core::log;

static Status pseudoInverseAnalytic3xk(
    const Eigen::MatrixXd& J_3xk,
    double lambda,
    Eigen::MatrixXd* J_pinv_kx3) {
  if (!J_pinv_kx3) return Status::InvalidParameter;
  if (J_3xk.rows() != 3) return Status::InvalidParameter;

  // J^† = J^T (J J^T + λI)^-1
  Eigen::Matrix3d A = (J_3xk * J_3xk.transpose()).eval();
  A.diagonal().array() += lambda;

  Eigen::LDLT<Eigen::Matrix3d> ldlt(A);
  if (ldlt.info() != Eigen::Success) {
    log(LogLevel::Error, "pseudoInverseAnalytic3xk: LDLT failed");
    return Status::Failure;
  }

  *J_pinv_kx3 = J_3xk.transpose() * ldlt.solve(Eigen::Matrix3d::Identity());
  return Status::Success;
}

// expects normals as 3×nc and each J_contact is 3×k_i (k_i varies per contact).
static Status adjustJointsFromArrays(
    double h,
    double safe_dist,
    double pinv_lambda,
    const std::vector<double>& dist_array,         // size nc
    const Eigen::MatrixXd& contact_normal_array,   // 3×nc (only 3 rows used)
    const Eigen::VectorXd& current_joint_values,  
    const Eigen::VectorXd& next_joint_values,
    const std::vector<Eigen::MatrixXd>& j_contact_array, // nc of (3×k_i)
    Eigen::VectorXd* adjusted_joint_values) {
  if (!adjusted_joint_values) {
    log(LogLevel::Error, "adjustJoints: null output");
    return Status::InvalidParameter;
  }

  const int dof = static_cast<int>(current_joint_values.size());
  if (next_joint_values.size() != dof) return Status::InvalidParameter;

  const int nc = static_cast<int>(contact_normal_array.cols());
  if (nc == 0) {
    *adjusted_joint_values = next_joint_values;
    return Status::Success;
  }
  if (contact_normal_array.rows() != 3) {
    log(LogLevel::Error, "adjustJoints: contact_normal_array must be 3×nc");
    return Status::InvalidParameter;
  }
  if (static_cast<int>(dist_array.size()) != nc) {
    log(LogLevel::Error, "adjustJoints: dist_array size mismatch");
    return Status::InvalidParameter;
  }
  if (static_cast<int>(j_contact_array.size()) != nc) {
    log(LogLevel::Error, "adjustJoints: j_contact_array size mismatch");
    return Status::InvalidParameter;
  }

  const Eigen::VectorXd delta = next_joint_values - current_joint_values;

  // Precompute analytic pseudoinverses for each contact (k_i×3)
  std::vector<Eigen::MatrixXd> j_pinv;
  j_pinv.resize(static_cast<size_t>(nc));

  std::vector<int> k_cols(nc, 0);
  for (int i = 0; i < nc; ++i) {
    const auto& J = j_contact_array[static_cast<size_t>(i)];
    if (J.rows() != 3) {
      log(LogLevel::Error, "adjustJoints: each contact Jacobian must be 3×k");
      return Status::InvalidParameter;
    }
    const int k = static_cast<int>(J.cols());
    if (k <= 0 || k > dof) {
      log(LogLevel::Error, "adjustJoints: invalid k in contact Jacobian");
      return Status::InvalidParameter;
    }
    k_cols[i] = k;

    Eigen::MatrixXd Jpinv;
    const Status st = pseudoInverseAnalytic3xk(J, pinv_lambda, &Jpinv);
    if (!ok(st)) return st;
    j_pinv[static_cast<size_t>(i)] = std::move(Jpinv); // k×3
  }

  // Build LCP: w = q + M z, with w>=0, z>=0, w^T z = 0
  Eigen::VectorXd q_lcp = Eigen::VectorXd::Zero(nc);
  Eigen::MatrixXd M_lcp = Eigen::MatrixXd::Zero(nc, nc);

  for (int n = 0; n < nc; ++n) {
    const double dist_n = std::max(dist_array[static_cast<size_t>(n)], 0.0);
    const Eigen::Vector3d nn = contact_normal_array.col(n);

    const int kn = k_cols[n];
    const auto& Jn = j_contact_array[static_cast<size_t>(n)]; // 3×kn

    // q[n] = d - d_safe + h * n^T * J * delta
    const double vel_term = nn.transpose() * (Jn * delta.head(kn));
    q_lcp[n] = (dist_n - safe_dist) + h * vel_term;

    for (int m = 0; m < nc; ++m) {
      const Eigen::Vector3d nm = contact_normal_array.col(m);

      const int km = k_cols[m];
      const auto& Jpinv_m = j_pinv[static_cast<size_t>(m)]; // km×3

      // Need Jn_full(3×dof) * Jpinv_m_full(dof×3) but avoid building full:
      // Equivalent: Jn(3×kn) * Jpinv_m_padded(kn×3),
      // where Jpinv_m_padded.topRows(min(kn,km)) = Jpinv_m.topRows(min(kn,km)).
      Eigen::MatrixXd Jpinv_m_to_kn = Eigen::MatrixXd::Zero(kn, 3);
      const int k_overlap = std::min(kn, km);
      Jpinv_m_to_kn.topRows(k_overlap) = Jpinv_m.topRows(k_overlap);

      const Eigen::Matrix3d T = (Jn * Jpinv_m_to_kn).eval(); // 3×3
      M_lcp(n, m) = h * (nn.transpose() * T * nm)(0, 0);
    }
  }

  // helps numerical behavior in Lemke
  constexpr double kScale = 1000.0;
  q_lcp *= kScale;
  M_lcp *= kScale;

  LemkeResult lemke_result;
  const Status lemke_status = lemkeSolve(q_lcp, M_lcp, &lemke_result);
  if (!ok(lemke_status)) return lemke_status;

  Eigen::VectorXd z = lemke_result.z;
  if (z.size() != nc) {
    log(LogLevel::Error, "adjustJoints: Lemke result size mismatch");
    return Status::Failure;
  }
  z /= kScale;

  // Convert complementarity impulses z into joint corrections
  Eigen::VectorXd comp_joint_values = Eigen::VectorXd::Zero(dof);
  for (int i = 0; i < nc; ++i) {
    const int k = k_cols[i];
    const Eigen::Vector3d ni = contact_normal_array.col(i);

    // dq_i (k×1) = J^† (k×3) * (n (3×1) * z_i)
    const Eigen::VectorXd dq_i = j_pinv[static_cast<size_t>(i)] * (ni * z[i]); // k×1
    comp_joint_values.head(k) += dq_i;
  }

  // Safeguard
  const double delta_norm = delta.norm();
  if (delta_norm > 0.0) {
    const double max_comp_norm = 3.0 * delta_norm;
    const double comp_norm = comp_joint_values.norm();
    if (comp_norm > max_comp_norm) {
      comp_joint_values *= (max_comp_norm / comp_norm);
    }
  }

  *adjusted_joint_values = next_joint_values + comp_joint_values;
  return Status::Success;
}

Status adjustJoints(
    const CollisionAvoidanceOptions& opt,
    const ContactSet& contacts,
    const Eigen::VectorXd& current_joint_values,
    const Eigen::VectorXd& next_joint_values,
    Eigen::VectorXd* adjusted_joint_values) {

  if (!adjusted_joint_values) return Status::InvalidParameter;
  if (opt.dt <= 0.0 || opt.safe_dist <= 0.0) return Status::InvalidParameter;

  const int dof = static_cast<int>(current_joint_values.size());
  if (next_joint_values.size() != dof) return Status::InvalidParameter;

  const int nc = static_cast<int>(contacts.contacts.size());
  if (nc == 0) {
    *adjusted_joint_values = next_joint_values;
    return Status::Success;
  }

  const double safe_dist = opt.safe_dist;
  const double comp_activate_tol = std::max(opt.comp_activate_tol, safe_dist);

  std::vector<double> dist_array(static_cast<size_t>(nc), 0.0);
  Eigen::MatrixXd contact_normal_array(3, nc);
  std::vector<Eigen::MatrixXd> j_contact_array;
  j_contact_array.reserve(static_cast<size_t>(nc));

  for (int i = 0; i < nc; ++i) {
    const Contact& c = contacts.contacts[static_cast<size_t>(i)];

    const double d = std::max(c.distance, 0.0);
    dist_array[static_cast<size_t>(i)] = d;

    if (c.J_contact.rows() != 3 || c.J_contact.cols() <= 0 || c.J_contact.cols() > dof) {
      log(LogLevel::Error, "adjustJoints: invalid reduced contact Jacobian (expected 3×k)");
      return Status::InvalidParameter;
    }

    if (d <= comp_activate_tol) {
      contact_normal_array.col(i) = c.normal;               // active
      j_contact_array.push_back(c.J_contact);               // active
    } else {
      contact_normal_array.col(i).setZero();                // inactive => no coupling
      j_contact_array.push_back(Eigen::MatrixXd::Zero(3, c.J_contact.cols()));
    }
  }

  return adjustJointsFromArrays(
      opt.dt,
      safe_dist,
      opt.pinv_lambda,
      dist_array,
      contact_normal_array,
      current_joint_values,
      next_joint_values,
      j_contact_array,
      adjusted_joint_values);
  }

}  // namespace sclerp::collision
