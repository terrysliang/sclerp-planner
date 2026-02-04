#include "sclerp/collision/avoidance.hpp"

#include "sclerp/collision/lemke.hpp"
#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/math/svd.hpp"

namespace sclerp::collision {

using sclerp::core::Status;
using sclerp::core::LogLevel;
using sclerp::core::log;

Status adjustJoints(
    double h,
    const std::vector<double>& dist_array,
    const Eigen::MatrixXd& contact_normal_array,
    double safe_dist,
    const Eigen::VectorXd& current_joint_values,
    const Eigen::VectorXd& next_joint_values,
    const std::vector<Eigen::MatrixXd>& j_contact_array,
    Eigen::VectorXd* adjusted_joint_values) {
  if (!adjusted_joint_values) {
    log(LogLevel::Error, "adjustJoints: null output");
    return Status::InvalidParameter;
  }

  const int nc = static_cast<int>(contact_normal_array.cols());
  if (nc == 0) {
    *adjusted_joint_values = next_joint_values;
    return Status::Success;
  }
  if (static_cast<int>(dist_array.size()) != nc) {
    log(LogLevel::Error, "adjustJoints: dist_array size mismatch");
    return Status::InvalidParameter;
  }
  if (static_cast<int>(j_contact_array.size()) != nc) {
    log(LogLevel::Error, "adjustJoints: j_contact_array size mismatch");
    return Status::InvalidParameter;
  }

  std::vector<int> violating_contacts;
  violating_contacts.reserve(dist_array.size());
  for (size_t i = 0; i < dist_array.size(); ++i) {
    if (dist_array[i] < safe_dist) {
      violating_contacts.push_back(static_cast<int>(i));
    }
  }

  if (violating_contacts.empty()) {
    *adjusted_joint_values = next_joint_values;
    return Status::Success;
  }

  const Eigen::VectorXd delta = next_joint_values - current_joint_values;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nc);
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nc, nc);

  std::vector<Eigen::MatrixXd> j_pinv;
  j_pinv.reserve(j_contact_array.size());
  for (const auto& jac : j_contact_array) {
    if (jac.cols() != current_joint_values.size()) {
      log(LogLevel::Error, "adjustJoints: contact Jacobian size mismatch");
      return Status::InvalidParameter;
    }
    j_pinv.push_back(sclerp::core::svdPseudoInverse(jac));
  }

  for (int n = 0; n < nc; ++n) {
    q[n] = dist_array[n] - safe_dist +
           h * contact_normal_array.col(n).transpose() * j_contact_array[n] * delta;

    for (int m = 0; m < nc; ++m) {
      M(n, m) = h * contact_normal_array.col(n).transpose() *
                j_contact_array[n] * j_pinv[m] * contact_normal_array.col(m);
    }
  }

  q *= 1000.0;
  M *= 1000.0;

  LemkeResult lemke_result;
  const Status lemke_status = lemkeSolve(q, M, &lemke_result);
  Eigen::VectorXd z = lemke_result.z;
  if (z.size() != nc) {
    log(LogLevel::Error, "adjustJoints: Lemke result size mismatch");
    return Status::Failure;
  }
  z /= 1000.0;

  Eigen::VectorXd comp_joint_values = Eigen::VectorXd::Zero(current_joint_values.size());
  for (int i = 0; i < nc; ++i) {
    comp_joint_values += j_pinv[i] * contact_normal_array.col(i) * z[i];
  }

  *adjusted_joint_values = next_joint_values + comp_joint_values;
  return lemke_status;
}

Status adjustJoints(
    double h,
    double safe_dist,
    const ContactSet& contacts,
    const Eigen::VectorXd& current_joint_values,
    const Eigen::VectorXd& next_joint_values,
    Eigen::VectorXd* adjusted_joint_values) {
  if (!adjusted_joint_values) {
    log(LogLevel::Error, "adjustJoints: null output");
    return Status::InvalidParameter;
  }

  const size_t nc = contacts.contacts.size();
  if (nc == 0) {
    *adjusted_joint_values = next_joint_values;
    return Status::Success;
  }

  std::vector<double> dist_array(nc, 0.0);
  Eigen::MatrixXd contact_normal_array = Eigen::MatrixXd::Zero(6, static_cast<int>(nc));
  std::vector<Eigen::MatrixXd> j_contact_array;
  j_contact_array.reserve(nc);

  for (size_t i = 0; i < nc; ++i) {
    const auto& contact = contacts.contacts[i];
    dist_array[i] = contact.distance;
    contact_normal_array.block<3,1>(0, static_cast<int>(i)) = contact.normal;
    j_contact_array.push_back(contact.J_contact);
  }

  return adjustJoints(h,
                      dist_array,
                      contact_normal_array,
                      safe_dist,
                      current_joint_values,
                      next_joint_values,
                      j_contact_array,
                      adjusted_joint_values);
}

}  // namespace sclerp::collision
