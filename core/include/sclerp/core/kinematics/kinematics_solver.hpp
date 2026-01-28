#pragma once
#include "sclerp/core/export.hpp"  
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/common/constants.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/model/manipulator_model.hpp"
#include "sclerp/core/dual_quat/dual_quat.hpp"

#include <vector>

namespace sclerp::core {

class SCLERP_CORE_API KinematicsSolver {
public:
  explicit KinematicsSolver(ManipulatorModel model);

  const ManipulatorModel& model() const { return model_; }

  // FK end-effector
  Status forwardKinematics(const Eigen::VectorXd& q, Transform* g_base_tool) const;

  // FK intermediate transforms (per joint tip); EE is last element if available
  Status forwardKinematicsAll(const Eigen::VectorXd& q,
                              std::vector<Transform>* intermediate_transforms) const;

  // Space Jacobian (6 x n), ordering: [v; w]
  Status spatialJacobian(const Eigen::VectorXd& q, Eigen::MatrixXd* J_space) const;

  // Dual-quat RMRC
  Status rmrcIncrement(const DualQuat& dq_i,
                       const DualQuat& dq_f,
                       const Eigen::VectorXd& q_current,
                       Eigen::VectorXd* dq) const;

private:
  ManipulatorModel model_;
};

}  // namespace sclerp::core
