#pragma once
#include "sclerp/core/export.hpp"  
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/common/constants.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/model/manipulator_model.hpp"
#include "sclerp/core/dual_quat/dual_quat.hpp"

#include <cstdint>
#include <vector>

namespace sclerp::core {

enum class RmrcDamping : std::uint8_t {
  None = 0,
  Constant = 1,
  Adaptive = 2
};

struct RmrcOptions {
  RmrcDamping damping{RmrcDamping::Adaptive};
  double lambda{5e-2};     // constant damping or max damping for adaptive
  double sigma_min{2e-2};  // adaptive threshold for smallest singular value
};

class SCLERP_CORE_API KinematicsSolver {
public:
  explicit KinematicsSolver(ManipulatorModel model);

  const ManipulatorModel& model() const { return model_; }

  // FK end-effector
  Status forwardKinematics(const Eigen::VectorXd& q, Transform* g_base_tool) const;

  // FK transforms including base (index 0) and per-joint tips;
  // EE is appended when a tool frame exists.
  Status forwardKinematicsAll(const Eigen::VectorXd& q,
                              std::vector<Transform>* intermediate_transforms) const;

  // Space Jacobian (6 x n), ordering: [v; w]
  Status spatialJacobian(const Eigen::VectorXd& q, Eigen::MatrixXd* J_space) const;
  Status spatialJacobian(const Eigen::VectorXd& q,
                         Eigen::Ref<Eigen::MatrixXd> J_space) const;

  // Dual-quat RMRC
  struct RmrcWorkspace {
    Eigen::MatrixXd s_jac;  // 6 x n
  };
  Status rmrcIncrement(const DualQuat& dq_i,
                       const DualQuat& dq_f,
                       const Eigen::VectorXd& q_current,
                       Eigen::VectorXd* dq,
                       const RmrcOptions& opt,
                       RmrcWorkspace* ws) const;

private:
  ManipulatorModel model_;
};

}  // namespace sclerp::core
