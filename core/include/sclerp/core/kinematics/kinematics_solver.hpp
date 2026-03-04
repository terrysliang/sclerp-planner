#pragma once
#include "sclerp/core/export.hpp"
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/common/constants.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/model/manipulator_model.hpp"
#include "sclerp/core/dual_quat/dual_quat.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace sclerp::core {

// Kinematics + differential kinematics for a serial-chain `ManipulatorModel`.
//
// Key outputs are expressed in the same world frame as the model:
// - Space Jacobian is 6×n with twist ordering [v; w].
// - FK returns `g_base_tool` as a `Transform` (SE(3)).
// - If `model.base_offset()` is non-identity, FK/Jacobians are left-multiplied / adjointed by it.
enum class RmrcDamping : std::uint8_t {
  None = 0,
  Constant = 1,
  Adaptive = 2
};

struct RmrcOptions {
  RmrcDamping damping{RmrcDamping::Adaptive};
  double lambda{5e-2};     // constant damping or max damping for adaptive
  double sigma_min{2e-2};  // adaptive threshold for smallest singular value

  // Optional task-priority nullspace term for redundant manipulators.
  //
  // When enabled (and dof > 6), `rmrcIncrement(...)` computes:
  //   dq = dq_primary + N * dq_secondary
  // where:
  //   dq_primary   := damped least-squares RMRC step (existing behavior)
  //   N            := I - J^T (J J^T + lambda^2 I)^{-1} J  (damped nullspace projector)
  //   dq_secondary := weighted sum of:
  //     - joint-limit avoidance (push toward mid-range near limits)
  //     - nominal posture tracking (push toward q_nominal, defaulting to mid-range)
  struct RmrcNullspaceJointLimitOptions {
    bool enabled{false};
    double weight{1.0};
    // Activate in the last `margin_frac` fraction of joint range near each limit.
    // Example: 0.10 -> ramps on in the last 10% of (upper-lower) near each bound.
    double margin_frac{0.10};
  };

  struct RmrcNullspacePostureOptions {
    bool enabled{false};
    double weight{0.10};
    // If unset, defaults to per-joint mid-range when limits are enabled; otherwise equals q_current.
    std::optional<Eigen::VectorXd> q_nominal{};
  };

  struct RmrcNullspaceOptions {
    bool enabled{false};
    double gain{1.0};

    // Clamp only the nullspace contribution (after projection + gain).
    double max_joint_step_frac{0.05};  // fraction of (upper-lower) when limits enabled
    double max_joint_step_abs{0.25};   // absolute clamp when limits disabled
    double max_norm_ratio{0.50};       // ||dq_null|| <= max_norm_ratio*(||dq_primary|| + 1e-9)
    double max_norm_abs{0.50};         // absolute norm cap for dq_null

    RmrcNullspaceJointLimitOptions joint_limits{};
    RmrcNullspacePostureOptions posture{};
  };

  RmrcNullspaceOptions nullspace{};
};

class SCLERP_CORE_API KinematicsSolver {
public:
  explicit KinematicsSolver(ManipulatorModel model);

  const ManipulatorModel& model() const { return model_; }

  // Forward kinematics for the end-effector/tool frame.
  //
  // `q` must be size `model().dof()`. Output is base->tool in the model/world frame.
  Status forwardKinematics(const Eigen::VectorXd& q, Transform* g_base_tool) const;

  // Forward kinematics for intermediate frames.
  //
  // Output includes:
  // - index 0: base frame
  // - indices 1..dof: per-joint tip frames (base->joint_tip)
  // - optional last index: tool frame (when `model().has_tool_frame()` is true)
  Status forwardKinematicsAll(const Eigen::VectorXd& q,
                              std::vector<Transform>* intermediate_transforms) const;

  // Space/world Jacobian (6×n), ordering: [v; w] (linear; angular).
  Status spatialJacobian(const Eigen::VectorXd& q, Eigen::MatrixXd* J_space) const;
  Status spatialJacobian(const Eigen::VectorXd& q,
                         Eigen::Ref<Eigen::MatrixXd> J_space) const;

  // Space Jacobian prefix up to (and including) `link_index`.
  // `J_space_prefix` must be 6×n; columns after link_index may be left unmodified.
  Status spatialJacobianUpToLink(const Eigen::VectorXd& q, int link_index,
                                 Eigen::Ref<Eigen::MatrixXd> J_space_prefix) const;

  // Point Jacobian prefix for a point expressed in the world frame (3×n).
  // `J_point_prefix` must be 3×n; columns after link_index may be left unmodified.
  Status pointJacobianUpToLink(const Eigen::VectorXd& q, int link_index,
                               const Vec3& point_space,
                               Eigen::Ref<Eigen::MatrixXd> J_point_prefix) const;

  // Dual-quat RMRC:
  // Computes a joint-space direction `dq` (delta-q) that moves the end-effector from dq_i toward dq_f.
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
