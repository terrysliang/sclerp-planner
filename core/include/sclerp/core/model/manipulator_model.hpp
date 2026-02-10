#pragma once
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/common/constants.hpp"
#include "sclerp/core/math/types.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sclerp::core {

// Minimal serial-chain manipulator model for POE kinematics.
//
// Conventions:
// - Joint axes/points are expressed in the base_link frame at q=0 (space frame).
// - For revolute joints: `axis` is unit direction w and `point` is a point q on the axis.
// - `joint_tip_home` is the home transform for that joint's tip frame (base -> tip at q=0).
// - `ee_home` (M in POE) is the end-effector home transform (base -> tool at q=0).
// - `S_space` caches the 6×n space screw axes with ordering [v; w].
enum class JointType : std::uint8_t {
  Revolute = 0,
  Prismatic = 1,
  Fixed = 2
};

struct JointLimit {
  double lower = 0.0;
  double upper = 0.0;
  bool enabled = false;
};

struct JointSpec {
  std::string name;
  JointType type{JointType::Revolute};

  // Space-frame axis direction (unit expected)
  Vec3 axis = Vec3::UnitZ();

  // A point on axis in space frame (used for revolute POE)
  Vec3 point = Vec3::Zero();

  JointLimit limit{};
  Transform joint_tip_home = Transform::Identity();
};

class ManipulatorModel {
public:
  ManipulatorModel() = default;

  // Initialize model with joints + EE home transform gst0_
  Status init(std::vector<JointSpec> joints,
              const Transform& ee_home,
              const Thresholds& thr = kDefaultThresholds);

  int dof() const { return static_cast<int>(joints_.size()); }

  const std::vector<JointSpec>& joints() const { return joints_; }
  const JointSpec& joint(int i) const { return joints_.at(static_cast<size_t>(i)); }

  const std::vector<std::string>& joint_names() const { return joint_names_; }

  const Transform& ee_home() const { return ee_home_; }       // gst0_
  bool has_tool_frame() const { return has_tool_frame_; }
  void set_has_tool_frame(bool has_tool_frame) { has_tool_frame_ = has_tool_frame; }
  const Transform& base_offset() const { return base_offset_; }
  bool has_base_offset() const { return has_base_offset_; }
  void set_base_offset(const Transform& base_offset) {
    base_offset_ = base_offset;
    has_base_offset_ = !base_offset_.isApprox(Transform::Identity(), 1e-12);
  }
  const ScrewMatrix& S_space() const { return S_space_; }     // cached 6 x n, [v; w]

  // Optional: per-joint reference transforms gst0_i (base -> joint frame at zero)
  Status set_joint_home_transforms(std::vector<Transform> gst0_i);
  const std::optional<std::vector<Transform>>& joint_home_transforms() const { return gst0_i_; }

  // Validation / limit helpers
  Status validate(const Thresholds& thr = kDefaultThresholds) const;
  bool within_limits(const Eigen::VectorXd& q, double tol = 0.0) const;
  Status clamp_to_limits(const Eigen::VectorXd& q, Eigen::VectorXd* out) const;

private:
  Status rebuild_cache(const Thresholds& thr);

  std::vector<JointSpec> joints_;

  std::vector<std::string> joint_names_;

  Transform ee_home_{Transform::Identity()};
  bool has_tool_frame_{false};
  Transform base_offset_{Transform::Identity()};  // world -> base_link (fixed)
  bool has_base_offset_{false};
  ScrewMatrix S_space_;  // 6 x n, [v; w]

  std::optional<std::vector<Transform>> gst0_i_; // optional
};

class ManipulatorBuilder {
public:
  ManipulatorBuilder& set_ee_home(const Transform& ee_home) {
    ee_home_ = ee_home;
    return *this;
  }

  ManipulatorBuilder& add_revolute(std::string name,
                                   const Vec3& w_unit,
                                   const Vec3& q_point,
                                   JointLimit limit = {},
                                   Transform joint_tip_home = Transform::Identity());

  ManipulatorBuilder& add_prismatic(std::string name,
                                    const Vec3& v_unit,
                                    JointLimit limit = {},
                                    Transform joint_tip_home = Transform::Identity());

  ManipulatorBuilder& add_fixed(std::string name,
                                Transform joint_tip_home = Transform::Identity());

  // Builds and validates the model using provided thresholds.
  Status build(ManipulatorModel* out, const Thresholds& thr = kDefaultThresholds) const;

private:
  std::vector<JointSpec> joints_;
  std::optional<Transform> ee_home_;
};

}  // namespace sclerp::core
