#include "sclerp/urdf/load_manipulator.hpp"

#include "sclerp/core/common/logger.hpp"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <Eigen/Geometry>


#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace sclerp::urdf {
using namespace sclerp::core;

static Transform poseToEigen(const ::urdf::Pose& p) {
  Transform T = Transform::Identity();
  T.translation() = Vec3(p.position.x, p.position.y, p.position.z);

  double rr = 0.0, rp = 0.0, ry = 0.0;
  p.rotation.getRPY(rr, rp, ry);

  Eigen::AngleAxisd Rx(rr, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Ry(rp, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rz(ry, Eigen::Vector3d::UnitZ());
  // URDF rpy: fixed-axis roll-pitch-yaw, applied in X-Y-Z order
  T.linear() = (Rz * Ry * Rx).toRotationMatrix();
  return T;
}

static Status readFileToString(const std::string& path, std::string* out) {
  if (!out) return Status::InvalidParameter;
  std::ifstream ifs(path);
  if (!ifs) return Status::Failure;
  std::ostringstream oss;
  oss << ifs.rdbuf();
  *out = oss.str();
  return Status::Success;
}

static LoadResult fail(Status st, std::string msg) {
  LoadResult r;
  r.status = st;
  r.message = std::move(msg);
  if (shouldLog((st == Status::Failure) ? LogLevel::Error : LogLevel::Warn)) {
    log((st == Status::Failure) ? LogLevel::Error : LogLevel::Warn, r.message);
  }
  return r;
}

static bool isFiniteVec3(const Vec3& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

static bool supportedJointType(decltype(::urdf::Joint().type) t) {
  return t == ::urdf::Joint::REVOLUTE ||
         t == ::urdf::Joint::CONTINUOUS ||
         t == ::urdf::Joint::PRISMATIC ||
         t == ::urdf::Joint::FIXED;
}

LoadResult loadManipulatorModelFromString(const std::string& urdf_xml,
                                          const LoadOptions& opt) {
  if (opt.base_link.empty() || opt.tip_link.empty()) {
    return fail(Status::InvalidParameter, "base_link / tip_link must be provided");
  }
  if (opt.base_link == opt.tip_link) {
    return fail(Status::InvalidParameter, "base_link == tip_link: zero-DOF chain not supported yet");
  }

  ::urdf::ModelInterfaceSharedPtr model = ::urdf::parseURDF(urdf_xml);
  if (!model) {
    return fail(Status::Failure, "urdf::parseURDF failed");
  }

  ::urdf::LinkConstSharedPtr tip = model->getLink(opt.tip_link);
  if (!tip) {
    return fail(Status::InvalidParameter, "tip_link not found: " + opt.tip_link);
  }

  // Walk from tip -> base using parent joints.
  std::vector<::urdf::JointConstSharedPtr> joints_tip_to_base;
  ::urdf::LinkConstSharedPtr cur = tip;

  while (cur && cur->name != opt.base_link) {
    ::urdf::JointConstSharedPtr pj = cur->parent_joint;
    if (!pj) {
      return fail(Status::InvalidParameter,
                  "No parent_joint while walking from tip to base. base_link might not be an ancestor.");
    }

    if (opt.strict && !supportedJointType(pj->type)) {
      return fail(Status::InvalidParameter,
                  "Unsupported joint type in chain: " + pj->name);
    }

    joints_tip_to_base.push_back(pj);

    // Move to parent link
    ::urdf::LinkConstSharedPtr pl = cur->getParent();
    if (!pl) {
      // Some urdfdom versions don't have getParent(); fallback via parent link name.
      if (pj->parent_link_name.empty()) {
        return fail(Status::InvalidParameter, "Cannot find parent link for joint: " + pj->name);
      }
      pl = model->getLink(pj->parent_link_name);
      if (!pl) {
        return fail(Status::InvalidParameter, "Parent link not found: " + pj->parent_link_name);
      }
    }
    cur = pl;
  }

  if (!cur || cur->name != opt.base_link) {
    return fail(Status::InvalidParameter,
                "base_link not reached from tip_link. Check base_link and tip_link.");
  }

  // Reverse to get base -> tip order
  std::reverse(joints_tip_to_base.begin(), joints_tip_to_base.end());

  // Build ManipulatorModel joints (movable only), but always propagate transforms (fixed included).
  std::vector<JointSpec> joints;
  joints.reserve(joints_tip_to_base.size());

  Transform T_base_parent = Transform::Identity();  // base_link frame is identity by definition

  for (const auto& j : joints_tip_to_base) {
    const Transform T_parent_child0 = poseToEigen(j->parent_to_joint_origin_transform);
    const Transform T_base_child0 = T_base_parent * T_parent_child0;

    if (j->type == ::urdf::Joint::FIXED) {
      if (opt.collapse_fixed_joints) {
        // Collapse fixed joints into the chain transforms.
        T_base_parent = T_base_child0;
        continue;
      }

      JointSpec js;
      js.name = j->name;
      js.type = JointType::Fixed;
      js.axis.setZero();
      js.point.setZero();
      js.limit.enabled = false;
      js.joint_tip_home = T_base_child0;

      joints.push_back(std::move(js));
      T_base_parent = T_base_child0;
      continue;
    }

    JointSpec js;
    js.name = j->name;
    js.joint_tip_home = T_base_child0;

    // Axis in joint frame, convert to space at q=0
    const Vec3 axis_joint(j->axis.x, j->axis.y, j->axis.z);
    if (!isFiniteVec3(axis_joint)) {
      return fail(Status::InvalidParameter,
                  "Non-finite joint axis for joint: " + j->name);
    }
    if (!(axis_joint.norm() > kDefaultThresholds.axis_norm_eps)) {
      return fail(Status::InvalidParameter,
                  "Joint axis near zero for joint: " + j->name);
    }
    Vec3 axis_space = T_base_child0.rotation() * axis_joint;
    axis_space.normalize();

    // Limits
    if (j->limits) {
      js.limit.enabled = true;
      js.limit.lower = j->limits->lower;
      js.limit.upper = j->limits->upper;
    } else {
      js.limit.enabled = false;
    }

    if (j->type == ::urdf::Joint::REVOLUTE || j->type == ::urdf::Joint::CONTINUOUS) {
      js.type = JointType::Revolute;
      js.axis = axis_space;

      // Point on axis in space (joint origin position)
      js.point = T_base_child0.translation();

      // Continuous joint: no limits
      if (j->type == ::urdf::Joint::CONTINUOUS) {
        js.limit.enabled = false;
      }
    } else if (j->type == ::urdf::Joint::PRISMATIC) {
      js.type = JointType::Prismatic;
      js.axis = axis_space;
      js.point.setZero();  // unused
    } else {
      if (opt.strict) {
        return fail(Status::InvalidParameter, "Unsupported joint type in chain: " + j->name);
      }
      // non-strict: treat as fixed
      if (opt.collapse_fixed_joints) {
        T_base_parent = T_base_child0;
        continue;
      }

      JointSpec js_fixed;
      js_fixed.name = j->name;
      js_fixed.type = JointType::Fixed;
      js_fixed.axis.setZero();
      js_fixed.point.setZero();
      js_fixed.limit.enabled = false;
      js_fixed.joint_tip_home = T_base_child0;

      joints.push_back(std::move(js_fixed));
      T_base_parent = T_base_child0;
      continue;
    }

    joints.push_back(std::move(js));

    // Move to child link frame for next joint
    T_base_parent = T_base_child0;
  }

  if (joints.empty()) {
    return fail(Status::InvalidParameter, "No movable joints found in chain");
  }

  // At the end of the traversal, T_base_parent is base -> tip_link at q=0
  // But careful: if the last joint is movable, we updated T_base_parent to its child0 already.
  // If there are fixed joints after last movable joint, T_base_parent includes them too (because we always propagate).
  const Transform T_base_tip0 = T_base_parent;

  const Transform ee_home = T_base_tip0 * opt.tool_offset;

  ManipulatorModel out_model;
  const Status st = out_model.init(std::move(joints), ee_home);
  if (!ok(st)) {
    return fail(st, "ManipulatorModel init failed");
  }

  LoadResult res;
  res.status = Status::Success;
  res.model = std::move(out_model);
  res.message = "OK";
  return res;
}

LoadResult loadManipulatorModelFromFile(const std::string& urdf_path,
                                        const LoadOptions& opt) {
  std::string xml;
  const Status st = readFileToString(urdf_path, &xml);
  if (!ok(st)) {
    return fail(st, "Failed to open URDF file: " + urdf_path);
  }
  return loadManipulatorModelFromString(xml, opt);
}

}  // namespace sclerp::urdf
