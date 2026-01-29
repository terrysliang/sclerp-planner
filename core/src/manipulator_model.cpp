#include "sclerp/core/model/manipulator_model.hpp"

#include "sclerp/core/common/logger.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sclerp::core {

static inline bool isFiniteVec3(const Vec3& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

static inline Status normalizeOrAssign(Vec3* a, double eps) {
  if (!a) {
    log(LogLevel::Error, "ManipulatorModel: null axis pointer");
    return Status::InvalidParameter;
  }
  const Vec3& v = *a;
  if (!isFiniteVec3(v)) {
    log(LogLevel::Error, "ManipulatorModel: axis is non-finite");
    return Status::InvalidParameter;
  }
  const double n = v.norm();
  if (!(n > eps)) {
    log(LogLevel::Error, "ManipulatorModel: axis norm too small");
    return Status::InvalidParameter;
  }
  *a = v / n;
  return Status::Success;
}

Status ManipulatorModel::init(std::vector<JointSpec> joints,
                              const Transform& ee_home,
                              const Thresholds& thr) {
  joints_ = std::move(joints);
  ee_home_ = ee_home;

  // Build names + map
  joint_names_.clear();
  joint_names_.reserve(joints_.size());
  joint_name_id_map_.clear();

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].name.empty()) {
      joints_.clear();
      joint_names_.clear();
      joint_name_id_map_.clear();
      log(LogLevel::Error, "ManipulatorModel: joint name is empty");
      return Status::InvalidParameter;
    }
    joint_names_.push_back(joints_[i].name);
    joint_name_id_map_[joints_[i].name] = static_cast<unsigned int>(i);
  }

  const Status cache_st = rebuild_cache(thr);
  if (!ok(cache_st)) {
    joints_.clear();
    joint_names_.clear();
    joint_name_id_map_.clear();
    log(LogLevel::Error, "ManipulatorModel: rebuild_cache failed");
    return cache_st;
  }

  const Status val_st = validate(thr);
  if (!ok(val_st)) {
    joints_.clear();
    joint_names_.clear();
    joint_name_id_map_.clear();
    log(LogLevel::Error, "ManipulatorModel: validate failed");
    return val_st;
  }
  return Status::Success;
}

Status ManipulatorModel::set_joint_home_transforms(std::vector<Transform> gst0_i) {
  // Optional: accept n or n+1 (some pipelines include EE as last).
  if (!gst0_i.empty()) {
    const std::size_t n = joints_.size();
    if (gst0_i.size() != n && gst0_i.size() != n + 1) {
      log(LogLevel::Error, "ManipulatorModel: gst0_i size mismatch");
      return Status::InvalidParameter;
    }
  }
  gst0_i_ = std::move(gst0_i);
  return Status::Success;
}

Status ManipulatorModel::validate(const Thresholds& thr) const {
  // Basic checks
  if (!std::isfinite(ee_home_.translation().x()) ||
      !std::isfinite(ee_home_.translation().y()) ||
      !std::isfinite(ee_home_.translation().z())) {
    log(LogLevel::Error, "ManipulatorModel: ee_home translation is non-finite");
    return Status::InvalidParameter;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto& j = joints_[i];

    if (j.name.empty()) {
      log(LogLevel::Error, "ManipulatorModel: joint name is empty");
      return Status::InvalidParameter;
    }

    if (j.type == JointType::Revolute || j.type == JointType::Prismatic) {
      if (!isFiniteVec3(j.axis)) {
        log(LogLevel::Error, "ManipulatorModel: joint axis is non-finite");
        return Status::InvalidParameter;
      }
      if (!(j.axis.norm() > thr.axis_norm_eps)) {
        log(LogLevel::Error, "ManipulatorModel: joint axis norm too small");
        return Status::InvalidParameter;
      }
    }

    if (j.type == JointType::Revolute) {
      if (!isFiniteVec3(j.point)) {
        log(LogLevel::Error, "ManipulatorModel: joint point is non-finite");
        return Status::InvalidParameter;
      }
    }

    if (j.limit.enabled) {
      if (!std::isfinite(j.limit.lower) || !std::isfinite(j.limit.upper)) {
        log(LogLevel::Error, "ManipulatorModel: joint limits are non-finite");
        return Status::InvalidParameter;
      }
      if (j.limit.lower > j.limit.upper) {
        log(LogLevel::Error, "ManipulatorModel: joint lower > upper");
        return Status::InvalidParameter;
      }
    }
  }
  return Status::Success;
}

bool ManipulatorModel::within_limits(const Eigen::VectorXd& q, double tol) const {
  if (q.size() != dof()) return false;

  for (int i = 0; i < dof(); ++i) {
    const auto& lim = joints_[static_cast<std::size_t>(i)].limit;
    if (!lim.enabled) continue;

    const double qi = q(i);
    if (qi < lim.lower - tol) return false;
    if (qi > lim.upper + tol) return false;
  }
  return true;
}

Status ManipulatorModel::clamp_to_limits(const Eigen::VectorXd& q, Eigen::VectorXd* out) const {
  if (!out) {
    log(LogLevel::Error, "ManipulatorModel: clamp_to_limits null output");
    return Status::InvalidParameter;
  }
  if (q.size() != dof()) {
    log(LogLevel::Error, "ManipulatorModel: clamp_to_limits size mismatch");
    return Status::InvalidParameter;
  }

  *out = q;
  for (int i = 0; i < dof(); ++i) {
    const auto& lim = joints_[static_cast<std::size_t>(i)].limit;
    if (!lim.enabled) continue;

    (*out)(i) = std::min(std::max((*out)(i), lim.lower), lim.upper);
  }
  return Status::Success;
}

Status ManipulatorModel::rebuild_cache(const Thresholds& thr) {
  const int n = dof();
  S_space_.resize(6, n);
  S_space_.setZero();

  for (int i = 0; i < n; ++i) {
    auto& j = joints_[static_cast<std::size_t>(i)];

    if (j.type == JointType::Fixed) {
      S_space_.col(i).setZero();
      j.axis.setZero();
      j.point.setZero();
      continue;
    }

    // Normalize axis and store back (invariant)
    Status st = normalizeOrAssign(&j.axis, thr.axis_norm_eps);
    if (!ok(st)) return st;

    if (j.type == JointType::Revolute) {
      // S = [w; v], v = -w x q
      const Vec3 w = j.axis;
      const Vec3 q = j.point;
      const Vec3 v = -w.cross(q);

      ScrewAxis S;
      S.head<3>() = w;
      S.tail<3>() = v;
      S_space_.col(i) = S;
    } else if (j.type == JointType::Prismatic) {
      // S = [0; v]
      const Vec3 v = j.axis;

      ScrewAxis S;
      S.head<3>().setZero();
      S.tail<3>() = v;
      S_space_.col(i) = S;

      // prismatic doesn't use point
      j.point.setZero();
    }
  }
  return Status::Success;
}

// -------- Builder --------

ManipulatorBuilder& ManipulatorBuilder::add_revolute(std::string name,
                                                     const Vec3& w_unit,
                                                     const Vec3& q_point,
                                                     JointLimit limit,
                                                     Transform joint_tip_home) {
  JointSpec j;
  j.name = std::move(name);
  j.type = JointType::Revolute;
  j.axis = w_unit;
  j.point = q_point;
  j.limit = limit;
  j.joint_tip_home = joint_tip_home;
  joints_.push_back(std::move(j));
  return *this;
}

ManipulatorBuilder& ManipulatorBuilder::add_prismatic(std::string name,
                                                      const Vec3& v_unit,
                                                      JointLimit limit,
                                                      Transform joint_tip_home) {
  JointSpec j;
  j.name = std::move(name);
  j.type = JointType::Prismatic;
  j.axis = v_unit;
  j.point = Vec3::Zero();
  j.limit = limit;
  j.joint_tip_home = joint_tip_home;
  joints_.push_back(std::move(j));
  return *this;
}

ManipulatorBuilder& ManipulatorBuilder::add_fixed(std::string name,
                                                  Transform joint_tip_home) {
  JointSpec j;
  j.name = std::move(name);
  j.type = JointType::Fixed;
  j.axis = Vec3::Zero();
  j.point = Vec3::Zero();
  j.limit = JointLimit{};
  j.joint_tip_home = joint_tip_home;
  joints_.push_back(std::move(j));
  return *this;
}

Status ManipulatorBuilder::build(ManipulatorModel* out, const Thresholds& thr) const {
  if (!out) {
    log(LogLevel::Error, "ManipulatorBuilder: output model is null");
    return Status::InvalidParameter;
  }
  if (!ee_home_.has_value()) {
    log(LogLevel::Error, "ManipulatorBuilder: ee_home not set");
    return Status::InvalidParameter;
  }
  return out->init(joints_, *ee_home_, thr);
}

}  // namespace sclerp::core
