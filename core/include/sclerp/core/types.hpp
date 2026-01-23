#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sclerp::core {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

struct SE3d {
  Quat q{Quat::Identity()};  // rotation
  Vec3 p{Vec3::Zero()};      // translation

  SE3d() = default;
  SE3d(const Quat& q_in, const Vec3& p_in) : q(q_in), p(p_in) {}

  static SE3d Identity() { return SE3d(); }

  Mat3 R() const { return q.toRotationMatrix(); }

  SE3d inverse() const {
    Quat qi = q.conjugate();
    return SE3d(qi, -(qi * p));
  }

  SE3d operator*(const SE3d& other) const {
    return SE3d(q * other.q, p + q * other.p);
  }
};

struct Twist6d {
  Vec3 omega{Vec3::Zero()};  // rotation part
  Vec3 v{Vec3::Zero()};      // translation part
};

}  // namespace sclerp::core
