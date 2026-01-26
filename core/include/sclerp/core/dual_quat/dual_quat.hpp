#pragma once
#include "sclerp/core/math/types.hpp"

namespace sclerp::core {

// Refactored dual quaternion (const-correct, Transform-based).
// Keeps the capability of DualQuat.h: from Transform/Matrix4, interpolate, pow, etc.
class DualQuat {
public:
  DualQuat();                             // identity
  explicit DualQuat(const Transform& T);
  explicit DualQuat(const Mat4& T);

  static DualQuat identity();

  // Accessors
  const Quat& real() const { return qr_; }
  const Quat& dual() const { return qd_; }

  DualQuat normalized() const;
  DualQuat conjugate() const;

  Transform toTransform() const;
  Mat4 toMatrix4() const;

  Quat rotation() const;                  // rotation quaternion
  Vec3 translation() const;               // translation vector

  DualQuat operator*(const DualQuat& rhs) const;

  // Dual-quat screw interpolation primitive
  DualQuat pow(double t) const;

  static DualQuat interpolate(const DualQuat& a, const DualQuat& b, double t);

private:
  // dq = qr + eps * qd
  Quat qr_{Quat::Identity()};
  Quat qd_{Quat(0.0, 0.0, 0.0, 0.0)};
};

double rotationDistance(const DualQuat& a, const DualQuat& b);

}  // namespace sclerp::core
