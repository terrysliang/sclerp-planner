#include "sclerp/core/dual_quat/dual_quat.hpp"

#include "sclerp/core/math/se3.hpp"
#include "sclerp/core/math/so3.hpp"
#include "sclerp/core/math/types.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sclerp::core {

static inline Quat quatNormalizeSafe(const Quat& q) {
  Quat out = q;
  const double n = out.norm();
  if (n < 1e-15) {
    throw std::runtime_error("DualQuat: zero-norm quaternion");
  }
  out.coeffs() /= n;
  return out;
}

static inline Quat quatMul(const Quat& a, const Quat& b) {
  return a * b;
}

static inline Quat quatConj(const Quat& q) {
  return q.conjugate();
}

static inline Quat quatFromTranslation(const Vec3& t) {
  return Quat(0.0, t.x(), t.y(), t.z());
}

DualQuat::DualQuat() = default;

DualQuat::DualQuat(const Transform& T) {
  qr_ = quatFromSO3(T.rotation());
  qr_ = quatNormalizeSafe(qr_);

  const Vec3 t = T.translation();
  // qd = 0.5 * (t_quat * qr)
  qd_ = quatMul(quatFromTranslation(t), qr_);
  qd_.coeffs() *= 0.5;
}

DualQuat::DualQuat(const Mat4& T)
  : DualQuat(transformFromMatrix4(T)) {}

DualQuat DualQuat::identity() { return DualQuat(); }

DualQuat DualQuat::normalized() const {
  // For rigid transforms, normalize qr to unit and scale qd accordingly.
  DualQuat out = *this;
  const double n = out.qr_.norm();
  if (n < 1e-15) {
    throw std::runtime_error("DualQuat::normalized: zero-norm real part");
  }
  out.qr_.coeffs() /= n;
  out.qd_.coeffs() /= n;

  // fix sign for canonicalization (optional but helps consistency)
  if (out.qr_.w() < 0.0) {
    out.qr_.coeffs() *= -1.0;
    out.qd_.coeffs() *= -1.0;
  }
  return out;
}

DualQuat DualQuat::conjugate() const {
  DualQuat out;
  out.qr_ = quatConj(qr_);
  out.qd_ = quatConj(qd_);
  return out;
}

Transform DualQuat::toTransform() const {
  const DualQuat dq = this->normalized();
  const Quat qr = dq.qr_;
  const Quat qd = dq.qd_;

  const Quat qr_conj = quatConj(qr);
  // t_quat = 2 * (qd * qr*)
  Quat t_quat = quatMul(qd, qr_conj);
  t_quat.coeffs() *= 2.0;

  Vec3 t(t_quat.x(), t_quat.y(), t_quat.z());
  Mat3 R = so3FromQuat(qr);

  Transform T = Transform::Identity();
  T.linear() = R;
  T.translation() = t;
  return T;
}

Mat4 DualQuat::toMatrix4() const {
  return matrix4FromTransform(toTransform());
}

Quat DualQuat::rotation() const {
  return this->normalized().qr_;
}

Vec3 DualQuat::translation() const {
  return this->toTransform().translation();
}

DualQuat DualQuat::operator*(const DualQuat& rhs) const {
  // (qr1 + eps qd1) (qr2 + eps qd2) = qr1*qr2 + eps(qr1*qd2 + qd1*qr2)
  DualQuat a = this->normalized();
  DualQuat b = rhs.normalized();

  DualQuat out;
  out.qr_ = quatMul(a.qr_, b.qr_);

  const Quat t1 = quatMul(a.qr_, b.qd_);
  const Quat t2 = quatMul(a.qd_, b.qr_);
  out.qd_.coeffs() = t1.coeffs() + t2.coeffs();

  return out.normalized();
}

DualQuat DualQuat::pow(double t) const {
  // Define power via SE(3) log/exp: T^t = exp(t * log(T))
  const Transform T = this->toTransform();
  const Twist xi = logSE3(T);
  const Transform Tt = expSE3(xi * t);
  return DualQuat(Tt);
}

DualQuat DualQuat::interpolate(const DualQuat& a, const DualQuat& b, double t) {
  // Robust: interpolate in SE(3) using exp/log (ScLERP in SE3)
  const Transform Ta = a.toTransform();
  const Transform Tb = b.toTransform();
  const Transform Trel = Ta.inverse() * Tb;
  const Twist xi = logSE3(Trel);
  const Transform Tstep = expSE3(xi * t);
  return DualQuat(Ta * Tstep);
}

double rotationDistance(const DualQuat& a, const DualQuat& b) {
  return sclerp::core::rotationDistance(a.rotation(), b.rotation());
}

}  // namespace sclerp::core
