#include "sclerp/core/dual_quat/dual_quat.hpp"

#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/math/se3.hpp"
#include "sclerp/core/math/so3.hpp"
#include "sclerp/core/math/types.hpp"

#include <algorithm>
#include <cmath>

namespace sclerp::core {

static inline bool quatNormalizeSafe(const Quat& q, Quat* out) {
  if (!out) return false;
  const double n = q.norm();
  if (n < 1e-15) {
    return false;
  }
  *out = q;
  out->coeffs() /= n;
  return true;
}

static inline bool nearUnit(double n, double tol = 1e-12) {
  return std::abs(n - 1.0) <= tol;
}

static inline Quat quatMul(const Quat& a, const Quat& b) {
  return a * b;
}

static inline Eigen::Vector4d quatVec(const Quat& q) {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

static inline Quat quatFromVec(const Eigen::Vector4d& v) {
  return Quat(v(0), v(1), v(2), v(3));
}

static inline Eigen::Vector4d quatProduct(const Eigen::Vector4d& q1,
                                          const Eigen::Vector4d& q2) {
  Eigen::Vector4d out;
  out(0) = (q1(0) * q2(0)) - q1.tail<3>().dot(q2.tail<3>());
  out.tail<3>() = (q1(0) * q2.tail<3>()) + (q2(0) * q1.tail<3>()) +
                  q1.tail<3>().cross(q2.tail<3>());
  return out;
}

DualQuat DualQuat::productRaw(const DualQuat& a, const DualQuat& b) {
  const Eigen::Vector4d r1 = quatVec(a.real());
  const Eigen::Vector4d d1 = quatVec(a.dual());
  const Eigen::Vector4d r2 = quatVec(b.real());
  const Eigen::Vector4d d2 = quatVec(b.dual());

  const Eigen::Vector4d r = quatProduct(r1, r2);
  const Eigen::Vector4d d = quatProduct(d1, r2) + quatProduct(r1, d2);

  DualQuat out;
  out.qr_ = quatFromVec(r);
  out.qd_ = quatFromVec(d);
  return out;
}

static inline Quat quatConj(const Quat& q) {
  return q.conjugate();
}

static inline Quat quatFromTranslation(const Vec3& t) {
  return Quat(0.0, t.x(), t.y(), t.z());
}

DualQuat::DualQuat() = default;

DualQuat::DualQuat(const Transform& T) {
  Quat qr;
  if (!quatNormalizeSafe(quatFromSO3(T.rotation()), &qr)) {
    log(LogLevel::Error, "DualQuat: zero-norm quaternion in constructor");
    qr_ = Quat::Identity();
    qd_ = Quat(0.0, 0.0, 0.0, 0.0);
    return;
  }
  qr_ = qr;

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
  const double n = qr_.norm();
  if (n < 1e-15) {
    log(LogLevel::Error, "DualQuat::normalized: zero-norm real part");
    return DualQuat();
  }
  if (nearUnit(n) && qr_.w() >= 0.0) {
    return *this;
  }

  DualQuat out = *this;
  if (!nearUnit(n)) {
    out.qr_.coeffs() /= n;
    out.qd_.coeffs() /= n;
  }

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
  Quat qr = qr_;
  Quat qd = qd_;
  const double n = qr.norm();
  if (n < 1e-15) {
    log(LogLevel::Error, "DualQuat::toTransform: zero-norm real part");
    return Transform::Identity();
  }
  if (!nearUnit(n)) {
    qr.coeffs() /= n;
    qd.coeffs() /= n;
  }
  if (qr.w() < 0.0) {
    qr.coeffs() *= -1.0;
    qd.coeffs() *= -1.0;
  }

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
  Quat qr = qr_;
  const double n = qr.norm();
  if (n < 1e-15) {
    log(LogLevel::Error, "DualQuat::rotation: zero-norm real part");
    return Quat::Identity();
  }
  if (!nearUnit(n)) {
    qr.coeffs() /= n;
  }
  if (qr.w() < 0.0) {
    qr.coeffs() *= -1.0;
  }
  return qr;
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
  // dual-quat power (raiseToPower).
  const DualQuat dq = this->normalized();
  Eigen::Vector4d real_part = quatVec(dq.qr_);
  Eigen::Vector4d dual_part = quatVec(dq.qd_);

  double theta;
  const double w = std::max(-1.0, std::min(1.0, real_part(0)));
  if (w <= -1.0) {
    theta = M_PI;
  } else if (w >= 1.0) {
    theta = 0.0;
  } else {
    theta = 2.0 * std::acos(w);
  }

  theta = std::fmod(theta + M_PI, 2.0 * M_PI);
  if (theta < 0.0) {
    theta += 2.0 * M_PI;
  }
  theta -= M_PI;

  const Transform T = dq.toTransform();
  const Vec3 p = T.translation();

  Eigen::Vector4d res_real_part;
  Eigen::Vector4d res_dual_part;

  if (std::abs(theta) < 1e-6) {
    Eigen::Vector3d v = dual_part.tail<3>();
    const double d = 2.0 * v.norm();

    if (d < 1e-12) {
      res_real_part << 1.0, 0.0, 0.0, 0.0;
      res_dual_part << 0.0, 0.0, 0.0, 0.0;
    } else {
      v = v / (d / 2.0);

      res_real_part(0) = std::cos(t * (theta / 2.0));
      res_real_part.tail<3>() = v * std::sin(t * (theta / 2.0));

      res_dual_part(0) = -(t * d / 2.0) * std::sin(t * (theta / 2.0));
      res_dual_part.tail<3>() =
          (t * d / 2.0) * std::cos(t * (theta / 2.0)) * v;
    }
  } else {
    Eigen::Vector3d rp = real_part.tail<3>();
    Eigen::Vector3d u = rp / rp.norm();

    const double d = p.dot(u);
    const Eigen::Vector3d m =
        (p.cross(u) + ((p - (d * u)) / std::tan(theta / 2.0))) / 2.0;

    res_real_part(0) = std::cos(t * (theta / 2.0));
    res_real_part.tail<3>() = std::sin(t * (theta / 2.0)) * u;

    res_dual_part(0) = -(t * d / 2.0) * std::sin(t * (theta / 2.0));
    res_dual_part.tail<3>() =
        (std::sin(t * (theta / 2.0)) * m) +
        (((t * d * std::cos(t * theta / 2.0)) / 2.0) * u);
  }

  DualQuat out;
  out.qr_ = quatFromVec(res_real_part);
  out.qd_ = quatFromVec(res_dual_part);
  return out;
}

DualQuat DualQuat::interpolate(const DualQuat& a, const DualQuat& b, double t) {
  // dual-quat interpolation: dq_i * (dq_i*^{-1} dq_f)^t
  const DualQuat a_norm = a.normalized();
  const DualQuat b_norm = b.normalized();
  const DualQuat dq_i_conj = a_norm.conjugate();
  DualQuat dq_rel = DualQuat::productRaw(dq_i_conj, b_norm);
  dq_rel = dq_rel.pow(t);
  return DualQuat::productRaw(a_norm, dq_rel);
}

double rotationDistance(const DualQuat& a, const DualQuat& b) {
  return sclerp::core::rotationDistance(a.rotation(), b.rotation());
}

}  // namespace sclerp::core
