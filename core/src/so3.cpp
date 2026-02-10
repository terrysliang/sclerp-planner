// SO(3) utilities.
// - `expSO3` uses Rodrigues' formula with a small-angle series fallback.
// - `logSO3` is numerically guarded near pi to avoid axis/angle instability.
// - `rotationDistance(q1,q2)` is the quaternion chordal distance:
//     min(||q1 - q2||, ||q1 + q2||)
//   (useful for convergence checks; not the rotation angle in radians).
#include "sclerp/core/math/so3.hpp"
#include <algorithm>
#include <cmath>

namespace sclerp::core {

static inline double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

Mat3 hat3(const Vec3& w) {
  Mat3 W;
  W <<     0.0, -w.z(),  w.y(),
        w.z(),     0.0, -w.x(),
       -w.y(),  w.x(),     0.0;
  return W;
}

Vec3 vee3(const Mat3& W) {
  return Vec3(W(2,1), W(0,2), W(1,0));
}

Mat3 expSO3(const Vec3& w) {
  const double theta = w.norm();
  const Mat3 I = Mat3::Identity();

  if (theta < 1e-12) {
    const Mat3 W = hat3(w);
    const Mat3 W2 = W * W;
    // Rodrigues small-angle series
    return I + W + 0.5 * W2;
  }

  const Vec3 a = w / theta;
  const Mat3 A = hat3(a);
  return I + std::sin(theta) * A + (1.0 - std::cos(theta)) * (A * A);
}

Vec3 logSO3(const Mat3& R) {
  const double cos_theta = clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double theta = std::acos(cos_theta);

  if (theta < 1e-12) {
    return Vec3::Zero();
  }

  // Near pi: standard formula becomes ill-conditioned; use robust extraction.
  if (M_PI - theta < 1e-6) {
    Vec3 axis;
    axis.x() = std::sqrt(std::max(0.0, (R(0,0) + 1.0) * 0.5));
    axis.y() = std::sqrt(std::max(0.0, (R(1,1) + 1.0) * 0.5));
    axis.z() = std::sqrt(std::max(0.0, (R(2,2) + 1.0) * 0.5));

    // Fix signs using off-diagonals
    if (R(2,1) - R(1,2) < 0) axis.x() = -axis.x();
    if (R(0,2) - R(2,0) < 0) axis.y() = -axis.y();
    if (R(1,0) - R(0,1) < 0) axis.z() = -axis.z();

    if (axis.norm() < 1e-12) {
      // Fallback
      axis = Vec3::UnitX();
    } else {
      axis.normalize();
    }
    return axis * theta;
  }

  const Mat3 W = (theta / (2.0 * std::sin(theta))) * (R - R.transpose());
  return vee3(W);
}

Quat quatFromSO3(const Mat3& R) {
  Quat q(R);
  q.normalize();
  return q;
}

Mat3 so3FromQuat(const Quat& q) {
  return q.normalized().toRotationMatrix();
}

Quat slerp(const Quat& q0, const Quat& q1, double t) {
  Quat a = q0.normalized();
  Quat b = q1.normalized();
  // Ensure shortest path
  if (a.dot(b) < 0.0) b.coeffs() *= -1.0;
  Quat out = a.slerp(t, b);
  out.normalize();
  return out;
}

double rotationDistance(const Mat3& R1, const Mat3& R2) {
  const Quat q1 = quatFromSO3(R1);
  const Quat q2 = quatFromSO3(R2);
  return rotationDistance(q1, q2);
}

double rotationDistance(const Quat& q1, const Quat& q2) {
  const Quat a = q1.normalized();
  const Quat b = q2.normalized();

  const Eigen::Vector4d v1 = a.coeffs();
  const Eigen::Vector4d v2 = b.coeffs();
  const double d1 = (v1 - v2).norm();
  const double d2 = (v1 + v2).norm();
  return (d1 > d2) ? d2 : d1;
}

}  // namespace sclerp::core
