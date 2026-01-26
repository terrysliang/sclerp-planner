#include "sclerp/core/math/se3.hpp"
#include "sclerp/core/math/types.hpp"
#include <algorithm>
#include <cmath>

namespace sclerp::core {

static inline double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

Mat4 hat6(const Twist& xi) {
  const Vec3 w = xi.head<3>();
  const Vec3 v = xi.tail<3>();

  Mat4 X = Mat4::Zero();
  X.block<3,3>(0,0) = hat3(w);
  X.block<3,1>(0,3) = v;
  return X;
}

Twist vee6(const Mat4& Xi) {
  Twist xi;
  xi.head<3>() = vee3(Xi.block<3,3>(0,0));
  xi.tail<3>() = Xi.block<3,1>(0,3);
  return xi;
}

Transform expSE3(const Twist& xi) {
  const Vec3 w = xi.head<3>();
  const Vec3 v = xi.tail<3>();
  const double theta = w.norm();

  Mat3 R = Mat3::Identity();
  Mat3 V = Mat3::Identity();

  if (theta < 1e-12) {
    const Mat3 W = hat3(w);
    const Mat3 W2 = W * W;
    // Small-angle series
    R = Mat3::Identity() + W + 0.5 * W2;
    V = Mat3::Identity() + 0.5 * W + (1.0/6.0) * W2;
  } else {
    const Mat3 W = hat3(w);
    const Mat3 W2 = W * W;

    const double A = std::sin(theta) / theta;
    const double B = (1.0 - std::cos(theta)) / (theta * theta);
    const double C = (theta - std::sin(theta)) / (theta * theta * theta);

    R = Mat3::Identity() + A * W + B * W2;
    V = Mat3::Identity() + B * W + C * W2;
  }

  Transform T = Transform::Identity();
  T.linear() = R;
  T.translation() = V * v;
  return T;
}

Twist logSE3(const Transform& T) {
  const Mat3 R = T.rotation();
  const Vec3 p = T.translation();

  Twist xi;
  const Vec3 w = logSO3(R);
  const double theta = w.norm();

  Mat3 V = Mat3::Identity();

  if (theta < 1e-12) {
    const Mat3 W = hat3(w);
    const Mat3 W2 = W * W;
    V = Mat3::Identity() + 0.5 * W + (1.0/6.0) * W2;
  } else {
    const Mat3 W = hat3(w);
    const Mat3 W2 = W * W;
    const double B = (1.0 - std::cos(theta)) / (theta * theta);
    const double C = (theta - std::sin(theta)) / (theta * theta * theta);
    V = Mat3::Identity() + B * W + C * W2;
  }

  xi.head<3>() = w;
  xi.tail<3>() = V.inverse() * p;
  return xi;
}

Transform invSE3(const Transform& T) {
  return T.inverse();
}

Mat4 getTransformationInv(const Mat4& g) {
  const Mat3 R = g.block<3,3>(0,0);
  const Vec3 p = g.block<3,1>(0,3);

  Mat4 out = Mat4::Identity();
  out.block<3,3>(0,0) = R.transpose();
  out.block<3,1>(0,3) = -R.transpose() * p;
  return out;
}

}  // namespace sclerp::core

