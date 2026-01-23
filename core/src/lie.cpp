#include "sclerp/core/lie.hpp"
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

Quat expSO3(const Vec3& w) {
  const double theta = w.norm();
  if (theta < 1e-12) {
    // Small angle: q ~ [1, 0.5*w]
    return Quat(1.0, 0.5*w.x(), 0.5*w.y(), 0.5*w.z()).normalized();
  }
  const Vec3 axis = w / theta;
  return Quat(Eigen::AngleAxisd(theta, axis));
}

Vec3 logSO3(const Mat3& R) {
  const double cos_theta = clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double theta = std::acos(cos_theta);

  if (theta < 1e-12) {
    return Vec3::Zero();
  }

  // w_hat = (theta/(2*sin(theta))) * (R - R^T)
  const Mat3 W = (theta / (2.0 * std::sin(theta))) * (R - R.transpose());
  return vee3(W);
}

SE3d expSE3(const Twist6d& xi) {
  const Vec3 w = xi.omega;
  const Vec3 v = xi.v;

  const double theta = w.norm();
  Mat3 R = Mat3::Identity();
  Mat3 V = Mat3::Identity();

  if (theta < 1e-12) {
    // R ~ I + W, V ~ I + 0.5 W + 1/6 W^2
    const Mat3 W = hat3(w);
    const Mat3 W2 = W * W;
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

  const Vec3 p = V * v;
  return SE3d(Quat(R), p);
}

Twist6d logSE3(const SE3d& T) {
  Twist6d xi;
  const Mat3 R = T.R();
  const Vec3 p = T.p;

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

  xi.omega = w;
  xi.v = V.inverse() * p;
  return xi;
}

}  // namespace sclerp::core
