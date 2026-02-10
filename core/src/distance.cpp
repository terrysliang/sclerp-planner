// Simple task-space distance helpers used by planners.
// - Position distance is Euclidean norm in meters.
// - Rotation distance delegates to SO(3) helpers and returns chordal quaternion distance.
#include "sclerp/core/math/distance.hpp"
#include "sclerp/core/math/so3.hpp"
#include "sclerp/core/math/types.hpp"

namespace sclerp::core {

double positionDistance(const Transform& T1, const Transform& T2) {
  return (T1.translation() - T2.translation()).norm();
}

double positionDistance(const Mat4& t1, const Mat4& t2) {
  return positionDistance(transformFromMatrix4(t1), transformFromMatrix4(t2));
}

double positionDistance(const Eigen::VectorXd& p1, const Eigen::VectorXd& p2) {
  return (p1 - p2).norm();
}

double rotationDistance(const Transform& T1, const Transform& T2) {
  const Mat3 R1 = T1.rotation();
  const Mat3 R2 = T2.rotation();
  return rotationDistance(R1, R2); 
}

}  // namespace sclerp::core
