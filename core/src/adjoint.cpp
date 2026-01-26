#include "sclerp/core/math/adjoint.hpp"
#include "sclerp/core/math/se3.hpp"

namespace sclerp::core {

AdjointMatrix adjoint(const Transform& T) {
  const Mat3 R = T.rotation();
  const Vec3 p = T.translation();

  AdjointMatrix Ad = AdjointMatrix::Zero();
  Ad.block<3,3>(0,0) = R;
  Ad.block<3,3>(3,3) = R;
  Ad.block<3,3>(3,0) = hat3(p) * R;  // [p]x R
  return Ad;
}

Eigen::Matrix<double, 6, 6> getAdjoint(const Mat4& g) {
  return adjoint(transformFromMatrix4(g));
}

}  // namespace sclerp::core

