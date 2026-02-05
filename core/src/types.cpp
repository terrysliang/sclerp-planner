#include "sclerp/core/math/types.hpp"

#include "sclerp/core/common/logger.hpp"

namespace sclerp::core {

Transform transformFromMatrix4(const Mat4& T) {
  // Expect a valid homogeneous transform. We tolerate small numeric drift.
  if (std::abs(T(3, 3) - 1.0) > 1e-9) {
    log(LogLevel::Warn, "transformFromMatrix4: non-homogeneous T(3,3); continuing");
  }

  Transform out = Transform::Identity();
  out.linear() = T.block<3, 3>(0, 0);
  out.translation() = T.block<3, 1>(0, 3);
  return out;
}

Mat4 matrix4FromTransform(const Transform& T) {
  Mat4 out = Mat4::Identity();
  out.block<3, 3>(0, 0) = T.rotation();
  out.block<3, 1>(0, 3) = T.translation();
  return out;
}

}  // namespace sclerp::core
