#pragma once
#include "types.hpp"

namespace sclerp::core {

// so(3) helpers
Mat3 hat3(const Vec3& w);
Vec3 vee3(const Mat3& W);

Quat expSO3(const Vec3& w);
Vec3 logSO3(const Mat3& R);

// SE(3) exp/log using closed form (V matrix)
SE3d expSE3(const Twist6d& xi);
Twist6d logSE3(const SE3d& T);

}  // namespace sclerp::core
