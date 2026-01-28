#pragma once
#include "sclerp/core/math/so3.hpp"

namespace sclerp::core {

// se(3)
Mat4 hat6(const Twist& xi);
Twist vee6(const Mat4& Xi);

// SE(3)
Transform expSE3(const Twist& xi);
Twist logSE3(const Transform& T);

Transform invSE3(const Transform& T);

// utilities
Mat4 getTransformationInv(const Mat4& g);

}  // namespace sclerp::core
