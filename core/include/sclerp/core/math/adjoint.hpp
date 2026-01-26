#pragma once
#include "sclerp/core/math/types.hpp"

namespace sclerp::core {

// Adjoint of SE(3)
AdjointMatrix adjoint(const Transform& T);
Eigen::Matrix<double, 6, 6> getAdjoint(const Mat4& g);  // kinlib-like utility

}  // namespace sclerp::core
