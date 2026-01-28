#pragma once
#include "sclerp/core/math/types.hpp"

namespace sclerp::core {

double positionDistance(const Transform& T1, const Transform& T2);
double positionDistance(const Mat4& t1, const Mat4& t2);
double positionDistance(const Eigen::VectorXd& p1, const Eigen::VectorXd& p2);

double rotationDistance(const Transform& T1, const Transform& T2);

}  // namespace sclerp::core
