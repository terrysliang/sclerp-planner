#pragma once
#include "sclerp/core/math/types.hpp"

namespace sclerp::core {

// so(3)
Mat3 hat3(const Vec3& w);
Vec3 vee3(const Mat3& W);

// SO(3)
Mat3 expSO3(const Vec3& w);
Vec3 logSO3(const Mat3& R);

Quat quatFromSO3(const Mat3& R);
Mat3 so3FromQuat(const Quat& q);

Quat slerp(const Quat& q0, const Quat& q1, double t);

// Rotation distance helpers
double rotationDistance(const Mat3& R1, const Mat3& R2);
double rotationDistance(const Quat& q1, const Quat& q2);

}  // namespace sclerp::core
