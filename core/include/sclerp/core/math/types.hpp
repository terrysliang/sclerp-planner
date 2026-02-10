#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sclerp::core {

// Fundamental math types and conventions used across the library.
// - `Transform` represents an SE(3) rigid transform (rotation + translation) and is composed by
//   left-multiplication (A * B applies B, then A).
// - `Twist` / `ScrewAxis` uses ordering [v; w] (linear; angular) in the world frame.
// - Units are meters and radians.
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Quat = Eigen::Quaterniond;

using Transform = Eigen::Isometry3d;                 // internal canonical SE(3)
using Twist = Eigen::Matrix<double, 6, 1>;           // [v; w] (linear; angular)
using AdjointMatrix = Eigen::Matrix<double, 6, 6>;
using ScrewAxis = Twist;
using ScrewMatrix = Eigen::Matrix<double, 6, Eigen::Dynamic>;

Transform transformFromMatrix4(const Mat4& T);
Mat4 matrix4FromTransform(const Transform& T);

// Convenience accessors
inline Vec3 translation(const Transform& T) { return T.translation(); }
inline Mat3 rotation(const Transform& T) { return T.rotation(); }

}  // namespace sclerp::core
