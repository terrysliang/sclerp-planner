#pragma once
#include "lie.hpp"
#include "constraints.hpp"

namespace sclerp::core {

// Full ScLERP: T(t) = T0 * exp( t * log(T0^{-1} T1) )
SE3d sclerp(const SE3d& T0, const SE3d& T1, double t);

// Constraint-aware interpolation (useful baseline for subgroup motions)
SE3d interpolate(const SE3d& T0, const SE3d& T1, const Constraint& c, double t);

}  // namespace sclerp::core
