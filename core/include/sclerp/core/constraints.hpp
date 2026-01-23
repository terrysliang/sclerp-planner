#pragma once
#include "types.hpp"
#include <cstdint>

namespace sclerp::core {

enum class ConstraintType : std::uint8_t {
  SE3,    // full 6D
  R3,     // translation only
  SO3,    // rotation only
  SE2,    // x,y,yaw (z fixed; roll/pitch fixed)
  Custom  // mask-based (future)
};

struct Constraint {
  ConstraintType type{ConstraintType::SE3};

  // For SE2: define the "up" axis as +Z of world frame (convention).
  // For Custom: fill later with masks/frames if needed.
};

}  // namespace sclerp::core
