#pragma once
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/common/constants.hpp"
#include "sclerp/core/math/types.hpp"

namespace sclerp::core {

// SE(3) screw-motion helpers.
// - `screwParameters` decomposes relative motion g_rel = g_i^{-1} g_f and classifies it under
//   thresholds (no motion / pure translation / pure rotation / general screw).
// - `sclerp` is the canonical SE(3) ScLERP curve: g(t) = g_i * exp(t * log(g_i^{-1} g_f)).
enum class ScrewMotionType : std::uint8_t {
  NoMotion = 0,
  GeneralScrew = 1,
  PureRotation = 2,
  PureTranslation = 3
};

// Refactored output form of getScrewParameters
struct ScrewParameters {
  ScrewMotionType type{ScrewMotionType::NoMotion};

  Vec3 omega = Vec3::Zero();   // unit axis if applicable
  double theta = 0.0;          // rotation magnitude (rad)
  double pitch = 0.0;          // h (m/rad), 0 for pure rotation
  Vec3 l = Vec3::Zero();       // direction vector

  // relative motion (optional convenience)
  Transform g_rel = Transform::Identity();
};

Status screwParameters(const Transform& g_i,
                       const Transform& g_f,
                       ScrewParameters* out,
                       const Thresholds& thr = kDefaultThresholds);

// SE(3) ScLERP: g(t) = g_i * exp(t * log(g_i^-1 g_f))
Transform sclerp(const Transform& g_i, const Transform& g_f, double t);

}  // namespace sclerp::core
