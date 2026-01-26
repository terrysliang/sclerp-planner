#pragma once

namespace sclerp::core {

// Thresholds exist in kinlib_kinematics.h as macros; move to a typed config.
struct Thresholds {
  double pure_translation_rot_angle = 1.0e-5;  // rad
  double pure_rotation_pitch = 1.0e-5;         // m/rad (pitch)
  double no_motion_magnitude = 1.0e-5;         // unit depends on measure

  // general numerical
  double axis_norm_eps = 1.0e-12;
  double svd_tol = 1.0e-6;
};

inline constexpr Thresholds kDefaultThresholds{};

}  // namespace sclerp::core
