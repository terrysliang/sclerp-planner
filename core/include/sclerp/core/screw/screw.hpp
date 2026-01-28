#pragma once
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/common/constants.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/dual_quat/dual_quat.hpp"

#include <vector>

namespace sclerp::core {

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

// Nearest pose on the constant screw from g_i to g_f to a target pose g_t.
// Returns tau in [0,1] and the distances.
struct NearestOnScrewResult {
  Status status{Status::Failure};
  double tau01{0.0};
  double d_pos{0.0};
  double d_rot{0.0};
  Transform g_near{Transform::Identity()};
};

NearestOnScrewResult nearestPoseOnScrew(const Transform& g_i,
                                        const Transform& g_f,
                                        const Transform& g_t,
                                        const Thresholds& thr = kDefaultThresholds);

// Important: segments a POSE SEQUENCE and returns END INDICES.
Status screwSegments(const std::vector<Transform>& g_seq,
                     std::vector<unsigned int>* seg_end_indices,
                     double max_pos_d = 0.015,
                     double max_rot_d = 0.15);

// Convenience overload to accept Matrix4d
Status screwSegments(const std::vector<Mat4>& g_seq,
                     std::vector<unsigned int>* seg_end_indices,
                     double max_pos_d = 0.015,
                     double max_rot_d = 0.15);

}  // namespace sclerp::core
