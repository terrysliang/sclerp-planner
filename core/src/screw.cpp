#include "sclerp/core/screw/screw.hpp"

#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/math/se3.hpp"
#include "sclerp/core/math/so3.hpp"
#include "sclerp/core/math/types.hpp"

#include <algorithm>
#include <cmath>

namespace sclerp::core {

static inline double clamp01(double t) {
  return std::max(0.0, std::min(1.0, t));
}

Status screwParameters(const Transform& g_i,
                       const Transform& g_f,
                       ScrewParameters* out,
                       const Thresholds& thr) {
  if (!out) {
    log(LogLevel::Error, "screwParameters: null output");
    return Status::InvalidParameter;
  }

  const Transform g_rel = g_i.inverse() * g_f;
  out->g_rel = g_rel;

  const Mat3 R = g_rel.rotation();
  const Vec3 p = g_rel.translation();

  // Rotation vector (axis * angle)
  const Vec3 w = logSO3(R);
  const double theta = w.norm();

  // Pure translation / no motion handling.
  if (theta < thr.pure_translation_rot_angle) {
    const double p_norm = p.norm();
    if (p_norm < thr.no_motion_magnitude) {
      out->type = ScrewMotionType::NoMotion;
      out->omega.setZero();
      out->theta = 0.0;
      out->pitch = 0.0;
      out->l.setZero();
      return Status::Success;
    }

    out->type = ScrewMotionType::PureTranslation;
    if (p_norm > thr.axis_norm_eps) {
      out->omega = p / p_norm;
    } else {
      out->omega.setZero();
    }
    out->theta = p_norm;
    out->pitch = 0.0;
    out->l.setZero();
    return Status::Success;
  }

  // General screw or pure rotation: use SE(3) log for consistent decomposition
  const Twist xi = logSE3(g_rel);
  const Vec3 v_se3 = xi.head<3>();
  const Vec3 w_se3 = xi.tail<3>();

  const double th = w_se3.norm();
  if (th < thr.pure_translation_rot_angle) {
    // Should not happen (we already handled), but be safe:
    out->type = ScrewMotionType::PureTranslation;
    out->omega.setZero();
    out->theta = 0.0;
    out->pitch = 0.0;
    out->l = (p.norm() > thr.axis_norm_eps) ? (p.normalized()) : Vec3::Zero();
    return Status::Success;
  }

  const Vec3 omega = w_se3 / th;

  // Important: in our exp/log convention, v_se3 is scaled consistently with w_se3
  // (i.e., both correspond to "twist * theta"), so pitch is:
  const double pitch = omega.dot(v_se3) / th;

  out->omega = omega;
  out->theta = th;
  out->pitch = pitch;

  // use the linear part per rad: v/theta = -omega×q + pitch*omega
  out->l = v_se3 / th;

  // Classify pure rotation vs general screw
  if (std::abs(pitch) < thr.pure_rotation_pitch) {
    out->type = ScrewMotionType::PureRotation;
  } else {
    out->type = ScrewMotionType::GeneralScrew;
  }

  return Status::Success;
}

Transform sclerp(const Transform& g_i, const Transform& g_f, double t) {
  const double tau = clamp01(t);
  const Transform g_rel = g_i.inverse() * g_f;
  const Twist xi = logSE3(g_rel);
  return g_i * expSE3(xi * tau);
}

}  // namespace sclerp::core
