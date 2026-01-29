#include "sclerp/core/screw/screw.hpp"

#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/math/distance.hpp"
#include "sclerp/core/math/se3.hpp"
#include "sclerp/core/math/so3.hpp"
#include "sclerp/core/math/types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sclerp::core {

static inline double clamp01(double t) {
  return std::max(0.0, std::min(1.0, t));
}

static inline double costPose(const Transform& a, const Transform& b) {
  // Simple combined cost (m + rad). You can tune later.
  const double dp = positionDistance(a, b);
  const double dr = rotationDistance(a, b);
  return dp + dr;
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
  const Vec3 w_se3 = xi.head<3>();
  const Vec3 v_se3 = xi.tail<3>();

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

NearestOnScrewResult nearestPoseOnScrew(const Transform& g_i,
                                        const Transform& g_f,
                                        const Transform& g_t,
                                        const Thresholds& thr) {
  NearestOnScrewResult res;

  // Handle degenerate / no-motion cases
  ScrewParameters sp;
  const Status st = screwParameters(g_i, g_f, &sp, thr);
  if (!ok(st)) {
    res.status = st;
    return res;
  }

  if (sp.type == ScrewMotionType::NoMotion) {
    res.status = Status::Success;
    res.tau01 = 0.0;
    res.g_near = g_i;
    res.d_pos = positionDistance(res.g_near, g_t);
    res.d_rot = rotationDistance(res.g_near, g_t);
    log(LogLevel::Error, "nearestPoseOnScrew: screwParameters failed");
    return res;
  }

  // 1D search on tau in [0, 1]
  // Coarse scan + local refinement (robust, simple).
  const int N = 101;  // coarse samples
  double best_tau = 0.0;
  double best_cost = std::numeric_limits<double>::infinity();
  Transform best_pose = g_i;

  for (int i = 0; i < N; ++i) {
    const double tau = static_cast<double>(i) / static_cast<double>(N - 1);
    const Transform g_tau = sclerp(g_i, g_f, tau);
    const double c = costPose(g_tau, g_t);
    if (c < best_cost) {
      best_cost = c;
      best_tau = tau;
      best_pose = g_tau;
    }
  }

  // Local refine around best_tau using ternary search on a small bracket
  double lo = std::max(0.0, best_tau - 1.0 / (N - 1));
  double hi = std::min(1.0, best_tau + 1.0 / (N - 1));

  for (int iter = 0; iter < 30; ++iter) {
    const double m1 = lo + (hi - lo) / 3.0;
    const double m2 = hi - (hi - lo) / 3.0;
    const double c1 = costPose(sclerp(g_i, g_f, m1), g_t);
    const double c2 = costPose(sclerp(g_i, g_f, m2), g_t);
    if (c1 < c2) hi = m2;
    else lo = m1;
  }

  const double tau_refined = 0.5 * (lo + hi);
  const Transform g_near = sclerp(g_i, g_f, tau_refined);

  res.status = Status::Success;
  res.tau01 = tau_refined;
  res.g_near = g_near;
  res.d_pos = positionDistance(g_near, g_t);
  res.d_rot = rotationDistance(g_near, g_t);
  return res;
}

Status screwSegments(const std::vector<Transform>& g_seq,
                     std::vector<unsigned int>* seg_end_indices,
                     double max_pos_d,
                     double max_rot_d) {
  if (!seg_end_indices) {
    log(LogLevel::Error, "screwSegments: null output");
    return Status::InvalidParameter;
  }
  seg_end_indices->clear();

  const int n = static_cast<int>(g_seq.size());
  if (n < 2) return Status::Success;

  int start = 0;
  while (start < n - 1) {
    int best_end = start + 1;

    // Try to extend segment end as far as possible.
    for (int end = start + 1; end < n; ++end) {
      bool segment_ok = true;

      // Check each pose in [start, end] fits the screw defined by endpoints.
      for (int k = start; k <= end; ++k) {
        const auto near = nearestPoseOnScrew(g_seq[start], g_seq[end], g_seq[k]);
        if (!ok(near.status) || near.d_pos > max_pos_d || near.d_rot > max_rot_d) {
          segment_ok = false;
          break;
        }
      }

      if (segment_ok) {
        best_end = end;
      } else {
        break;  // cannot extend further
      }
    }

    // Ensure forward progress
    if (best_end <= start) best_end = start + 1;

    seg_end_indices->push_back(static_cast<unsigned int>(best_end));
    start = best_end;
  }

  return Status::Success;
}

Status screwSegments(const std::vector<Mat4>& g_seq,
                     std::vector<unsigned int>* seg_end_indices,
                     double max_pos_d,
                     double max_rot_d) {
  if (!seg_end_indices) {
    log(LogLevel::Error, "screwSegments(Mat4): null output");
    return Status::InvalidParameter;
  }
  std::vector<Transform> seq;
  seq.reserve(g_seq.size());
  for (const auto& g : g_seq) {
    seq.push_back(transformFromMatrix4(g));
  }
  return screwSegments(seq, seg_end_indices, max_pos_d, max_rot_d);
}

}  // namespace sclerp::core
