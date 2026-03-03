#pragma once

#include "sclerp/trajectory/interpolator.hpp"
#include "sclerp/core/common/logger.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

namespace sclerp::trajectory::detail {

using sclerp::core::LogLevel;
using sclerp::core::log;
using sclerp::core::ok;

inline bool validOut(const void* out, const char* msg) {
  if (out) return true;
  log(LogLevel::Error, msg);
  return false;
}

inline Eigen::VectorXd broadcastOrError(const Eigen::VectorXd& x, int dof, const char* name, Status* st) {
  if (st) *st = Status::Success;
  if (x.size() == dof) return x;
  if (x.size() == 1) return Eigen::VectorXd::Constant(dof, x[0]);

  if (st) *st = Status::InvalidParameter;
  std::ostringstream oss;
  oss << "trajectory: " << name << " must have size dof (" << dof << ") or 1 (broadcast)";
  log(LogLevel::Error, oss.str());
  return Eigen::VectorXd();
}

inline bool isFiniteLimitsVec(const Eigen::VectorXd& v) {
  return v.size() > 0 && v.allFinite();
}

inline double wrapToPi(double x) {
  // Avoid M_PI portability issues
  constexpr double kPi = 3.1415926535897932384626433832795;
  x = std::fmod(x + kPi, 2.0 * kPi);
  if (x < 0.0) x += 2.0 * kPi;
  return x - kPi;
}

inline void unwrapContinuousRevolute(std::vector<Eigen::VectorXd>* qs,
                                     const std::vector<bool>& unwrap_mask) {
  if (!qs || qs->size() < 2) return;
  const int dof = static_cast<int>((*qs)[0].size());
  if (static_cast<int>(unwrap_mask.size()) != dof) return;

  for (std::size_t i = 1; i < qs->size(); ++i) {
    Eigen::VectorXd& cur = (*qs)[i];
    const Eigen::VectorXd& prev = (*qs)[i - 1];
    for (int j = 0; j < dof; ++j) {
      if (!unwrap_mask[j]) continue;
      const double dq = wrapToPi(cur[j] - prev[j]);
      cur[j] = prev[j] + dq;
    }
  }
}

// 1) Dedup by per-joint max abs (good for exact duplicates / quantization noise)
inline std::vector<Eigen::VectorXd> dropNearDuplicatesMaxAbs(const std::vector<Eigen::VectorXd>& qs_in,
                                                             double eps_max_abs) {
  if (qs_in.empty()) return {};
  std::vector<Eigen::VectorXd> out;
  out.reserve(qs_in.size());
  out.push_back(qs_in.front());
  for (std::size_t i = 1; i < qs_in.size(); ++i) {
    const Eigen::VectorXd d = qs_in[i] - out.back();
    if (d.cwiseAbs().maxCoeff() < eps_max_abs) continue;
    out.push_back(qs_in[i]);
  }
  return out;
}

// 2) Drop micro segments by Euclidean norm (consistent with chord-length s)
inline std::vector<Eigen::VectorXd> dropMicroSegmentsNorm(const std::vector<Eigen::VectorXd>& qs_in,
                                                          double eps_norm) {
  if (qs_in.empty()) return {};
  std::vector<Eigen::VectorXd> out;
  out.reserve(qs_in.size());
  out.push_back(qs_in.front());
  for (std::size_t i = 1; i < qs_in.size(); ++i) {
    const Eigen::VectorXd d = qs_in[i] - out.back();
    if (d.norm() < eps_norm) continue;
    out.push_back(qs_in[i]);
  }
  return out;
}

// Backward-compatible wrapper: keep your old semantics.
inline std::vector<Eigen::VectorXd> dropDuplicatesAndMicroSegments(const std::vector<Eigen::VectorXd>& qs_in,
                                                                   double drop_eps) {
  return dropNearDuplicatesMaxAbs(qs_in, drop_eps);
}

inline Status validateWaypoints(const std::vector<Eigen::VectorXd>& qs_in) {
  if (qs_in.size() < 2) {
    log(LogLevel::Error, "trajectory: need at least 2 waypoints");
    return Status::InvalidParameter;
  }
  const int dof = static_cast<int>(qs_in.front().size());
  if (dof <= 0) {
    log(LogLevel::Error, "trajectory: invalid dof");
    return Status::InvalidParameter;
  }
  for (const auto& q : qs_in) {
    if (q.size() != dof) {
      log(LogLevel::Error, "trajectory: waypoint dof mismatch");
      return Status::InvalidParameter;
    }
    if (!q.allFinite()) {
      log(LogLevel::Error, "trajectory: waypoint contains NaN/Inf");
      return Status::InvalidParameter;
    }
  }
  return Status::Success;
}

inline Status validateLimits(const Limits& lim,
                            int dof,
                            Eigen::VectorXd* v_max_out,
                            Eigen::VectorXd* a_max_out,
                            Eigen::VectorXd* j_max_out) {
  if (!validOut(v_max_out, "trajectory: null v_max_out")) return Status::InvalidParameter;
  if (!validOut(a_max_out, "trajectory: null a_max_out")) return Status::InvalidParameter;
  if (!validOut(j_max_out, "trajectory: null j_max_out")) return Status::InvalidParameter;

  if (!isFiniteLimitsVec(lim.v_max) || !isFiniteLimitsVec(lim.a_max)) {
    log(LogLevel::Error, "trajectory: v_max and a_max must be provided and finite");
    return Status::InvalidParameter;
  }

  Status st = Status::Success;
  Eigen::VectorXd v_max = broadcastOrError(lim.v_max, dof, "v_max", &st);
  if (!ok(st)) return st;
  Eigen::VectorXd a_max = broadcastOrError(lim.a_max, dof, "a_max", &st);
  if (!ok(st)) return st;

  Eigen::VectorXd j_max;
  if (lim.j_max.size() == 0) {
    j_max = Eigen::VectorXd::Constant(dof, std::numeric_limits<double>::infinity());
  } else {
    j_max = broadcastOrError(lim.j_max, dof, "j_max", &st);
    if (!ok(st)) return st;
  }

  for (int i = 0; i < dof; ++i) {
    if (!(v_max[i] > 0.0) || !std::isfinite(v_max[i])) {
      log(LogLevel::Error, "trajectory: v_max entries must be > 0 and finite");
      return Status::InvalidParameter;
    }
    if (!(a_max[i] > 0.0) || !std::isfinite(a_max[i])) {
      log(LogLevel::Error, "trajectory: a_max entries must be > 0 and finite");
      return Status::InvalidParameter;
    }
    if (!std::isfinite(j_max[i])) {
      if (std::isinf(j_max[i])) continue;
      log(LogLevel::Error, "trajectory: j_max entries must be finite or +inf");
      return Status::InvalidParameter;
    }
    if (!(j_max[i] > 0.0)) {
      log(LogLevel::Error, "trajectory: j_max entries must be > 0");
      return Status::InvalidParameter;
    }
  }

  *v_max_out = std::move(v_max);
  *a_max_out = std::move(a_max);
  *j_max_out = std::move(j_max);
  return Status::Success;
}

}  // namespace sclerp::trajectory::detail