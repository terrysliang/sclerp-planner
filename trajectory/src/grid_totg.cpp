#include "sclerp/trajectory/interpolator.hpp"

#include "sclerp/core/common/logger.hpp"
#include "detail/interpolator_common.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sclerp::trajectory {
namespace {

using sclerp::core::LogLevel;
using sclerp::core::log;
using sclerp::core::ok;

static double safeSqrt(double x) { return std::sqrt(std::max(0.0, x)); }

// Piecewise cubic Hermite spline per segment in arc-length s.
// With finite-difference tangents, this is C1 in general (q_ss may be discontinuous at knots).
struct HermiteSpline {
  std::vector<double> s;                 // size K
  std::vector<Eigen::VectorXd> q;        // size K
  std::vector<Eigen::VectorXd> m;        // size K

  int dof{0};

  int K() const { return static_cast<int>(s.size()); }
  double s0() const { return s.front(); }
  double sN() const { return s.back(); }

  int findSeg(double ss) const {
    const int Kk = K();
    if (ss <= s[0]) return 0;
    if (ss >= s[Kk - 1]) return Kk - 2;

    int lo = 0, hi = Kk - 1;
    while (hi - lo > 1) {
      int mid = (lo + hi) / 2;
      if (s[mid] <= ss) lo = mid;
      else hi = mid;
    }
    return std::min(lo, Kk - 2);
  }

  void eval(double ss, Eigen::VectorXd* out_q, Eigen::VectorXd* out_qs, Eigen::VectorXd* out_qss) const {
    const int i = findSeg(ss);
    const double s_i = s[i];
    const double s_ip1 = s[i + 1];
    double h = s_ip1 - s_i;
    if (!(h > 0.0)) h = 1e-9;

    double u = (ss - s_i) / h;
    u = std::clamp(u, 0.0, 1.0);

    const double u2 = u * u;
    const double u3 = u2 * u;

    const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    const double h10 = u3 - 2.0 * u2 + u;
    const double h01 = -2.0 * u3 + 3.0 * u2;
    const double h11 = u3 - u2;

    if (out_q) {
      *out_q = h00 * q[i] + (h * h10) * m[i] + h01 * q[i + 1] + (h * h11) * m[i + 1];
    }

    const double dh00 = 6.0 * u2 - 6.0 * u;
    const double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
    const double dh01 = -dh00;
    const double dh11 = 3.0 * u2 - 2.0 * u;

    const double d2h00 = 12.0 * u - 6.0;
    const double d2h10 = 6.0 * u - 4.0;
    const double d2h01 = -d2h00;
    const double d2h11 = 6.0 * u - 2.0;

    if (out_qs) {
      Eigen::VectorXd dqdu =
          dh00 * q[i] + (h * dh10) * m[i] + dh01 * q[i + 1] + (h * dh11) * m[i + 1];
      *out_qs = dqdu / h;
    }

    if (out_qss) {
      Eigen::VectorXd d2qdu2 =
          d2h00 * q[i] + (h * d2h10) * m[i] + d2h01 * q[i + 1] + (h * d2h11) * m[i + 1];
      *out_qss = d2qdu2 / (h * h);
    }
  }
};

static std::vector<double> chordLengthS(const std::vector<Eigen::VectorXd>& qs) {
  const int N = static_cast<int>(qs.size());
  std::vector<double> s(N, 0.0);
  double acc = 0.0;
  for (int i = 1; i < N; ++i) {
    acc += (qs[i] - qs[i - 1]).norm();
    s[i] = acc;
  }
  return s;
}

static std::vector<Eigen::VectorXd> resampleUniform(const std::vector<Eigen::VectorXd>& qs,
                                                    double ds_target,
                                                    int max_pts) {
  const int N = static_cast<int>(qs.size());
  if (N < 2) return qs;
  if (!(ds_target > 0.0)) return qs;

  const std::vector<double> s = chordLengthS(qs);
  const double S = s.back();
  if (!(S > 0.0)) return qs;

  const int dof = static_cast<int>(qs.front().size());
  const int M = std::min(max_pts, std::max(2, static_cast<int>(std::ceil(S / ds_target)) + 1));

  std::vector<Eigen::VectorXd> out;
  out.reserve(static_cast<std::size_t>(M));

  int seg = 0;
  for (int k = 0; k < M; ++k) {
    const double ss = (k == M - 1) ? S : (static_cast<double>(k) * (S / (M - 1)));
    while (seg < N - 2 && s[seg + 1] < ss) ++seg;

    const double s0 = s[seg];
    const double s1 = s[seg + 1];
    const double h = std::max(1e-12, s1 - s0);
    double u = (ss - s0) / h;
    u = std::clamp(u, 0.0, 1.0);

    Eigen::VectorXd q = (1.0 - u) * qs[seg] + u * qs[seg + 1];
    if (q.size() != dof) q.conservativeResize(dof);
    out.emplace_back(std::move(q));
  }
  return out;
}

static HermiteSpline buildHermite(const std::vector<Eigen::VectorXd>& qs) {
  HermiteSpline sp;
  const int N = static_cast<int>(qs.size());
  sp.dof = static_cast<int>(qs.front().size());
  sp.q = qs;
  sp.s = chordLengthS(qs);
  sp.m.resize(static_cast<std::size_t>(N), Eigen::VectorXd::Zero(sp.dof));

  for (int i = 0; i < N; ++i) {
    if (i == 0) {
      const double ds = std::max(1e-9, sp.s[1] - sp.s[0]);
      sp.m[0] = (sp.q[1] - sp.q[0]) / ds;
    } else if (i == N - 1) {
      const double ds = std::max(1e-9, sp.s[N - 1] - sp.s[N - 2]);
      sp.m[N - 1] = (sp.q[N - 1] - sp.q[N - 2]) / ds;
    } else {
      const double ds = std::max(1e-9, sp.s[i + 1] - sp.s[i - 1]);
      sp.m[i] = (sp.q[i + 1] - sp.q[i - 1]) / ds;
    }
  }
  return sp;
}

// Given q_s, q_ss at a point and current sdot, compute feasible bounds on sddot.
// Also returns an additional speed cap induced by near-zero q_s.
static void sddotBoundsFromJointAcc(const Eigen::VectorXd& q_s,
                                   const Eigen::VectorXd& q_ss,
                                   const Eigen::VectorXd& a_max,
                                   double sdot,
                                   double* out_sddot_min,
                                   double* out_sddot_max,
                                   double* out_sdot_cap_from_acc) {
  double sdd_min = -std::numeric_limits<double>::infinity();
  double sdd_max =  std::numeric_limits<double>::infinity();
  double sdot_cap =  std::numeric_limits<double>::infinity();

  const int dof = static_cast<int>(q_s.size());
  const double sdot2 = sdot * sdot;

  for (int j = 0; j < dof; ++j) {
    const double A = q_s[j];
    const double B = q_ss[j] * sdot2;
    const double a = std::abs(a_max[j]);

    if (std::abs(A) < 1e-10) {
      // Constraint reduces to |q_ss| * sdot^2 <= a
      const double c = std::abs(q_ss[j]);
      if (c > 1e-12) {
        sdot_cap = std::min(sdot_cap, std::sqrt(a / c));
      }
      continue;
    }

    double lo = (-a - B) / A;
    double hi = ( a - B) / A;
    if (lo > hi) std::swap(lo, hi);

    sdd_min = std::max(sdd_min, lo);
    sdd_max = std::min(sdd_max, hi);
  }

  *out_sddot_min = sdd_min;
  *out_sddot_max = sdd_max;
  *out_sdot_cap_from_acc = sdot_cap;
}

// Velocity cap from |q_s| * sdot <= v_max (and optionally near-zero q_s acceleration cap).
static double sdotCapAt(const Eigen::VectorXd& q_s,
                        const Eigen::VectorXd& q_ss,
                        const Eigen::VectorXd& v_max,
                        const Eigen::VectorXd& a_max) {
  const int dof = static_cast<int>(q_s.size());

  double cap_v = std::numeric_limits<double>::infinity();
  for (int j = 0; j < dof; ++j) {
    const double denom = std::abs(q_s[j]);
    if (denom < 1e-12) continue;
    cap_v = std::min(cap_v, std::abs(v_max[j]) / denom);
  }
  if (!std::isfinite(cap_v)) cap_v = 1e6;

  // Only apply sqrt(a/|q_ss|) when |q_s| is near zero (otherwise this is overly conservative).
  constexpr double kQsNearZero = 1e-6;
  double cap_nz = std::numeric_limits<double>::infinity();
  for (int j = 0; j < dof; ++j) {
    if (std::abs(q_s[j]) > kQsNearZero) continue;
    const double c = std::abs(q_ss[j]);
    if (c < 1e-12) continue;
    cap_nz = std::min(cap_nz, std::sqrt(std::abs(a_max[j]) / c));
  }

  double cap = std::min(cap_v, cap_nz);
  if (!std::isfinite(cap) || !(cap > 0.0)) cap = 1e6;
  return cap;
}

// Shrink v until sddot interval is feasible (sdd_min <= sdd_max). Cheap and very robust.
static void makeFeasibleV(const Eigen::VectorXd& q_s,
                          const Eigen::VectorXd& q_ss,
                          const Eigen::VectorXd& a_max,
                          double v_in,
                          double* v_out,
                          double* sdd_min_out,
                          double* sdd_max_out,
                          double* cap_from_acc_out) {
  double v_hi = std::max(0.0, v_in);
  double v_lo = 0.0;

  double best_v = 0.0;
  double best_min = -std::numeric_limits<double>::infinity();
  double best_max =  std::numeric_limits<double>::infinity();
  double best_cap =  std::numeric_limits<double>::infinity();

  // Try v_hi first.
  {
    double sdd_min, sdd_max, cap;
    sddotBoundsFromJointAcc(q_s, q_ss, a_max, v_hi, &sdd_min, &sdd_max, &cap);
    if (sdd_min <= sdd_max) {
      best_v = v_hi;
      best_min = sdd_min;
      best_max = sdd_max;
      best_cap = cap;
      *v_out = best_v;
      *sdd_min_out = best_min;
      *sdd_max_out = best_max;
      *cap_from_acc_out = best_cap;
      return;
    }
  }

  // Bisection
  for (int it = 0; it < 30; ++it) {
    const double v_mid = 0.5 * (v_lo + v_hi);
    double sdd_min, sdd_max, cap;
    sddotBoundsFromJointAcc(q_s, q_ss, a_max, v_mid, &sdd_min, &sdd_max, &cap);

    if (sdd_min <= sdd_max) {
      best_v = v_mid;
      best_min = sdd_min;
      best_max = sdd_max;
      best_cap = cap;
      v_lo = v_mid;
    } else {
      v_hi = v_mid;
    }
  }

  *v_out = best_v;
  *sdd_min_out = best_min;
  *sdd_max_out = best_max;
  *cap_from_acc_out = best_cap;
}

}  // namespace

GridTotg::GridTotg(Limits lim) : lim_(std::move(lim)), cfg_() {}
GridTotg::GridTotg(Limits lim, Config cfg) : lim_(std::move(lim)), cfg_(std::move(cfg)) {}

Status GridTotg::plan(const sclerp::core::JointPath& path,
                      double sample_dt,
                      PlannedTrajectory* out,
                      const std::vector<bool>& unwrap_revolute_mask) const {
  if (!detail::validOut(out, "GridTotg::plan: null out")) return Status::InvalidParameter;
  const Status st = plan(path.positions, sample_dt, out, unwrap_revolute_mask);
  if (ok(st)) out->joint_names = path.joint_names;
  return st;
}

Status GridTotg::plan(const std::vector<Eigen::VectorXd>& qs_in,
                      double sample_dt,
                      PlannedTrajectory* out,
                      const std::vector<bool>& unwrap_revolute_mask) const {
  if (!detail::validOut(out, "GridTotg::plan: null out")) return Status::InvalidParameter;
  *out = PlannedTrajectory{};

  const Status wp_st = detail::validateWaypoints(qs_in);
  if (!ok(wp_st)) return wp_st;
  const int dof = static_cast<int>(qs_in.front().size());

  if (!(sample_dt > 0.0) || !std::isfinite(sample_dt)) {
    log(LogLevel::Error, "GridTotg::plan: sample_dt must be > 0");
    return Status::InvalidParameter;
  }

  Eigen::VectorXd v_max, a_max, j_max;
  const Status lim_st = detail::validateLimits(lim_, dof, &v_max, &a_max, &j_max);
  if (!ok(lim_st)) return lim_st;

  std::vector<Eigen::VectorXd> qs = qs_in;

  if (cfg_.unwrap_angles) {
    detail::unwrapContinuousRevolute(&qs, unwrap_revolute_mask);
  }

  qs = detail::dropNearDuplicatesMaxAbs(qs, cfg_.dedup_eps);
  qs = detail::dropMicroSegmentsNorm(qs, cfg_.drop_eps);
  if (qs.size() < 2) {
    log(LogLevel::Error, "GridTotg::plan: path collapsed to <2 waypoints after preprocessing");
    return Status::InvalidParameter;
  }

  if (cfg_.resample_uniform) {
    qs = resampleUniform(qs, cfg_.resample_ds, cfg_.max_resample_points);
    qs = detail::dropMicroSegmentsNorm(qs, cfg_.drop_eps);
    if (qs.size() < 2) {
      log(LogLevel::Error, "GridTotg::plan: path collapsed after resampling");
      return Status::InvalidParameter;
    }
  }

  HermiteSpline sp = buildHermite(qs);
  const double S = sp.sN();
  if (!(S > 1e-12)) {
    log(LogLevel::Error, "GridTotg::plan: zero-length path");
    return Status::InvalidParameter;
  }

  const int M = std::min(cfg_.max_grid_points,
                         std::max(2, static_cast<int>(std::ceil(S / cfg_.grid_ds)) + 1));
  std::vector<double> s_grid(M, 0.0);
  for (int k = 0; k < M; ++k) {
    s_grid[k] = (k == M - 1) ? S : (static_cast<double>(k) * (S / (M - 1)));
  }

  // Precompute q_s, q_ss at nodes
  std::vector<Eigen::VectorXd> qs_grid(M), qss_grid(M);
  for (int k = 0; k < M; ++k) {
    Eigen::VectorXd q, q_s, q_ss;
    sp.eval(s_grid[k], &q, &q_s, &q_ss);
    qs_grid[k] = std::move(q_s);
    qss_grid[k] = std::move(q_ss);
  }

  // Node speed caps, then tighten with midpoint caps
  std::vector<double> sdot_cap(M, std::numeric_limits<double>::infinity());
  for (int k = 0; k < M; ++k) {
    sdot_cap[k] = sdotCapAt(qs_grid[k], qss_grid[k], v_max, a_max);
  }
  for (int k = 0; k < M - 1; ++k) {
    const double s_mid = 0.5 * (s_grid[k] + s_grid[k + 1]);
    Eigen::VectorXd q, q_s, q_ss;
    sp.eval(s_mid, &q, &q_s, &q_ss);
    const double cap_mid = sdotCapAt(q_s, q_ss, v_max, a_max);
    sdot_cap[k] = std::min(sdot_cap[k], cap_mid);
    sdot_cap[k + 1] = std::min(sdot_cap[k + 1], cap_mid);
  }

  // Forward-backward integration in s
  std::vector<double> sdot_fwd(M, 0.0), sdot_bwd(M, 0.0);

  const double sdot_start = (std::isfinite(cfg_.sdot_start) ? cfg_.sdot_start : 0.0);
  const double sdot_goal  = (std::isfinite(cfg_.sdot_goal)  ? cfg_.sdot_goal  : 0.0);

  sdot_fwd[0] = std::clamp(sdot_start, 0.0, sdot_cap[0]);
  for (int k = 0; k < M - 1; ++k) {
    const double ds = std::max(1e-12, s_grid[k + 1] - s_grid[k]);
    double v = sdot_fwd[k];
    if (!std::isfinite(v) || v < 0.0) v = 0.0;
    v = std::min(v, sdot_cap[k]);

    double sdd_min, sdd_max, cap_from_acc;
    makeFeasibleV(qs_grid[k], qss_grid[k], a_max, v, &v, &sdd_min, &sdd_max, &cap_from_acc);
    v = std::min(v, cap_from_acc);
    v = std::min(v, sdot_cap[k]);

    // Max feasible acceleration
    const double a_use = std::max(0.0, sdd_max);
    double v_next = safeSqrt(v * v + 2.0 * a_use * ds);
    v_next = std::min(v_next, sdot_cap[k + 1]);
    sdot_fwd[k + 1] = v_next;
  }

  sdot_bwd[M - 1] = std::clamp(sdot_goal, 0.0, sdot_cap[M - 1]);
  for (int k = M - 1; k > 0; --k) {
    const double ds = std::max(1e-12, s_grid[k] - s_grid[k - 1]);
    double v = sdot_bwd[k];
    if (!std::isfinite(v) || v < 0.0) v = 0.0;
    v = std::min(v, sdot_cap[k]);

    double sdd_min, sdd_max, cap_from_acc;
    makeFeasibleV(qs_grid[k], qss_grid[k], a_max, v, &v, &sdd_min, &sdd_max, &cap_from_acc);
    v = std::min(v, cap_from_acc);
    v = std::min(v, sdot_cap[k]);

    // Max feasible deceleration magnitude
    const double dec_use = std::max(0.0, -sdd_min);
    double v_prev = safeSqrt(v * v + 2.0 * dec_use * ds);
    v_prev = std::min(v_prev, sdot_cap[k - 1]);
    sdot_bwd[k - 1] = v_prev;
  }

  std::vector<double> sdot(M, 0.0);
  for (int k = 0; k < M; ++k) {
    sdot[k] = std::min({sdot_fwd[k], sdot_bwd[k], sdot_cap[k]});
    if (!std::isfinite(sdot[k]) || sdot[k] < 0.0) sdot[k] = 0.0;
  }

  // Integrate time along grid using trapezoid rule
  std::vector<double> t_grid(M, 0.0);
  double T = 0.0;
  for (int k = 0; k < M - 1; ++k) {
    const double ds = std::max(1e-12, s_grid[k + 1] - s_grid[k]);
    const double v0 = sdot[k];
    const double v1 = sdot[k + 1];

    double dt = 0.0;
    const double v_sum = v0 + v1;
    if (v_sum > 1e-9) {
      dt = 2.0 * ds / v_sum;
    } else {
      // Fallback: estimate dt from feasible acceleration at v=0
      double v = 0.0, sdd_min, sdd_max, cap_from_acc;
      makeFeasibleV(qs_grid[k], qss_grid[k], a_max, v, &v, &sdd_min, &sdd_max, &cap_from_acc);
      const double a_mag = std::max(1e-6, std::max(sdd_max, -sdd_min));
      dt = std::sqrt(2.0 * ds / a_mag);
    }

    if (!std::isfinite(dt) || dt < 0.0) dt = 0.0;
    T += dt;
    t_grid[k + 1] = T;
  }

  out->total_time = T;
  out->sample_dt = sample_dt;
  out->segs.clear();
  out->table.clear();

  const int steps = std::max(1, static_cast<int>(std::ceil(T / sample_dt)));
  out->table.reserve(static_cast<std::size_t>(steps + 1));

  // Sample uniformly in time. Use quadratic s(t) within each grid interval assuming constant sddot in s.
  int k = 0;
  for (int n = 0; n <= steps; ++n) {
    double t = n * sample_dt;
    if (t > T) t = T;

    while (k < M - 2 && t_grid[k + 1] < t) ++k;

    const double t0 = t_grid[k];
    const double t1 = t_grid[k + 1];
    const double dt = std::max(1e-12, t1 - t0);
    const double tau = std::clamp(t - t0, 0.0, dt);

    const double s0 = s_grid[k];
    const double s1 = s_grid[k + 1];
    const double v0 = sdot[k];
    const double v1 = sdot[k + 1];
    const double a = (v1 - v0) / dt;

    double s = s0 + v0 * tau + 0.5 * a * tau * tau;
    s = std::clamp(s, s0, s1);

    Eigen::VectorXd q, q_s, q_ss;
    sp.eval(s, &q, (cfg_.compute_qd_qdd ? &q_s : nullptr), (cfg_.compute_qd_qdd ? &q_ss : nullptr));

    Sample smp;
    smp.t = t;
    smp.q = std::move(q);

    if (cfg_.compute_qd_qdd) {
      const double sdot_now = v0 + a * tau;
      smp.qd = q_s * sdot_now;
      smp.qdd = q_s * a + q_ss * (sdot_now * sdot_now);
    } else {
      smp.qd = Eigen::VectorXd::Zero(dof);
      smp.qdd = Eigen::VectorXd::Zero(dof);
    }

    out->table.emplace_back(std::move(smp));
  }

  return Status::Success;
}

}  // namespace sclerp::trajectory
