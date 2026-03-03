#include "sclerp/trajectory/interpolator.hpp"

#include "sclerp/core/common/logger.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#if defined(SCLERP_TRAJECTORY_HAS_TOPPRA)
#include <toppra/toppra.hpp>
#include <toppra/geometric_path.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/algorithm/toppra.hpp>
#include <toppra/parametrizer/const_accel.hpp>
#endif

namespace sclerp::trajectory {
namespace {

using sclerp::core::LogLevel;
using sclerp::core::log;
using sclerp::core::ok;

#if defined(SCLERP_TRAJECTORY_HAS_TOPPRA)
static double wrapToPi(double x) {
  x = std::fmod(x + M_PI, 2.0 * M_PI);
  if (x < 0.0) x += 2.0 * M_PI;
  return x - M_PI;
}

static void unwrapContinuousRevolute(std::vector<Eigen::VectorXd>* qs,
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

static Eigen::VectorXd broadcastOrError(const Eigen::VectorXd& x, int dof, const char* name, Status* st) {
  if (st) *st = Status::Success;
  if (x.size() == dof) return x;
  if (x.size() == 1) return Eigen::VectorXd::Constant(dof, x[0]);
  if (st) *st = Status::InvalidParameter;
  std::ostringstream oss;
  oss << "planWithToppra: " << name << " must have size dof (" << dof << ") or 1 (broadcast)";
  log(LogLevel::Error, oss.str());
  return {};
}
#endif

}  // namespace

Status planWithToppra(const sclerp::core::JointPath& path,
                      const Limits& lim,
                      double sample_dt,
                      PlannedTrajectory* out,
                      const std::vector<bool>& unwrap_revolute_mask) {
  if (!out) {
    log(LogLevel::Error, "planWithToppra: null out");
    return Status::InvalidParameter;
  }
  const Status st = planWithToppra(path.positions, lim, sample_dt, out, unwrap_revolute_mask);
  if (ok(st)) out->joint_names = path.joint_names;
  return st;
}

Status planWithToppra(const std::vector<Eigen::VectorXd>& qs_in,
                      const Limits& lim,
                      double sample_dt,
                      PlannedTrajectory* out,
                      const std::vector<bool>& unwrap_revolute_mask) {
  if (!out) {
    log(LogLevel::Error, "planWithToppra: null out");
    return Status::InvalidParameter;
  }
  *out = PlannedTrajectory{};

#if !defined(SCLERP_TRAJECTORY_HAS_TOPPRA)
  (void)qs_in;
  (void)lim;
  (void)sample_dt;
  (void)unwrap_revolute_mask;
  log(LogLevel::Error, "planWithToppra: TOPPRA support not built (SCLERP_TRAJECTORY_HAS_TOPPRA is not set)");
  return Status::Failure;
#else
  if (qs_in.size() < 2) {
    log(LogLevel::Error, "planWithToppra: need at least 2 waypoints");
    return Status::InvalidParameter;
  }
  if (!(sample_dt > 0.0) || !std::isfinite(sample_dt)) {
    log(LogLevel::Error, "planWithToppra: sample_dt must be > 0");
    return Status::InvalidParameter;
  }

  const int dof = static_cast<int>(qs_in.front().size());
  if (dof <= 0) {
    log(LogLevel::Error, "planWithToppra: invalid dof");
    return Status::InvalidParameter;
  }
  for (const auto& q : qs_in) {
    if (q.size() != dof) {
      log(LogLevel::Error, "planWithToppra: waypoint dof mismatch");
      return Status::InvalidParameter;
    }
    if (!q.allFinite()) {
      log(LogLevel::Error, "planWithToppra: waypoint contains NaN/Inf");
      return Status::InvalidParameter;
    }
  }

  if (lim.v_max.size() == 0 || lim.a_max.size() == 0) {
    log(LogLevel::Error, "planWithToppra: lim.v_max and lim.a_max must be provided");
    return Status::InvalidParameter;
  }

  Status st = Status::Success;
  const Eigen::VectorXd v_max = broadcastOrError(lim.v_max, dof, "v_max", &st);
  if (!ok(st)) return st;
  const Eigen::VectorXd a_max = broadcastOrError(lim.a_max, dof, "a_max", &st);
  if (!ok(st)) return st;

  for (int j = 0; j < dof; ++j) {
    if (!(v_max[j] > 0.0) || !std::isfinite(v_max[j])) {
      log(LogLevel::Error, "planWithToppra: v_max entries must be > 0 and finite");
      return Status::InvalidParameter;
    }
    if (!(a_max[j] > 0.0) || !std::isfinite(a_max[j])) {
      log(LogLevel::Error, "planWithToppra: a_max entries must be > 0 and finite");
      return Status::InvalidParameter;
    }
  }

  std::vector<Eigen::VectorXd> qs = qs_in;
  unwrapContinuousRevolute(&qs, unwrap_revolute_mask);

  // ---- Build TOPPRA input ----
  namespace tp = toppra;
  namespace tp_gp = toppra;
  namespace tp_cst = toppra::constraint;
  namespace tp_alg = toppra::algorithm;
  namespace tp_par = toppra::parametrizer;

  tp::Vectors positions;
  positions.reserve(static_cast<std::size_t>(qs.size()));
  for (const auto& q_e : qs) {
    tp::Vector q(dof);
    for (int j = 0; j < dof; ++j) q(j) = q_e(j);
    positions.push_back(q);
  }

  const int N = static_cast<int>(qs.size());
  tp::Vector times(N);
  for (int i = 0; i < N; ++i) {
    times(i) = (N == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(N - 1);
  }

  tp::BoundaryCondFull bc;
  bc[0] = tp::BoundaryCond("natural");
  bc[1] = tp::BoundaryCond("natural");

  tp_gp::PiecewisePolyPath path_obj = tp_gp::PiecewisePolyPath::CubicSpline(positions, times, bc);
  auto path = std::make_shared<tp_gp::PiecewisePolyPath>(path_obj);
  tp::GeometricPathPtr path_ptr = path;

  tp::Vector vmax(dof), amax(dof);
  for (int j = 0; j < dof; ++j) {
    vmax(j) = v_max[j];
    amax(j) = a_max[j];
  }

  const tp::Vector vmin = -vmax;
  const tp::Vector amin = -amax;

  auto pc_vel = std::make_shared<tp_cst::LinearJointVelocity>(vmin, vmax);
  auto pc_acc = std::make_shared<tp_cst::LinearJointAcceleration>(amin, amax);

  tp::LinearConstraintPtrs constraints;
  constraints.push_back(pc_vel);
  constraints.push_back(pc_acc);

  tp_alg::TOPPRA algo(constraints, path_ptr);
  algo.setN(std::max(1, N - 1));

  const toppra::ReturnCode rc = algo.computePathParametrization(0.0, 0.0);
  if (rc != toppra::ReturnCode::OK) {
    log(LogLevel::Error, "planWithToppra: TOPPRA computePathParametrization failed");
    return Status::Failure;
  }

  const toppra::ParametrizationData pdata = algo.getParameterizationData();
  if (pdata.ret_code != tp::ReturnCode::OK) {
    log(LogLevel::Error, "planWithToppra: TOPPRA getParameterizationData failed");
    return Status::Failure;
  }

  auto param = std::make_shared<tp_par::ConstAccel>(path_ptr, pdata.gridpoints, pdata.parametrization);
  const tp::Bound time_range = param->pathInterval();
  const double t0 = time_range(0);
  const double t1 = time_range(1);

  if (!(t1 >= t0) || !std::isfinite(t0) || !std::isfinite(t1)) {
    log(LogLevel::Error, "planWithToppra: invalid TOPPRA time interval");
    return Status::Failure;
  }

  const double T_total = t1 - t0;
  out->total_time = T_total;
  out->sample_dt = sample_dt;
  out->segs.clear();
  out->table.clear();

  const int K = std::max(1, static_cast<int>(std::ceil(T_total / sample_dt)));
  out->table.reserve(static_cast<std::size_t>(K + 1));

  for (int k = 0; k <= K; ++k) {
    const double t_abs = t0 + std::min(T_total, k * sample_dt);
    const double t_rel = t_abs - t0;

    const tp::Vector q = param->eval_single(t_abs, 0);
    const tp::Vector qd = param->eval_single(t_abs, 1);
    const tp::Vector qdd = param->eval_single(t_abs, 2);

    Sample s;
    s.t = t_rel;
    s.q = Eigen::Map<const Eigen::VectorXd>(q.data(), dof);
    s.qd = Eigen::Map<const Eigen::VectorXd>(qd.data(), dof);
    s.qdd = Eigen::Map<const Eigen::VectorXd>(qdd.data(), dof);
    out->table.emplace_back(std::move(s));
  }

  return Status::Success;
#endif
}

}  // namespace sclerp::trajectory
