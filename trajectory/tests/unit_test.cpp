#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "sclerp/core/common/status.hpp"
#include "sclerp/trajectory/interpolator.hpp"

using sclerp::core::Status;
using sclerp::core::ok;
using sclerp::trajectory::CsvMode;
using sclerp::trajectory::Limits;
using sclerp::trajectory::PlannedTrajectory;
using sclerp::trajectory::GridTotg;
using sclerp::trajectory::planWithToppra;
using sclerp::trajectory::writeTrajectoryCsv;

static std::string slurp(const std::string& path) {
  std::ifstream in(path);
  assert(in && "failed to open file");
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

static void test_grid_totg_respects_limits_on_samples() {
  const int dof = 2;
  std::vector<Eigen::VectorXd> qs;
  qs.push_back((Eigen::Vector2d() << 0.0, 0.0).finished());
  qs.push_back((Eigen::Vector2d() << 0.8, -0.4).finished());
  qs.push_back((Eigen::Vector2d() << 1.2, -0.1).finished());

  Limits lim;
  lim.v_max = (Eigen::Vector2d() << 0.6, 0.5).finished();
  lim.a_max = (Eigen::Vector2d() << 1.5, 1.2).finished();
  lim.j_max = Eigen::VectorXd();  // unused

  GridTotg::Config cfg;
  cfg.compute_qd_qdd = true;
  GridTotg totg(lim, cfg);
  PlannedTrajectory traj;
  const double dt = 0.01;
  const Status st = totg.plan(qs, dt, &traj);
  assert(ok(st));
  assert(!traj.table.empty());

  assert(traj.table.front().q.size() == dof);
  assert(std::abs(traj.table.front().q[0] - qs.front()[0]) < 1e-9);
  assert(std::abs(traj.table.front().q[1] - qs.front()[1]) < 1e-9);

  double max_v0 = 0.0, max_v1 = 0.0;
  for (const auto& s : traj.table) {
    assert(s.q.size() == dof);
    assert(s.qd.size() == dof);
    assert(s.qdd.size() == dof);
    assert(std::isfinite(s.t));
    assert(s.q.allFinite());
    assert(s.qd.allFinite());
    assert(s.qdd.allFinite());

    max_v0 = std::max(max_v0, std::abs(s.qd[0]));
    max_v1 = std::max(max_v1, std::abs(s.qd[1]));
  }

  const double v0_lim = lim.v_max[0] + 1e-3;
  const double v1_lim = lim.v_max[1] + 1e-3;

  assert(max_v0 <= v0_lim);
  assert(max_v1 <= v1_lim);
}

static void test_grid_totg_two_point_basic() {
  const int dof = 3;
  std::vector<Eigen::VectorXd> qs;
  qs.push_back((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  qs.push_back((Eigen::Vector3d() << 0.5, -0.2, 0.1).finished());

  Limits lim;
  lim.v_max = Eigen::VectorXd::Constant(dof, 0.8);
  lim.a_max = Eigen::VectorXd::Constant(dof, 2.0);
  lim.j_max = Eigen::VectorXd();

  GridTotg totg(lim);
  PlannedTrajectory traj;
  const Status st = totg.plan(qs, 0.01, &traj);
  assert(ok(st));
  assert(!traj.table.empty());

  // First and last samples should match endpoints (within small tolerance).
  assert((traj.table.front().q - qs.front()).cwiseAbs().maxCoeff() < 1e-9);
  assert((traj.table.back().q - qs.back()).cwiseAbs().maxCoeff() < 1e-9);
}

static void test_unwrap_mask_applied_only_when_provided() {
  std::vector<Eigen::VectorXd> qs;
  qs.push_back((Eigen::VectorXd(1) << 3.0).finished());
  qs.push_back((Eigen::VectorXd(1) << -3.0).finished());

  Limits lim;
  lim.v_max = Eigen::VectorXd::Constant(1, 1.0);
  lim.a_max = Eigen::VectorXd::Constant(1, 2.0);
  lim.j_max = Eigen::VectorXd();

  PlannedTrajectory traj_no_unwrap;
  {
    GridTotg totg(lim);
    assert(ok(totg.plan(qs, 0.01, &traj_no_unwrap)));
    assert(std::abs(traj_no_unwrap.table.back().q[0] - (-3.0)) < 1e-9);
  }

  PlannedTrajectory traj_unwrap;
  {
    GridTotg totg(lim);
    const std::vector<bool> mask = {true};
    assert(ok(totg.plan(qs, 0.01, &traj_unwrap, mask)));
    // Expect the goal to be unwrapped near 3.283.. instead of -3.0.
    assert(traj_unwrap.table.back().q[0] > M_PI);
  }
}

static void test_write_csv_position_only() {
  std::vector<Eigen::VectorXd> qs;
  qs.push_back((Eigen::Vector2d() << 0.0, 0.0).finished());
  qs.push_back((Eigen::Vector2d() << 0.2, -0.1).finished());

  Limits lim;
  lim.v_max = (Eigen::Vector2d() << 1.0, 1.0).finished();
  lim.a_max = (Eigen::Vector2d() << 2.0, 2.0).finished();
  lim.j_max = Eigen::VectorXd();

  GridTotg totg(lim);
  PlannedTrajectory traj;
  assert(ok(totg.plan(qs, 0.01, &traj)));
  traj.joint_names = {"j1", "j2"};

  const std::string path = "sclerp_trajectory_test.csv";
  assert(ok(writeTrajectoryCsv(traj, path, CsvMode::PositionOnly)));
  const std::string csv = slurp(path);
  assert(csv.rfind("time,j1,j2\n", 0) == 0);
  std::remove(path.c_str());
}

static void test_toppra_basic_if_available() {
#if defined(SCLERP_TRAJECTORY_HAS_TOPPRA)
  const int dof = 2;
  std::vector<Eigen::VectorXd> qs;
  qs.push_back((Eigen::Vector2d() << 0.0, 0.0).finished());
  qs.push_back((Eigen::Vector2d() << 0.6, -0.2).finished());
  qs.push_back((Eigen::Vector2d() << 1.0, 0.1).finished());

  Limits lim;
  lim.v_max = Eigen::VectorXd::Constant(dof, 1.0);
  lim.a_max = Eigen::VectorXd::Constant(dof, 2.0);
  lim.j_max = Eigen::VectorXd();

  PlannedTrajectory traj;
  const double dt = 0.01;
  const Status st = planWithToppra(qs, lim, dt, &traj);
  assert(ok(st));
  assert(!traj.table.empty());
  assert(traj.sample_dt == dt);
  assert(traj.total_time > 0.0);

  assert(std::abs(traj.table.front().t) < 1e-12);
  assert((traj.table.front().q - qs.front()).cwiseAbs().maxCoeff() < 1e-8);
  assert((traj.table.back().q - qs.back()).cwiseAbs().maxCoeff() < 1e-6);
  assert(std::abs(traj.table.back().t - traj.total_time) < 1e-9);

  for (const auto& s : traj.table) {
    assert(s.q.size() == dof);
    assert(s.qd.size() == dof);
    assert(s.qdd.size() == dof);
    assert(std::isfinite(s.t));
    assert(s.q.allFinite());
    assert(s.qd.allFinite());
    assert(s.qdd.allFinite());
    for (int j = 0; j < dof; ++j) {
      assert(std::abs(s.qd[j]) <= lim.v_max[j] + 1e-5);
      assert(std::abs(s.qdd[j]) <= lim.a_max[j] + 1e-4);
    }
  }
#endif
}

int main() {
  test_grid_totg_respects_limits_on_samples();
  test_grid_totg_two_point_basic();
  test_unwrap_mask_applied_only_when_provided();
  test_write_csv_position_only();
  test_toppra_basic_if_available();
  std::cout << "sclerp_trajectory_unit_test: PASS\n";
  return 0;
}
