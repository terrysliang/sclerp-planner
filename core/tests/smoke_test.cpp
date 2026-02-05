#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Core>

#include "sclerp/core/model/manipulator_model.hpp"
#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/planning/motion_plan.hpp"
#include "sclerp/core/math/se3.hpp"
#include "sclerp/core/math/distance.hpp"

using sclerp::core::ManipulatorBuilder;
using sclerp::core::ManipulatorModel;
using sclerp::core::KinematicsSolver;
using sclerp::core::Transform;
using sclerp::core::Vec3;
using sclerp::core::Status;
using sclerp::core::ok;

static bool near(double a, double b, double tol) {
  return std::abs(a - b) <= tol;
}

static void test_fk_and_jacobian() {
  const double L1 = 1.0;
  const double L2 = 1.0;

  // 2R planar in XY, rotations about Z
  // Space-frame POE:
  // joint1: w=[0,0,1], q=[0,0,0]
  // joint2: w=[0,0,1], q=[L1,0,0]
  Transform M = Transform::Identity();
  M.translation() = Vec3(L1 + L2, 0.0, 0.0);

  ManipulatorModel model;
  {
    const Status st = ManipulatorBuilder{}
      .set_ee_home(M)
      .add_revolute("j1", Vec3(0,0,1), Vec3(0,0,0))
      .add_revolute("j2", Vec3(0,0,1), Vec3(L1,0,0))
      .build(&model);
    assert(ok(st));
  }

  KinematicsSolver solver(model);

  // FK at q=[0,0] -> (L1+L2, 0, 0)
  Eigen::Vector2d q0(0.0, 0.0);
  Transform g0 = Transform::Identity();
  {
    const Status st = solver.forwardKinematics(q0, &g0);
    assert(ok(st));
  }
  assert(near(g0.translation().x(), L1 + L2, 1e-9));
  assert(near(g0.translation().y(), 0.0, 1e-9));

  // FK at q=[pi/2, 0] -> (0, L1+L2, 0)
  Eigen::Vector2d q1(M_PI * 0.5, 0.0);
  Transform g1 = Transform::Identity();
  {
    const Status st = solver.forwardKinematics(q1, &g1);
    assert(ok(st));
  }
  assert(std::abs(g1.translation().x()) < 1e-6);
  assert(std::abs(g1.translation().y() - (L1 + L2)) < 1e-6);

  // Jacobian finite-difference check (spatial twist):
  // xi_space ≈ log( g(q+eps ei) * g(q)^-1 ) / eps
  Eigen::Vector2d qtest(0.3, -0.2);
  Transform g = Transform::Identity();
  assert(ok(solver.forwardKinematics(qtest, &g)));

  Eigen::MatrixXd J;
  assert(ok(solver.spatialJacobian(qtest, &J)));
  assert(J.rows() == 6 && J.cols() == 2);

  const double eps = 1e-7;
  for (int i = 0; i < 2; ++i) {
    Eigen::Vector2d qn = qtest;
    qn(i) += eps;

    Transform gn = Transform::Identity();
    assert(ok(solver.forwardKinematics(qn, &gn)));

    Transform Trel = gn * g.inverse();
    const sclerp::core::Twist xi = (sclerp::core::logSE3(Trel) / eps).eval();

    // Compare twist
    const Eigen::VectorXd Ji = J.col(i);
    const double err = (xi - Ji).norm();
    assert(err < 1e-4); // loose but robust
  }
}

static void test_motion_plan() {
  const double L1 = 1.0;
  const double L2 = 1.0;

  Transform M = Transform::Identity();
  M.translation() = Vec3(L1 + L2, 0.0, 0.0);

  sclerp::core::JointLimit lim;
  lim.enabled = true;
  lim.lower = -M_PI;
  lim.upper = M_PI;

  ManipulatorModel model;
  {
    const Status st = ManipulatorBuilder{}
      .set_ee_home(M)
      .add_revolute("j1", Vec3(0,0,1), Vec3(0,0,0), lim)
      .add_revolute("j2", Vec3(0,0,1), Vec3(L1,0,0), lim)
      .build(&model);
    assert(ok(st));
  }

  KinematicsSolver solver(model);

  Eigen::Vector2d q_init(0.0, 0.0);
  Eigen::Vector2d q_goal(0.8, -0.4);

  Transform g_i = Transform::Identity();
  Transform g_f = Transform::Identity();
  assert(ok(solver.forwardKinematics(q_init, &g_i)));
  assert(ok(solver.forwardKinematics(q_goal, &g_f)));

  sclerp::core::MotionPlanRequest req;
  req.q_init = q_init;
  req.g_i = g_i;
  req.g_f = g_f;

  sclerp::core::MotionPlanOptions opt;
  opt.max_iters = 4000;
  opt.pos_tol = 1e-4;
  opt.rot_tol = 1e-3;
  opt.beta = 0.5;
  opt.tau = 0.001;
  opt.tau_i = 0.01;
  opt.tau_max = 0.1;
  opt.tau_break = 0.9;

  auto res = sclerp::core::planMotionSclerp(solver, req, opt);
  assert(res.status == Status::Success);
  assert(res.path.size() >= 2);

  // Verify final pose close to goal
  Transform g_end = Transform::Identity();
  assert(ok(solver.forwardKinematics(res.path.positions.back(), &g_end)));

  const double dp = sclerp::core::positionDistance(g_end, g_f);
  const double dr = sclerp::core::rotationDistance(g_end, g_f);
  assert(dp <= opt.pos_tol * 10.0);
  assert(dr <= opt.rot_tol * 10.0);
}

int main() {
  test_fk_and_jacobian();
  test_motion_plan();
  std::cout << "sclerp_core_smoke_test: PASS\n";
  return 0;
}
