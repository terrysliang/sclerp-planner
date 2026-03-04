#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

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
using sclerp::core::DualQuat;
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

static void test_rmrc_nullspace_projection() {
  constexpr int kDof = 7;
  constexpr double kLink = 0.3;

  Transform M = Transform::Identity();
  M.translation() = Vec3(kLink * static_cast<double>(kDof), 0.0, 0.0);

  sclerp::core::JointLimit lim;
  lim.enabled = true;
  lim.lower = -M_PI;
  lim.upper = M_PI;

  ManipulatorModel model;
  {
    ManipulatorBuilder b;
    b.set_ee_home(M);
    for (int i = 0; i < kDof; ++i) {
      const double x = kLink * static_cast<double>(i);
      b.add_revolute("j" + std::to_string(i + 1), Vec3(0, 0, 1), Vec3(x, 0, 0), lim);
    }
    const Status st = b.build(&model);
    assert(ok(st));
  }

  KinematicsSolver solver(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(kDof);
  q << 2.9, 2.7, 2.8, 0.0, 0.0, 0.0, 0.0;

  Transform g = Transform::Identity();
  assert(ok(solver.forwardKinematics(q, &g)));

  const DualQuat dq_i(g);
  const DualQuat dq_f(g);  // no primary task delta

  Eigen::VectorXd dq_out(kDof);

  // Nullspace disabled: expect ~0 increment when dq_i == dq_f.
  {
    sclerp::core::RmrcOptions ropt;
    ropt.nullspace.enabled = false;

    KinematicsSolver::RmrcWorkspace ws;
    assert(ok(solver.rmrcIncrement(dq_i, dq_f, q, &dq_out, ropt, &ws)));
    assert(dq_out.norm() < 1e-10);
  }

  // Nullspace enabled: expect a posture/limit-avoidance motion that is mostly task-invisible.
  {
    sclerp::core::RmrcOptions ropt;
    ropt.nullspace.enabled = true;
    ropt.nullspace.joint_limits.enabled = true;
    ropt.nullspace.posture.enabled = true;
    // Primary task delta is zero here, so disable ratio clamping to allow a measurable secondary step.
    ropt.nullspace.max_norm_ratio = 0.0;

    KinematicsSolver::RmrcWorkspace ws;
    assert(ok(solver.rmrcIncrement(dq_i, dq_f, q, &dq_out, ropt, &ws)));
    assert(dq_out.allFinite());
    assert(dq_out.norm() > 1e-6);

    // Cost function consistent with the implemented gradients, freezing the joint-limit activation
    // weights at the current q.
    std::vector<double> mid(static_cast<std::size_t>(kDof), 0.0);
    std::vector<double> half(static_cast<std::size_t>(kDof), 1.0);
    std::vector<double> w_act(static_cast<std::size_t>(kDof), 0.0);

    const double m = std::clamp(ropt.nullspace.joint_limits.margin_frac, 0.0, 0.49);
    for (int i = 0; i < kDof; ++i) {
      const auto& jl = model.joint(i).limit;
      const double range = jl.upper - jl.lower;
      mid[static_cast<std::size_t>(i)] = 0.5 * (jl.lower + jl.upper);
      half[static_cast<std::size_t>(i)] = 0.5 * range;

      const double x = (q(i) - mid[static_cast<std::size_t>(i)]) / half[static_cast<std::size_t>(i)];
      const double absx = std::abs(x);
      double w = 0.0;
      if (m <= 0.0) {
        w = 1.0;
      } else {
        const double x0 = 1.0 - 2.0 * m;
        if (absx > x0) {
          const double denom = std::max(1e-12, 1.0 - x0);
          w = std::clamp((absx - x0) / denom, 0.0, 1.0);
        }
      }
      w_act[static_cast<std::size_t>(i)] = w;
    }

    auto cost = [&](const Eigen::VectorXd& qq) {
      double c = 0.0;
      for (int i = 0; i < kDof; ++i) {
        const double x = (qq(i) - mid[static_cast<std::size_t>(i)]) / half[static_cast<std::size_t>(i)];
        c += ropt.nullspace.posture.weight * (x * x);
        c += ropt.nullspace.joint_limits.weight * w_act[static_cast<std::size_t>(i)] * (x * x);
      }
      return c;
    };

    const double c0 = cost(q);
    const double eps = 1e-3;
    const Eigen::VectorXd q1 = q + eps * dq_out;
    const double c1 = cost(q1);
    assert(c1 < c0);

    Eigen::MatrixXd J;
    assert(ok(solver.spatialJacobian(q, &J)));
    const Eigen::VectorXd task = (J * dq_out).eval();
    assert(task.norm() <= 1e-2 * dq_out.norm() + 1e-9);
  }
}

int main() {
  test_fk_and_jacobian();
  test_motion_plan();
  test_rmrc_nullspace_projection();
  std::cout << "sclerp_core_smoke_test: PASS\n";
  return 0;
}
