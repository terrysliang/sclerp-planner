#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>

#include <Eigen/Core>

#include "sclerp/urdf/load_manipulator.hpp"
#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/math/se3.hpp"
#include "sclerp/core/math/distance.hpp"

using sclerp::core::Transform;
using sclerp::core::Vec3;
using sclerp::core::Status;
using sclerp::core::ok;

static bool near(double a, double b, double tol) {
  return std::abs(a - b) <= tol;
}

static std::string make2RUrdf(double L1, double L2) {
  // 2 revolute joints about Z.
  // joint2 origin is at x=L1 relative to link1.
  // fixed joint to tool0 at x=L2 relative to link2.
  std::ostringstream oss;
  oss <<
R"(
<robot name="two_r_planar">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="tool0"/>

  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.1415926535" upper="3.1415926535" effort="1" velocity="1"/>
  </joint>

  <joint name="j2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz=")" << L1 << R"( 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.1415926535" upper="3.1415926535" effort="1" velocity="1"/>
  </joint>

  <joint name="tool_fixed" type="fixed">
    <parent link="link2"/>
    <child link="tool0"/>
    <origin xyz=")" << L2 << R"( 0 0" rpy="0 0 0"/>
  </joint>
</robot>
)";
  return oss.str();
}

static void test_urdf_load_and_fk() {
  const double L1 = 1.0;
  const double L2 = 1.0;

  const std::string urdf = make2RUrdf(L1, L2);

  sclerp::urdf::LoadOptions opt;
  opt.base_link = "base_link";
  opt.tip_link  = "tool0";

  auto lr = sclerp::urdf::loadManipulatorModelFromString(urdf, opt);
  assert(ok(lr.status));

  // Check dof and names
  assert(lr.model.dof() == 2);
  const auto& names = lr.model.joint_names();
  assert(names.size() == 2);
  assert(names[0] == "j1");
  assert(names[1] == "j2");

  sclerp::core::KinematicsSolver solver(lr.model);

  // FK at q=[0,0] -> x = L1+L2
  Eigen::Vector2d q0(0.0, 0.0);
  Transform g0 = Transform::Identity();
  assert(ok(solver.forwardKinematics(q0, &g0)));
  assert(near(g0.translation().x(), L1 + L2, 1e-9));
  assert(near(g0.translation().y(), 0.0, 1e-9));

  // FK at q=[pi/2, 0] -> y = L1+L2
  Eigen::Vector2d q1(M_PI * 0.5, 0.0);
  Transform g1 = Transform::Identity();
  assert(ok(solver.forwardKinematics(q1, &g1)));
  assert(std::abs(g1.translation().x()) < 1e-6);
  assert(std::abs(g1.translation().y() - (L1 + L2)) < 1e-6);
}

static void test_jacobian_fd() {
  const double L1 = 1.0;
  const double L2 = 1.0;

  const std::string urdf = make2RUrdf(L1, L2);

  sclerp::urdf::LoadOptions opt;
  opt.base_link = "base_link";
  opt.tip_link  = "tool0";

  auto lr = sclerp::urdf::loadManipulatorModelFromString(urdf, opt);
  assert(ok(lr.status));

  sclerp::core::KinematicsSolver solver(lr.model);

  Eigen::Vector2d q(0.3, -0.2);

  Transform g = Transform::Identity();
  assert(ok(solver.forwardKinematics(q, &g)));

  Eigen::MatrixXd J;
  assert(ok(solver.spatialJacobian(q, &J)));
  assert(J.rows() == 6 && J.cols() == 2);

  // Spatial finite difference:
  // xi_space ≈ log( g(q+eps ei) * g(q)^-1 ) / eps
  const double eps = 1e-7;

  for (int i = 0; i < 2; ++i) {
    Eigen::Vector2d qn = q;
    qn(i) += eps;

    Transform gn = Transform::Identity();
    assert(ok(solver.forwardKinematics(qn, &gn)));

    const Transform Trel = gn * g.inverse();
    const sclerp::core::Twist xi = (sclerp::core::logSE3(Trel) / eps).eval();
    const double err = (xi - J.col(i)).norm();
    // Loose tolerance: depends on eps + log/exp numeric conditioning
    assert(err < 1e-4);
  }
}

int main() {
  test_urdf_load_and_fk();
  test_jacobian_fd();
  std::cout << "sclerp_urdf_kinematics_test: PASS\n";
  return 0;
}
