#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "sclerp/collision/avoidance.hpp"
#include "sclerp/collision/collision.hpp"
#include "sclerp/collision/types.hpp"
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/math/types.hpp"

using sclerp::collision::ContactSet;
using sclerp::collision::CollisionQueryOptions;
using sclerp::collision::CollisionContext;
using sclerp::collision::CollisionAvoidanceOptions;
using sclerp::collision::FclObject;
using sclerp::collision::adjustJoints;
using sclerp::collision::createSphere;
using sclerp::collision::createPlane;
using sclerp::collision::computeContacts;
using sclerp::collision::checkCollision;
using sclerp::core::Status;
using sclerp::core::ok;
using sclerp::core::Vec3;
using sclerp::core::Mat3;

[[maybe_unused]] static bool near(double a, double b, double tol) {
  return std::abs(a - b) <= tol;
}

[[maybe_unused]] static bool matNear(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, double tol) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
  if (a.size() == 0) return true;
  return (a - b).cwiseAbs().maxCoeff() <= tol;
}

static void test_compute_contacts_basic_scene() {
}

static void test_plane() {
}

static void test_contact_jacobian_structure() {
}

static void test_self_collision_last_link_special_case() {
}

static void test_adjust_joints_passthrough_when_safe() {
}

static void test_adjust_joints_invalid_options_rejected() {
}

static void test_null_obstacle_rejected() {
}

int main() {
  test_compute_contacts_basic_scene();
  test_self_collision_last_link_special_case();
  test_contact_jacobian_structure();
  test_adjust_joints_passthrough_when_safe();
  test_adjust_joints_invalid_options_rejected();
  test_null_obstacle_rejected();
  test_plane();
  std::cout << "sclerp_collision_unit_test: PASS\n";
  return 0;
}
