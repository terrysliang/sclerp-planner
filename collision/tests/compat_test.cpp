#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "sclerp/collision/collision_utils.hpp"
#include "sclerp/collision/types.hpp"
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/math/types.hpp"

using sclerp::collision::ContactSet;
using sclerp::collision::CollisionQueryOptions;
using sclerp::collision::ObstacleBase;
using sclerp::collision::createSphere;
using sclerp::collision::getCollisionInfo;
using sclerp::collision::computeContacts;
using sclerp::core::Status;
using sclerp::core::ok;
using sclerp::core::Vec3;
using sclerp::core::Mat3;

static bool near(double a, double b, double tol) {
  return std::abs(a - b) <= tol;
}

static bool vecNear(const Vec3& a, const Vec3& b, double tol) {
  return (a - b).norm() <= tol;
}

static bool matNear(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, double tol) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
  if (a.size() == 0) return true;
  return (a - b).cwiseAbs().maxCoeff() <= tol;
}

static void test_compat() {
  constexpr double tol = 1e-9;

  // Build a minimal scene: base + 2 links, 1 obstacle.
  std::vector<std::shared_ptr<ObstacleBase>> link_cylinders;
  link_cylinders.reserve(3);

  std::shared_ptr<ObstacleBase> base;
  assert(ok(createSphere(0.1, Vec3(0.0, 0.0, 0.0), Mat3::Identity(), &base)));
  link_cylinders.push_back(base);

  std::shared_ptr<ObstacleBase> link1;
  assert(ok(createSphere(0.1, Vec3(1.0, 0.0, 0.0), Mat3::Identity(), &link1)));
  link_cylinders.push_back(link1);

  std::shared_ptr<ObstacleBase> link2;
  assert(ok(createSphere(0.1, Vec3(2.0, 0.0, 0.0), Mat3::Identity(), &link2)));
  link_cylinders.push_back(link2);

  std::vector<std::shared_ptr<ObstacleBase>> obstacles;
  std::shared_ptr<ObstacleBase> obstacle;
  assert(ok(createSphere(0.1, Vec3(5.0, 0.0, 0.0), Mat3::Identity(), &obstacle)));
  obstacles.push_back(obstacle);

  const std::shared_ptr<ObstacleBase> grasped_object = nullptr;

  // Spatial Jacobian (6 x dof)
  const int dof = 2;
  Eigen::MatrixXd spatial_jacobian(6, dof);
  spatial_jacobian << 1, 2,
                      3, 4,
                      5, 6,
                      7, 8,
                      9, 10,
                      11, 12;

  Eigen::MatrixXd contact_normal_array;
  std::vector<double> dist_array;
  std::vector<Eigen::MatrixXd> contact_points_array;
  std::vector<Eigen::MatrixXd> j_contact_array;

  const Status st = getCollisionInfo(
      link_cylinders,
      obstacles,
      grasped_object,
      spatial_jacobian,
      /*check_self_collision=*/true,
      /*num_links_ignore=*/0,
      dof,
      &contact_normal_array,
      &dist_array,
      &contact_points_array,
      &j_contact_array);
  assert(ok(st));

  CollisionQueryOptions opt;
  opt.check_self_collision = true;
  opt.num_links_ignore = 0;
  opt.dof = dof;

  ContactSet contacts;
  assert(ok(computeContacts(link_cylinders,
                            obstacles,
                            grasped_object,
                            spatial_jacobian,
                            opt,
                            &contacts)));

  assert(dist_array.size() == contacts.contacts.size());
  for (size_t i = 0; i < dist_array.size(); ++i) {
    const auto& c = contacts.contacts[i];
    assert(near(dist_array[i], c.distance, tol));
    assert(vecNear(contact_normal_array.block<3,1>(0, static_cast<int>(i)), c.normal, tol));

    if (contact_points_array[i].rows() == 3 && contact_points_array[i].cols() == 2) {
      assert(vecNear(contact_points_array[i].col(0), c.point_obj, tol));
      assert(vecNear(contact_points_array[i].col(1), c.point_link, tol));
    }

    if (i < j_contact_array.size()) {
      assert(matNear(j_contact_array[i], c.J_contact, tol));
    }
  }
}

int main() {
  test_compat();
  std::cout << "sclerp_collision_compat_test: PASS\n";
  return 0;
}
