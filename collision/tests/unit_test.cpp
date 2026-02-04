#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "sclerp/collision/avoidance.hpp"
#include "sclerp/collision/collision_utils.hpp"
#include "sclerp/collision/types.hpp"
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/math/types.hpp"

using sclerp::collision::ContactSet;
using sclerp::collision::CollisionQueryOptions;
using sclerp::collision::FclObject;
using sclerp::collision::adjustJoints;
using sclerp::collision::getContactJacobian;
using sclerp::collision::createSphere;
using sclerp::collision::createPlane;
using sclerp::collision::getCollisionInfo;
using sclerp::collision::computeContacts;
using sclerp::collision::checkCollision;
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

static void test_compute_contacts_consistency() {
  constexpr double tol = 1e-9;

  // Build a minimal scene: base + 2 links, 1 obstacle.
  std::vector<std::shared_ptr<FclObject>> link_cylinders;
  link_cylinders.reserve(3);

  std::shared_ptr<FclObject> base;
  assert(ok(createSphere(0.1, Vec3(0.0, 0.0, 0.0), Mat3::Identity(), &base)));
  link_cylinders.push_back(base);

  std::shared_ptr<FclObject> link1;
  assert(ok(createSphere(0.1, Vec3(1.0, 0.0, 0.0), Mat3::Identity(), &link1)));
  link_cylinders.push_back(link1);

  std::shared_ptr<FclObject> link2;
  assert(ok(createSphere(0.1, Vec3(2.0, 0.0, 0.0), Mat3::Identity(), &link2)));
  link_cylinders.push_back(link2);

  std::vector<std::shared_ptr<FclObject>> obstacles;
  std::shared_ptr<FclObject> obstacle;
  assert(ok(createSphere(0.1, Vec3(5.0, 0.0, 0.0), Mat3::Identity(), &obstacle)));
  obstacles.push_back(obstacle);

  const std::shared_ptr<FclObject> grasped_object = nullptr;

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

static void test_plane() {
  constexpr double tol = 1e-5;

  std::shared_ptr<FclObject> plane;
  assert(ok(createPlane(Vec3(0.0, 0.0, 1.0), 0.0, &plane)));

  std::shared_ptr<FclObject> sphere_clear;
  assert(ok(createSphere(0.1, Vec3(0.0, 0.0, 0.5), Mat3::Identity(), &sphere_clear)));
  double min_d = 0.0;
  Vec3 cp_plane = Vec3::Zero();
  Vec3 cp_sphere = Vec3::Zero();
  assert(ok(checkCollision(*plane, *sphere_clear, &min_d, &cp_plane, &cp_sphere)));
  assert(near(min_d, 0.4, tol));

  std::shared_ptr<FclObject> sphere_penetrating;
  assert(ok(createSphere(0.1, Vec3(0.0, 0.0, 0.05), Mat3::Identity(), &sphere_penetrating)));
  assert(ok(checkCollision(*plane, *sphere_penetrating, &min_d, &cp_plane, &cp_sphere)));
  assert(min_d < 0.0);
}

static void test_contact_jacobian_structure() {
  constexpr double tol = 1e-12;
  Eigen::MatrixXd spatial_jacobian(6, 3);
  spatial_jacobian << 1.0, 2.0, 3.0,
                      4.0, 5.0, 6.0,
                      7.0, 8.0, 9.0,
                      0.1, 0.2, 0.3,
                      0.4, 0.5, 0.6,
                      0.7, 0.8, 0.9;

  const Vec3 p(0.2, -0.1, 0.3);
  Eigen::MatrixXd Jc;
  assert(ok(getContactJacobian(/*link_index=*/1, p, spatial_jacobian, &Jc)));
  assert(Jc.rows() == 6);
  assert(Jc.cols() == 3);

  Eigen::MatrixXd Js = Eigen::MatrixXd::Zero(6, 3);
  Js.leftCols(2) = spatial_jacobian.leftCols(2);
  const Eigen::Matrix3d hat = (Eigen::Matrix3d() << 0.0, -p.z(), p.y(),
                                                    p.z(), 0.0, -p.x(),
                                                   -p.y(), p.x(), 0.0).finished();
  const Eigen::MatrixXd expected_top =
      (Eigen::MatrixXd(3, 6) << Eigen::Matrix3d::Identity(), -hat).finished() * Js;
  Eigen::MatrixXd expected = Eigen::MatrixXd::Zero(6, 3);
  expected.topRows(3) = expected_top;

  assert(matNear(Jc, expected, tol));
}

static void test_self_collision_last_link_special_case() {
  // dof=3 => cylinders: [base, link1, link2, link3]
  // Ignore link1 for contact outputs. Last active link is link3.
  // Place link3 near base so only the last-link special case can catch it.
  std::vector<std::shared_ptr<FclObject>> link_cylinders;
  link_cylinders.reserve(4);

  std::shared_ptr<FclObject> base;
  std::shared_ptr<FclObject> link1;
  std::shared_ptr<FclObject> link2;
  std::shared_ptr<FclObject> link3;
  assert(ok(createSphere(0.1, Vec3(0.00, 0.0, 0.0), Mat3::Identity(), &base)));
  assert(ok(createSphere(0.1, Vec3(5.00, 0.0, 0.0), Mat3::Identity(), &link1)));  // ignored
  assert(ok(createSphere(0.1, Vec3(2.00, 0.0, 0.0), Mat3::Identity(), &link2)));
  assert(ok(createSphere(0.1, Vec3(0.18, 0.0, 0.0), Mat3::Identity(), &link3)));  // penetrates base
  link_cylinders.push_back(base);
  link_cylinders.push_back(link1);
  link_cylinders.push_back(link2);
  link_cylinders.push_back(link3);

  const std::vector<std::shared_ptr<FclObject>> obstacles;
  const std::shared_ptr<FclObject> grasped_object = nullptr;

  Eigen::MatrixXd spatial_jacobian = Eigen::MatrixXd::Zero(6, 3);
  spatial_jacobian.block<3,3>(0, 0).setIdentity();

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
      /*num_links_ignore=*/1,
      /*dof=*/3,
      &contact_normal_array,
      &dist_array,
      &contact_points_array,
      &j_contact_array);
  assert(ok(st));
  assert(dist_array.size() == 2);
  assert(dist_array[1] < 0.0);
}

static void test_adjust_joints_passthrough_when_safe() {
  ContactSet contacts;
  contacts.contacts.resize(1);
  contacts.contacts[0].distance = 0.5;
  contacts.contacts[0].normal = Vec3(1.0, 0.0, 0.0);
  contacts.contacts[0].J_contact = Eigen::MatrixXd::Zero(6, 2);

  Eigen::Vector2d q_current(0.0, 0.0);
  Eigen::Vector2d q_next(0.1, -0.2);
  Eigen::VectorXd adjusted;
  assert(ok(adjustJoints(/*h=*/0.001,
                         /*safe_dist=*/0.01,
                         contacts,
                         q_current,
                         q_next,
                         &adjusted)));
  assert((adjusted - q_next).norm() <= 1e-12);
}

int main() {
  test_compute_contacts_consistency();
  test_self_collision_last_link_special_case();
  test_contact_jacobian_structure();
  test_adjust_joints_passthrough_when_safe();
  test_plane();
  std::cout << "sclerp_collision_unit_test: PASS\n";
  return 0;
}
