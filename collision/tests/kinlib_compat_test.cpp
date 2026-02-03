#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "sclerp/collision/collision_utils.hpp"
#include "sclerp/collision/avoidance.hpp"
#include "sclerp/core/common/status.hpp"

#include "kinlib/collision_utils.h"
#include "kinlib/kinlib.h"
#include "kinlib/kinlib_kinematics.h"

namespace sc = sclerp::collision;
namespace kc = CollisionUtils;

using sclerp::core::Status;
using sclerp::core::ok;
using sclerp::core::Vec3;
using sclerp::core::Mat3;

static bool near(double a, double b, double tol) {
  return std::abs(a - b) <= tol;
}

static bool vecNear(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol) {
  return (a - b).norm() <= tol;
}

static bool vecNear(const Eigen::VectorXd& a, const Eigen::VectorXd& b, double tol) {
  if (a.size() != b.size()) return false;
  if (a.size() == 0) return true;
  return (a - b).norm() <= tol;
}

static bool matNear(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, double tol) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
  if (a.size() == 0) return true;
  return (a - b).cwiseAbs().maxCoeff() <= tol;
}

int main() {
  constexpr double tol = 1e-6;

  // Build a minimal scene: base + 2 links, 1 obstacle.
  std::vector<std::shared_ptr<sc::ObstacleBase>> sc_links;
  std::vector<std::shared_ptr<kc::ObstacleBase>> kc_links;

  std::shared_ptr<sc::ObstacleBase> sc_base;
  assert(ok(sc::createSphere(0.1, Vec3(0.0, 0.0, 0.0), Mat3::Identity(), &sc_base)));
  sc_links.push_back(sc_base);

  std::shared_ptr<sc::ObstacleBase> sc_link1;
  assert(ok(sc::createSphere(0.1, Vec3(1.0, 0.0, 0.0), Mat3::Identity(), &sc_link1)));
  sc_links.push_back(sc_link1);

  std::shared_ptr<sc::ObstacleBase> sc_link2;
  assert(ok(sc::createSphere(0.1, Vec3(2.0, 0.0, 0.0), Mat3::Identity(), &sc_link2)));
  sc_links.push_back(sc_link2);

  kc_links.push_back(kc::createSphere(0.1, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Matrix3d::Identity()));
  kc_links.push_back(kc::createSphere(0.1, Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Matrix3d::Identity()));
  kc_links.push_back(kc::createSphere(0.1, Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Matrix3d::Identity()));

  std::vector<std::shared_ptr<sc::ObstacleBase>> sc_obstacles;
  std::vector<std::shared_ptr<kc::ObstacleBase>> kc_obstacles;

  std::shared_ptr<sc::ObstacleBase> sc_obstacle;
  assert(ok(sc::createSphere(0.1, Vec3(1.5, 0.2, 0.0), Mat3::Identity(), &sc_obstacle)));
  sc_obstacles.push_back(sc_obstacle);
  kc_obstacles.push_back(kc::createSphere(0.1, Eigen::Vector3d(1.5, 0.2, 0.0), Eigen::Matrix3d::Identity()));

  const std::shared_ptr<sc::ObstacleBase> sc_grasped = nullptr;
  const std::shared_ptr<kc::ObstacleBase> kc_grasped = nullptr;

  // Spatial Jacobian (6 x dof)
  const int dof = 2;
  Eigen::MatrixXd spatial_jacobian(6, dof);
  spatial_jacobian << 1, 2,
                      3, 4,
                      5, 6,
                      7, 8,
                      9, 10,
                      11, 12;

  // sclerp outputs
  Eigen::MatrixXd sc_contact_normal;
  std::vector<double> sc_dist;
  std::vector<Eigen::MatrixXd> sc_contact_points;
  std::vector<Eigen::MatrixXd> sc_j_contacts;
  const Status sc_st = sc::getCollisionInfo(
      sc_links,
      sc_obstacles,
      sc_grasped,
      spatial_jacobian,
      /*check_self_collision=*/false,
      /*num_links_ignore=*/0,
      dof,
      &sc_contact_normal,
      &sc_dist,
      &sc_contact_points,
      &sc_j_contacts);
  assert(ok(sc_st));

  // kinlib outputs
  Eigen::MatrixXd kc_contact_normal;
  std::vector<double> kc_dist;
  std::vector<Eigen::MatrixXd> kc_contact_points;
  std::vector<Eigen::MatrixXd> kc_j_contacts;
  const auto kc_st = kc::getCollisionInfo(
      kc_links,
      kc_obstacles,
      kc_grasped,
      spatial_jacobian,
      /*check_self_collision=*/false,
      /*num_links_ignore=*/0,
      dof,
      kc_contact_normal,
      kc_dist,
      kc_contact_points,
      kc_j_contacts);
  assert(kc_st == kinlib::OPERATION_SUCCESS);

  assert(sc_dist.size() == kc_dist.size());
  for (size_t i = 0; i < sc_dist.size(); ++i) {
    assert(near(sc_dist[i], kc_dist[i], tol));

    const Eigen::Vector3d sc_n = sc_contact_normal.block<3,1>(0, static_cast<int>(i));
    const Eigen::Vector3d kc_n = kc_contact_normal.block<3,1>(0, static_cast<int>(i));
    assert(vecNear(sc_n, kc_n, tol));

    if (i < sc_contact_points.size() && i < kc_contact_points.size()) {
      if (sc_contact_points[i].rows() == 3 && sc_contact_points[i].cols() == 2 &&
          kc_contact_points[i].rows() == 3 && kc_contact_points[i].cols() == 2) {
        const Eigen::Vector3d sc_p0 = sc_contact_points[i].col(0);
        const Eigen::Vector3d kc_p0 = kc_contact_points[i].col(0);
        const Eigen::Vector3d sc_p1 = sc_contact_points[i].col(1);
        const Eigen::Vector3d kc_p1 = kc_contact_points[i].col(1);
        assert(vecNear(sc_p0, kc_p0, tol));
        assert(vecNear(sc_p1, kc_p1, tol));
      }
    }

    if (i < sc_j_contacts.size() && i < kc_j_contacts.size()) {
      assert(matNear(sc_j_contacts[i], kc_j_contacts[i], tol));
    }
  }

  // Compare adjustJoints outputs.
  const double h = 0.1;
  double safe_dist = 0.0;
  if (!sc_dist.empty()) {
    safe_dist = sc_dist[0] + 0.05;
  }
  Eigen::VectorXd q_current(dof);
  Eigen::VectorXd q_next(dof);
  q_current.setZero();
  q_next << 0.1, -0.05;

  Eigen::VectorXd sc_adjusted;
  const Status adj_st = sc::adjustJoints(
      h,
      sc_dist,
      sc_contact_normal,
      safe_dist,
      q_current,
      q_next,
      sc_j_contacts,
      &sc_adjusted);
  assert(ok(adj_st));

  kinlib::KinematicsSolver ks;
  const Eigen::VectorXd kc_adjusted = ks.getAdjustedJoints(
      h,
      kc_dist,
      kc_contact_normal,
      safe_dist,
      q_current,
      q_next,
      kc_j_contacts);

  assert(vecNear(sc_adjusted, kc_adjusted, 1e-5));

  std::cout << "sclerp_collision_kinlib_compat_test: PASS\n";
  return 0;
}
