// Benchmark obstacle broadphase reuse for planner-style repeated environment contact queries.
//
// This measures the intended fast path: build one obstacle tree, then reuse it across many robot
// poses while keeping exact narrowphase contact extraction unchanged.
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "../src/contact_compute_internal.hpp"
#include "../src/obstacle_broadphase.hpp"

#include "sclerp/collision/collision.hpp"
#include "sclerp/collision/types.hpp"
#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/model/manipulator_model.hpp"

using sclerp::collision::CollisionContext;
using sclerp::collision::CollisionQueryOptions;
using sclerp::collision::ContactSet;
using sclerp::collision::FclObject;
using sclerp::collision::createSphere;
using sclerp::core::KinematicsSolver;
using sclerp::core::ManipulatorBuilder;
using sclerp::core::ManipulatorModel;
using sclerp::core::Mat3;
using sclerp::core::Mat4;
using sclerp::core::Status;
using sclerp::core::Transform;
using sclerp::core::Vec3;
using sclerp::core::ok;
namespace collision_detail = sclerp::collision::detail;

static Transform translation(double x, double y, double z) {
  Transform T = Transform::Identity();
  T.translation() = Vec3(x, y, z);
  return T;
}

static KinematicsSolver makePlanarChainSolver(int dof) {
  ManipulatorBuilder b;
  b.set_ee_home(translation(static_cast<double>(dof), 0.0, 0.0));
  for (int i = 0; i < dof; ++i) {
    b.add_revolute("joint" + std::to_string(i + 1),
                   Vec3::UnitZ(),
                   Vec3(static_cast<double>(i), 0.0, 0.0),
                   {},
                   translation(static_cast<double>(i + 1), 0.0, 0.0));
  }

  ManipulatorModel model;
  assert(ok(b.build(&model)));
  return KinematicsSolver(model);
}

static std::shared_ptr<FclObject> makeSphereOrDie(double radius, const Vec3& position) {
  std::shared_ptr<FclObject> out;
  assert(ok(createSphere(radius, position, Mat3::Identity(), &out)));
  return out;
}

static std::vector<std::shared_ptr<FclObject>> makeLinkMeshesOrDie(std::size_t count, double radius) {
  std::vector<std::shared_ptr<FclObject>> out;
  out.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    out.push_back(makeSphereOrDie(radius, Vec3::Zero()));
  }
  return out;
}

static std::vector<Mat4> identityOffsets(std::size_t count) {
  return std::vector<Mat4>(count, Mat4::Identity());
}

static void updateSceneOrDie(const KinematicsSolver& solver,
                             const Eigen::VectorXd& q,
                             const std::vector<std::shared_ptr<FclObject>>& link_meshes) {
  std::vector<Transform> fk_all;
  assert(ok(solver.forwardKinematicsAll(q, &fk_all)));
  assert(fk_all.size() == link_meshes.size());

  std::vector<Mat4> matrices;
  matrices.reserve(fk_all.size());
  for (const auto& T : fk_all) {
    matrices.push_back(T.matrix());
  }

  assert(ok(sclerp::collision::updateLinkMeshTransforms(
      link_meshes, matrices, identityOffsets(link_meshes.size()))));
}

static bool matNear(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, double tol) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
  if (a.size() == 0) return true;
  return (a - b).cwiseAbs().maxCoeff() <= tol;
}

static void assertContactSetsNear(const ContactSet& a, const ContactSet& b, double tol) {
  assert(a.contacts.size() == b.contacts.size());
  for (std::size_t i = 0; i < a.contacts.size(); ++i) {
    const auto& ca = a.contacts[i];
    const auto& cb = b.contacts[i];
    assert(ca.link_index == cb.link_index);
    assert(ca.is_grasped == cb.is_grasped);
    assert(std::abs(ca.distance - cb.distance) <= tol);
    assert((ca.normal - cb.normal).norm() <= tol);
    assert((ca.point_obj - cb.point_obj).norm() <= tol);
    assert((ca.point_link - cb.point_link).norm() <= tol);
    assert(matNear(ca.J_contact, cb.J_contact, tol));
  }
}

int main() {
  constexpr int kDof = 6;
  constexpr int kPoses = 250;
  constexpr double kLinkRadius = 0.07;
  constexpr double kObstacleRadius = 0.035;

  const KinematicsSolver solver = makePlanarChainSolver(kDof);
  const auto link_meshes = makeLinkMeshesOrDie(static_cast<std::size_t>(kDof + 1), kLinkRadius);

  std::vector<std::shared_ptr<FclObject>> obstacles;
  for (int ix = -4; ix <= 16; ++ix) {
    for (int iy = -8; iy <= 8; ++iy) {
      if ((ix + iy) % 3 != 0) continue;
      obstacles.push_back(
          makeSphereOrDie(kObstacleRadius, Vec3(0.35 * ix, 0.24 * iy, 0.0)));
    }
  }

  std::shared_ptr<FclObject> grasped_object;
  const CollisionContext ctx{link_meshes, obstacles, grasped_object};

  collision_detail::ObstacleBroadphaseCache cache;
  assert(ok(cache.build(obstacles)));

  CollisionQueryOptions opt;
  opt.use_obstacle_broadphase = true;

  std::uint64_t brute_ns = 0;
  std::uint64_t broad_ns = 0;
  std::size_t brute_exact_checks = 0;
  std::size_t broad_exact_checks = 0;

  for (int pose_idx = 0; pose_idx < kPoses; ++pose_idx) {
    Eigen::VectorXd q(kDof);
    for (int joint = 0; joint < kDof; ++joint) {
      q[joint] = 0.55 * std::sin(0.17 * pose_idx + 0.31 * joint);
    }

    updateSceneOrDie(solver, q, link_meshes);

    ContactSet brute_contacts;
    ContactSet broad_contacts;
    collision_detail::ComputeContactsStats brute_stats;
    collision_detail::ComputeContactsStats broad_stats;

    const auto brute_start = std::chrono::steady_clock::now();
    const Status brute_st =
        collision_detail::computeContactsImpl(solver, q, ctx, opt, nullptr, &brute_contacts, &brute_stats);
    const auto brute_end = std::chrono::steady_clock::now();
    assert(ok(brute_st));

    const auto broad_start = std::chrono::steady_clock::now();
    const Status broad_st =
        collision_detail::computeContactsImpl(solver, q, ctx, opt, &cache, &broad_contacts, &broad_stats);
    const auto broad_end = std::chrono::steady_clock::now();
    assert(ok(broad_st));

    assertContactSetsNear(brute_contacts, broad_contacts, 1e-8);

    brute_ns += static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(brute_end - brute_start).count());
    broad_ns += static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(broad_end - broad_start).count());
    brute_exact_checks += brute_stats.exact_obstacle_narrowphase_checks;
    broad_exact_checks += broad_stats.exact_obstacle_narrowphase_checks;
  }

  const double brute_ms = static_cast<double>(brute_ns) * 1e-6;
  const double broad_ms = static_cast<double>(broad_ns) * 1e-6;
  const double speedup = broad_ms > 0.0 ? brute_ms / broad_ms : 0.0;

  std::cout << "sclerp_collision_obstacle_broadphase_benchmark\n";
  std::cout << "  dof: " << kDof << "\n";
  std::cout << "  poses: " << kPoses << "\n";
  std::cout << "  obstacles: " << obstacles.size() << "\n";
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  brute_force_ms: " << brute_ms << "\n";
  std::cout << "  broadphase_ms: " << broad_ms << "\n";
  std::cout << "  speedup: " << speedup << "x\n";
  std::cout << "  brute_force_exact_checks: " << brute_exact_checks << "\n";
  std::cout << "  broadphase_exact_checks: " << broad_exact_checks << "\n";
  return 0;
}
