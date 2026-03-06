#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "../src/contact_compute_internal.hpp"
#include "../src/obstacle_broadphase.hpp"

#include "sclerp/collision/avoidance.hpp"
#include "sclerp/collision/collision.hpp"
#include "sclerp/collision/robot_link_meshes.hpp"
#include "sclerp/collision/types.hpp"
#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/model/manipulator_model.hpp"

using sclerp::collision::ContactSet;
using sclerp::collision::CollisionQueryOptions;
using sclerp::collision::CollisionContext;
using sclerp::collision::CollisionAvoidanceOptions;
using sclerp::collision::Contact;
using sclerp::collision::FclObject;
using sclerp::collision::SphereObject;
using sclerp::collision::adjustJoints;
using sclerp::collision::createSphere;
using sclerp::collision::createPlane;
using sclerp::collision::computeContacts;
using sclerp::collision::checkCollision;
using sclerp::collision::RobotLinkMeshSpec;
using sclerp::collision::buildRobotLinkMeshesFromStlDirectory;
using sclerp::core::LogLevel;
using sclerp::core::LogSink;
using sclerp::core::Status;
using sclerp::core::KinematicsSolver;
using sclerp::core::ManipulatorBuilder;
using sclerp::core::ManipulatorModel;
using sclerp::core::Transform;
using sclerp::core::ok;
using sclerp::core::Vec3;
using sclerp::core::Mat3;
using sclerp::core::Mat4;
namespace collision_detail = sclerp::collision::detail;

namespace {

void noopLogSink(LogLevel, const std::string&) {}

class ScopedLogSilencer {
public:
  ScopedLogSilencer()
      : level_(sclerp::core::getLogLevel()),
        sink_(sclerp::core::getLogSink()) {
    sclerp::core::setLogSink(&noopLogSink);
  }

  ~ScopedLogSilencer() {
    sclerp::core::setLogLevel(level_);
    sclerp::core::setLogSink(sink_);
  }

private:
  LogLevel level_;
  LogSink sink_;
};

}  // namespace

[[maybe_unused]] static bool near(double a, double b, double tol) {
  return std::abs(a - b) <= tol;
}

[[maybe_unused]] static bool matNear(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, double tol) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
  if (a.size() == 0) return true;
  return (a - b).cwiseAbs().maxCoeff() <= tol;
}

static Transform translation(double x, double y, double z) {
  Transform T = Transform::Identity();
  T.translation() = Vec3(x, y, z);
  return T;
}

static KinematicsSolver makePlanarTwoLinkSolver(bool has_tool_frame = false) {
  ManipulatorBuilder b;
  b.set_ee_home(has_tool_frame ? translation(2.4, 0.0, 0.0) : translation(2.0, 0.0, 0.0));
  b.add_revolute("joint1", Vec3::UnitZ(), Vec3::Zero(), {}, translation(1.0, 0.0, 0.0));
  b.add_revolute("joint2", Vec3::UnitZ(), Vec3(1.0, 0.0, 0.0), {}, translation(2.0, 0.0, 0.0));

  ManipulatorModel model;
  assert(ok(b.build(&model)));
  model.set_has_tool_frame(has_tool_frame);
  return KinematicsSolver(model);
}

static std::shared_ptr<FclObject> makeSphereOrDie(double radius, const Vec3& position) {
  std::shared_ptr<FclObject> out;
  assert(ok(createSphere(radius, position, Mat3::Identity(), &out)));
  assert(out);
  return out;
}

static std::vector<std::shared_ptr<FclObject>> makeLinkMeshesOrDie(std::size_t count, double radius) {
  std::vector<std::shared_ptr<FclObject>> link_meshes;
  link_meshes.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    link_meshes.push_back(makeSphereOrDie(radius, Vec3::Zero()));
  }
  return link_meshes;
}

static std::vector<Mat4> identityOffsets(std::size_t count) {
  return std::vector<Mat4>(count, Mat4::Identity());
}

static std::vector<Mat4> fkAllMatricesOrDie(const KinematicsSolver& solver, const Eigen::VectorXd& q) {
  std::vector<Transform> fk_all;
  assert(ok(solver.forwardKinematicsAll(q, &fk_all)));

  std::vector<Mat4> out;
  out.reserve(fk_all.size());
  for (const auto& T : fk_all) {
    out.push_back(T.matrix());
  }
  return out;
}

static void updateSceneOrDie(const KinematicsSolver& solver,
                             const Eigen::VectorXd& q,
                             const std::vector<std::shared_ptr<FclObject>>& link_meshes,
                             const std::shared_ptr<FclObject>& grasped_object = nullptr) {
  const std::vector<Mat4> fk_all = fkAllMatricesOrDie(solver, q);
  assert(fk_all.size() == link_meshes.size());
  assert(ok(sclerp::collision::updateLinkMeshTransforms(
      link_meshes, fk_all, identityOffsets(link_meshes.size()))));

  if (grasped_object) {
    const Mat4& g_attach = fk_all.back();
    grasped_object->setTransform(g_attach.block<3,1>(0, 3), g_attach.block<3,3>(0, 0));
  }
}

static void assertContactSetsNear(const ContactSet& a, const ContactSet& b, double tol) {
  assert(a.contacts.size() == b.contacts.size());
  for (std::size_t i = 0; i < a.contacts.size(); ++i) {
    const auto& ca = a.contacts[i];
    const auto& cb = b.contacts[i];
    assert(ca.link_index == cb.link_index);
    assert(ca.is_grasped == cb.is_grasped);
    assert(near(ca.distance, cb.distance, tol));
    assert((ca.normal - cb.normal).norm() <= tol);
    assert((ca.point_obj - cb.point_obj).norm() <= tol);
    assert((ca.point_link - cb.point_link).norm() <= tol);
    assert(matNear(ca.J_contact, cb.J_contact, tol));
  }
}

static void test_compute_contacts_basic_scene() {
  const KinematicsSolver solver = makePlanarTwoLinkSolver();
  Eigen::Vector2d q;
  q << 0.0, 0.35;

  const auto link_meshes = makeLinkMeshesOrDie(3, 0.08);
  updateSceneOrDie(solver, q, link_meshes);

  std::vector<std::shared_ptr<FclObject>> obstacles;
  obstacles.push_back(makeSphereOrDie(0.06, Vec3(1.24, 0.10, 0.0)));
  obstacles.push_back(makeSphereOrDie(0.06, Vec3(1.98, 0.42, 0.0)));
  obstacles.push_back(makeSphereOrDie(0.06, Vec3(-0.80, 1.10, 0.0)));

  std::shared_ptr<FclObject> grasped_object;
  const CollisionContext ctx{link_meshes, obstacles, grasped_object};

  CollisionQueryOptions brute_opt;
  brute_opt.use_obstacle_broadphase = false;
  CollisionQueryOptions broad_opt = brute_opt;
  broad_opt.use_obstacle_broadphase = true;

  ContactSet brute_contacts;
  ContactSet broad_contacts;
  assert(ok(computeContacts(solver, q, ctx, brute_opt, &brute_contacts)));
  assert(ok(computeContacts(solver, q, ctx, broad_opt, &broad_contacts)));
  assertContactSetsNear(brute_contacts, broad_contacts, 1e-8);
}

static void test_compute_contacts_no_obstacles_broadphase_equivalence() {
  const KinematicsSolver solver = makePlanarTwoLinkSolver();
  Eigen::Vector2d q;
  q << 0.2, -0.4;

  const auto link_meshes = makeLinkMeshesOrDie(3, 0.08);
  updateSceneOrDie(solver, q, link_meshes);

  std::vector<std::shared_ptr<FclObject>> obstacles;
  std::shared_ptr<FclObject> grasped_object;
  const CollisionContext ctx{link_meshes, obstacles, grasped_object};

  CollisionQueryOptions brute_opt;
  brute_opt.use_obstacle_broadphase = false;
  CollisionQueryOptions broad_opt = brute_opt;
  broad_opt.use_obstacle_broadphase = true;

  ContactSet brute_contacts;
  ContactSet broad_contacts;
  assert(ok(computeContacts(solver, q, ctx, brute_opt, &brute_contacts)));
  assert(ok(computeContacts(solver, q, ctx, broad_opt, &broad_contacts)));
  assertContactSetsNear(brute_contacts, broad_contacts, 1e-12);
}

static void test_compute_contacts_plane_obstacle_broadphase_equivalence() {
  const KinematicsSolver solver = makePlanarTwoLinkSolver();
  Eigen::Vector2d q;
  q << 0.1, -0.3;

  const auto link_meshes = makeLinkMeshesOrDie(3, 0.08);
  updateSceneOrDie(solver, q, link_meshes);

  std::shared_ptr<FclObject> plane;
  assert(ok(createPlane(Vec3::UnitZ(), -0.5, &plane)));

  std::vector<std::shared_ptr<FclObject>> obstacles{plane};
  std::shared_ptr<FclObject> grasped_object;
  const CollisionContext ctx{link_meshes, obstacles, grasped_object};

  CollisionQueryOptions brute_opt;
  brute_opt.use_obstacle_broadphase = false;
  CollisionQueryOptions broad_opt = brute_opt;
  broad_opt.use_obstacle_broadphase = true;

  ContactSet brute_contacts;
  ContactSet broad_contacts;
  assert(ok(computeContacts(solver, q, ctx, brute_opt, &brute_contacts)));
  assert(ok(computeContacts(solver, q, ctx, broad_opt, &broad_contacts)));
  assertContactSetsNear(brute_contacts, broad_contacts, 1e-8);
}

static void test_plane() {
  std::shared_ptr<FclObject> plane;
  std::shared_ptr<FclObject> sphere;
  assert(ok(createPlane(Vec3::UnitZ(), 0.0, &plane)));
  assert(ok(createSphere(0.2, Vec3(0.0, 0.0, 1.0), Mat3::Identity(), &sphere)));

  double dist = 0.0;
  Vec3 p_plane = Vec3::Zero();
  Vec3 p_sphere = Vec3::Zero();
  assert(ok(checkCollision(*plane, *sphere, &dist, &p_plane, &p_sphere)));
  assert(dist >= 0.0);
}

static void test_contact_jacobian_structure() {
  const KinematicsSolver solver = makePlanarTwoLinkSolver();
  Eigen::Vector2d q;
  q << 0.0, 0.25;

  const auto link_meshes = makeLinkMeshesOrDie(3, 0.08);
  updateSceneOrDie(solver, q, link_meshes);

  std::vector<std::shared_ptr<FclObject>> obstacles;
  obstacles.push_back(makeSphereOrDie(0.06, Vec3(1.18, 0.06, 0.0)));
  obstacles.push_back(makeSphereOrDie(0.06, Vec3(2.06, 0.26, 0.0)));

  std::shared_ptr<FclObject> grasped_object;
  const CollisionContext ctx{link_meshes, obstacles, grasped_object};

  CollisionQueryOptions opt;
  opt.use_obstacle_broadphase = true;

  ContactSet contacts;
  assert(ok(computeContacts(solver, q, ctx, opt, &contacts)));
  assert(contacts.contacts.size() == 2);
  for (const auto& c : contacts.contacts) {
    assert(c.J_contact.rows() == 3);
    assert(c.J_contact.cols() == c.link_index + 1);
  }
}

static void test_self_collision_last_link_special_case() {
  const KinematicsSolver solver = makePlanarTwoLinkSolver();
  Eigen::Vector2d q;
  q << 0.0, 0.0;

  const auto link_meshes = makeLinkMeshesOrDie(3, 0.20);
  updateSceneOrDie(solver, q, link_meshes);

  std::vector<std::shared_ptr<FclObject>> obstacles;
  std::shared_ptr<FclObject> grasped_object;
  const CollisionContext ctx{link_meshes, obstacles, grasped_object};

  CollisionQueryOptions opt;
  opt.check_self_collision = true;
  opt.use_obstacle_broadphase = true;

  ContactSet contacts;
  assert(ok(computeContacts(solver, q, ctx, opt, &contacts)));
  assert(contacts.contacts.size() == 2);
}

static void test_compute_contacts_grasped_object_broadphase_equivalence() {
  const KinematicsSolver solver = makePlanarTwoLinkSolver(true);
  Eigen::Vector2d q;
  q << 0.1, -0.2;

  const auto link_meshes = makeLinkMeshesOrDie(4, 0.08);
  std::shared_ptr<FclObject> grasped_object = makeSphereOrDie(0.05, Vec3::Zero());
  updateSceneOrDie(solver, q, link_meshes, grasped_object);

  const Vec3 tool_pos = fkAllMatricesOrDie(solver, q).back().block<3,1>(0, 3);
  std::vector<std::shared_ptr<FclObject>> obstacles;
  obstacles.push_back(makeSphereOrDie(0.05, tool_pos + Vec3(0.14, 0.03, 0.0)));
  obstacles.push_back(makeSphereOrDie(0.05, Vec3(1.10, -0.20, 0.0)));

  const CollisionContext ctx{link_meshes, obstacles, grasped_object};

  CollisionQueryOptions brute_opt;
  brute_opt.use_obstacle_broadphase = false;
  CollisionQueryOptions broad_opt = brute_opt;
  broad_opt.use_obstacle_broadphase = true;

  ContactSet brute_contacts;
  ContactSet broad_contacts;
  assert(ok(computeContacts(solver, q, ctx, brute_opt, &brute_contacts)));
  assert(ok(computeContacts(solver, q, ctx, broad_opt, &broad_contacts)));
  assertContactSetsNear(brute_contacts, broad_contacts, 1e-8);
  assert(brute_contacts.contacts.size() == 3);
  assert(brute_contacts.contacts.back().is_grasped);
}

static void test_adjust_joints_passthrough_when_safe() {
  const ContactSet contacts;
  Eigen::Vector2d q_curr;
  q_curr << 0.0, 0.0;
  Eigen::Vector2d q_next;
  q_next << 0.1, -0.2;

  Eigen::VectorXd adjusted;
  assert(ok(adjustJoints(CollisionAvoidanceOptions{}, contacts, q_curr, q_next, &adjusted)));
  assert(adjusted.isApprox(q_next));
}

static void test_adjust_joints_invalid_options_rejected() {
  Contact c;
  c.link_index = 0;
  c.distance = 0.001;
  c.normal = Vec3::UnitX();
  c.J_contact = Eigen::MatrixXd::Zero(3, 1);
  ContactSet contacts;
  contacts.contacts.push_back(c);

  CollisionAvoidanceOptions opt;
  opt.dt = 0.0;

  Eigen::VectorXd adjusted;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
  assert(adjustJoints(opt, contacts, q, q, &adjusted) == Status::InvalidParameter);
}

static void test_null_obstacle_rejected() {
  const KinematicsSolver solver = makePlanarTwoLinkSolver();
  Eigen::Vector2d q;
  q << 0.0, 0.0;

  const auto link_meshes = makeLinkMeshesOrDie(3, 0.08);
  updateSceneOrDie(solver, q, link_meshes);

  std::vector<std::shared_ptr<FclObject>> obstacles{nullptr};
  std::shared_ptr<FclObject> grasped_object;
  const CollisionContext ctx{link_meshes, obstacles, grasped_object};

  CollisionQueryOptions opt;
  opt.use_obstacle_broadphase = true;

  ContactSet contacts;
  {
    ScopedLogSilencer silence_expected_error;
    assert(computeContacts(solver, q, ctx, opt, &contacts) == Status::InvalidParameter);
  }
}

static void test_broadphase_cache_handles_moved_query_objects() {
  const KinematicsSolver solver = makePlanarTwoLinkSolver();
  const auto link_meshes = makeLinkMeshesOrDie(3, 0.08);

  std::vector<std::shared_ptr<FclObject>> obstacles;
  obstacles.push_back(makeSphereOrDie(0.06, Vec3(2.22, 0.00, 0.0)));
  obstacles.push_back(makeSphereOrDie(0.06, Vec3(0.00, 2.22, 0.0)));

  std::shared_ptr<FclObject> grasped_object;
  const CollisionContext ctx{link_meshes, obstacles, grasped_object};

  collision_detail::ObstacleBroadphaseCache cache;
  assert(ok(cache.build(obstacles)));

  CollisionQueryOptions opt;
  opt.use_obstacle_broadphase = true;

  Eigen::Vector2d q1;
  q1 << 0.0, 0.0;
  updateSceneOrDie(solver, q1, link_meshes);

  ContactSet cached_q1;
  ContactSet brute_q1;
  assert(ok(collision_detail::computeContactsImpl(solver, q1, ctx, opt, &cache, &cached_q1)));
  assert(ok(collision_detail::computeContactsImpl(solver, q1, ctx, opt, nullptr, &brute_q1)));
  assertContactSetsNear(cached_q1, brute_q1, 1e-8);

  const double kPi = std::acos(-1.0);
  Eigen::Vector2d q2;
  q2 << 0.5 * kPi, 0.0;
  updateSceneOrDie(solver, q2, link_meshes);

  ContactSet cached_q2;
  ContactSet brute_q2;
  assert(ok(collision_detail::computeContactsImpl(solver, q2, ctx, opt, &cache, &cached_q2)));
  assert(ok(collision_detail::computeContactsImpl(solver, q2, ctx, opt, nullptr, &brute_q2)));
  assertContactSetsNear(cached_q2, brute_q2, 1e-8);
}

static std::string minimalAsciiStl() {
  // Smallest practical triangle STL Assimp reliably loads.
  return "solid tri\n"
         "facet normal 0 0 1\n"
         "  outer loop\n"
         "    vertex 0 0 0\n"
         "    vertex 1 0 0\n"
         "    vertex 0 1 0\n"
         "  endloop\n"
         "endfacet\n"
         "endsolid tri\n";
}

static std::filesystem::path makeTempDirOrDie() {
  namespace fs = std::filesystem;
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  fs::path p = fs::temp_directory_path() / ("sclerp_collision_unit_test_" + std::to_string(now));
  fs::create_directories(p);
  return p;
}

static void writeTextFileOrDie(const std::filesystem::path& path, const std::string& text) {
  std::ofstream out(path);
  assert(out);
  out << text;
  out.close();
  assert(out);
}

static void test_build_robot_link_meshes_from_stl_directory_basic() {
  namespace fs = std::filesystem;

  const fs::path tmp = makeTempDirOrDie();
  struct Cleanup {
    fs::path p;
    ~Cleanup() {
      std::error_code ec;
      fs::remove_all(p, ec);
    }
  } cleanup{tmp};

  const std::string stl = minimalAsciiStl();
  writeTextFileOrDie(tmp / "base_link.stl", stl);
  writeTextFileOrDie(tmp / "link1.stl", stl);

  ManipulatorBuilder b;
  b.set_ee_home(Transform::Identity());
  b.add_revolute("joint1", Vec3::UnitZ(), Vec3::Zero());

  ManipulatorModel model;
  assert(ok(b.build(&model)));

  const KinematicsSolver solver(model);

  const RobotLinkMeshSpec spec{std::vector<std::string>{"base_link", "link1"}, {}};
  std::vector<std::shared_ptr<FclObject>> link_meshes;
  std::vector<Mat4> mesh_offsets;
  const Status st = buildRobotLinkMeshesFromStlDirectory(solver, tmp, spec, &link_meshes, &mesh_offsets);
  assert(ok(st));
  assert(link_meshes.size() == 2);
  assert(mesh_offsets.size() == 2);
  assert(link_meshes[0]);
  assert(link_meshes[1]);
  assert(mesh_offsets[0].isApprox(Mat4::Identity()));
  assert(mesh_offsets[1].isApprox(Mat4::Identity()));
}

static void test_build_robot_link_meshes_from_stl_directory_tool_placeholder() {
  namespace fs = std::filesystem;

  const fs::path tmp = makeTempDirOrDie();
  struct Cleanup {
    fs::path p;
    ~Cleanup() {
      std::error_code ec;
      fs::remove_all(p, ec);
    }
  } cleanup{tmp};

  const std::string stl = minimalAsciiStl();
  writeTextFileOrDie(tmp / "base_link.stl", stl);
  writeTextFileOrDie(tmp / "link1.stl", stl);
  // No tool STL file; the builder should fall back to a placeholder sphere.

  ManipulatorBuilder b;
  b.set_ee_home(Transform::Identity());
  b.add_revolute("joint1", Vec3::UnitZ(), Vec3::Zero());

  ManipulatorModel model;
  assert(ok(b.build(&model)));
  model.set_has_tool_frame(true);

  const KinematicsSolver solver(model);

  const RobotLinkMeshSpec spec{std::vector<std::string>{"base_link", "link1", "tool"}, {}};
  std::vector<std::shared_ptr<FclObject>> link_meshes;
  std::vector<Mat4> mesh_offsets;
  const Status st = buildRobotLinkMeshesFromStlDirectory(solver, tmp, spec, &link_meshes, &mesh_offsets);
  assert(ok(st));
  assert(link_meshes.size() == 3);
  assert(mesh_offsets.size() == 3);
  assert(link_meshes[0]);
  assert(link_meshes[1]);
  assert(link_meshes[2]);
  assert(dynamic_cast<const SphereObject*>(link_meshes[2].get()) != nullptr);
}

int main() {
  test_compute_contacts_basic_scene();
  test_compute_contacts_no_obstacles_broadphase_equivalence();
  test_compute_contacts_plane_obstacle_broadphase_equivalence();
  test_self_collision_last_link_special_case();
  test_contact_jacobian_structure();
  test_compute_contacts_grasped_object_broadphase_equivalence();
  test_broadphase_cache_handles_moved_query_objects();
  test_adjust_joints_passthrough_when_safe();
  test_adjust_joints_invalid_options_rejected();
  test_null_obstacle_rejected();
  test_plane();
  test_build_robot_link_meshes_from_stl_directory_basic();
  test_build_robot_link_meshes_from_stl_directory_tool_placeholder();
  std::cout << "sclerp_collision_unit_test: PASS\n";
  return 0;
}
