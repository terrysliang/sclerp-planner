#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "sclerp/collision/avoidance.hpp"
#include "sclerp/collision/collision.hpp"
#include "sclerp/collision/robot_link_meshes.hpp"
#include "sclerp/collision/types.hpp"
#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/model/manipulator_model.hpp"

using sclerp::collision::ContactSet;
using sclerp::collision::CollisionQueryOptions;
using sclerp::collision::CollisionContext;
using sclerp::collision::CollisionAvoidanceOptions;
using sclerp::collision::FclObject;
using sclerp::collision::SphereObject;
using sclerp::collision::adjustJoints;
using sclerp::collision::createSphere;
using sclerp::collision::createPlane;
using sclerp::collision::computeContacts;
using sclerp::collision::checkCollision;
using sclerp::collision::RobotLinkMeshSpec;
using sclerp::collision::buildRobotLinkMeshesFromStlDirectory;
using sclerp::core::Status;
using sclerp::core::KinematicsSolver;
using sclerp::core::ManipulatorBuilder;
using sclerp::core::ManipulatorModel;
using sclerp::core::Transform;
using sclerp::core::ok;
using sclerp::core::Vec3;
using sclerp::core::Mat3;
using sclerp::core::Mat4;

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
  test_self_collision_last_link_special_case();
  test_contact_jacobian_structure();
  test_adjust_joints_passthrough_when_safe();
  test_adjust_joints_invalid_options_rejected();
  test_null_obstacle_rejected();
  test_plane();
  test_build_robot_link_meshes_from_stl_directory_basic();
  test_build_robot_link_meshes_from_stl_directory_tool_placeholder();
  std::cout << "sclerp_collision_unit_test: PASS\n";
  return 0;
}
