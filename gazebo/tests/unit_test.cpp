#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "sclerp/collision/collision.hpp"
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/path/joint_path.hpp"
#include "sclerp/gazebo/obstacle_registry.hpp"

using sclerp::collision::createBox;
using sclerp::collision::createCylinder;
using sclerp::collision::createMeshFromSTL;
using sclerp::collision::createPlane;
using sclerp::collision::createSphere;
using sclerp::core::Mat3;
using sclerp::core::Mat4;
using sclerp::core::Status;
using sclerp::core::Vec3;
using sclerp::core::ok;
using sclerp::gazebo::ObstacleRegistry;
using sclerp::gazebo::RobotModelFromUrdf;
using sclerp::gazebo::WorldExportOptions;
using sclerp::gazebo::writeJointTrajectoryCsv;

static std::string slurp(const std::string& path) {
  std::ifstream in(path);
  assert(in && "failed to open file");
  std::string s((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return s;
}

static void writeAsciiStlTriangle(const std::string& path) {
  std::ofstream out(path);
  assert(out && "failed to create stl file");
  out << "solid tri\n";
  out << "  facet normal 0 0 1\n";
  out << "    outer loop\n";
  out << "      vertex 0 0 0\n";
  out << "      vertex 1 0 0\n";
  out << "      vertex 0 1 0\n";
  out << "    endloop\n";
  out << "  endfacet\n";
  out << "endsolid tri\n";
}

static void test_exports() {
  ObstacleRegistry reg;

  std::shared_ptr<sclerp::collision::FclObject> box;
  assert(ok(createBox(Vec3(1.0, 2.0, 3.0), Vec3(0.1, 0.2, 0.3), Mat3::Identity(), &box)));
  assert(ok(reg.registerObstacle(box, "box")));

  std::shared_ptr<sclerp::collision::FclObject> sphere;
  assert(ok(createSphere(0.5, Vec3(1.0, 0.0, 0.0), Mat3::Identity(), &sphere)));
  assert(ok(reg.registerObstacle(sphere, "sphere")));

  std::shared_ptr<sclerp::collision::FclObject> cyl;
  assert(ok(createCylinder(0.2, 1.0, Vec3(0.0, 1.0, 0.0), Mat3::Identity(), &cyl)));
  assert(ok(reg.registerObstacle(cyl, "cylinder")));

  std::shared_ptr<sclerp::collision::FclObject> plane;
  assert(ok(createPlane(Vec3(0.0, 0.0, 1.0), 0.0, &plane)));
  assert(ok(reg.registerObstacle(plane, "plane")));

  const std::string stl_path = "sclerp_gazebo_test_mesh.stl";
  writeAsciiStlTriangle(stl_path);

  std::shared_ptr<sclerp::collision::FclObject> mesh;
  assert(ok(createMeshFromSTL(stl_path, Mat4::Identity(), &mesh)));
  assert(ok(reg.registerObstacle(mesh, "mesh")));

  const std::string json_path = "sclerp_gazebo_test_scene.json";
  assert(ok(reg.writeJson(json_path)));
  const std::string json = slurp(json_path);
  assert(json.find("\"obstacles\"") != std::string::npos);
  assert(json.find("\"type\": \"box\"") != std::string::npos);
  assert(json.find("\"type\": \"mesh\"") != std::string::npos);
  assert(json.find(stl_path) != std::string::npos);

  const std::string sdf_path = "sclerp_gazebo_test_world.sdf";
  WorldExportOptions opt;
  opt.include_ground_plane = false;
  opt.include_sun = false;
  assert(ok(reg.writeSdfWorld(sdf_path, opt)));
  const std::string sdf = slurp(sdf_path);
  assert(sdf.find("<sdf version=\"1.7\">") != std::string::npos);
  assert(sdf.find("<model name=\"box\">") != std::string::npos);
  assert(sdf.find("<mesh>") != std::string::npos);
  assert(sdf.find(stl_path) != std::string::npos);

  // Cleanup
  std::remove(stl_path.c_str());
  std::remove(json_path.c_str());
  std::remove(sdf_path.c_str());
}

static void test_trajectory_csv() {
  sclerp::core::JointPath path;
  path.joint_names = {"j1", "j2"};
  path.positions.push_back((Eigen::Vector2d() << 0.0, 1.0).finished());
  path.positions.push_back((Eigen::Vector2d() << 0.1, 1.1).finished());

  const std::string csv_path = "sclerp_gazebo_test_traj.csv";
  assert(ok(writeJointTrajectoryCsv(path, 0.05, csv_path)));
  const std::string csv = slurp(csv_path);
  assert(csv.rfind("time,j1,j2\n", 0) == 0);
  std::remove(csv_path.c_str());
}

static void test_robot_from_urdf_inline_export() {
  // Minimal 1-DOF chain with a fixed joint after the last movable joint (tool frame).
  const std::string urdf_path = "sclerp_gazebo_test_robot.urdf";
  {
    std::ofstream out(urdf_path);
    assert(out && "failed to create URDF file");
    out << R"(
<robot name="test_robot">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="tool0"/>

  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>

  <joint name="tool_fixed" type="fixed">
    <parent link="link1"/>
    <child link="tool0"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
  </joint>
</robot>
)";
  }

  const std::string stl_base = "sclerp_gazebo_test_robot_base.stl";
  const std::string stl_link1 = "sclerp_gazebo_test_robot_link1.stl";
  const std::string stl_tool = "sclerp_gazebo_test_robot_tool.stl";
  writeAsciiStlTriangle(stl_base);
  writeAsciiStlTriangle(stl_link1);
  writeAsciiStlTriangle(stl_tool);

  ObstacleRegistry reg;

  WorldExportOptions opt;
  opt.include_ground_plane = false;
  opt.include_sun = false;

  RobotModelFromUrdf robot;
  robot.name = "robot";
  robot.urdf_path = urdf_path;
  robot.base_link = "base_link";
  robot.tip_link = "tool0";
  robot.collapse_fixed_joints = true;
  robot.link_mesh_stl_files = {stl_base, stl_link1, stl_tool};

  opt.robot_from_urdf = robot;

  // Also validate plugin stanza wiring.
  opt.joint_trajectory = sclerp::gazebo::JointTrajectoryPlayback{};
  opt.joint_trajectory->csv_path = "traj.csv";
  opt.joint_trajectory->loop = false;
  opt.joint_trajectory->rate = 1.0;

  opt.world_sdf_saver = sclerp::gazebo::WorldSdfSaverOptions{};
  opt.world_sdf_saver->output_path = "world_saved.sdf";

  opt.primitive_resizer = sclerp::gazebo::PrimitiveResizerOptions{};

  const std::string sdf_path = "sclerp_gazebo_test_world_with_robot.sdf";
  assert(ok(reg.writeSdfWorld(sdf_path, opt)));
  const std::string sdf = slurp(sdf_path);

  assert(sdf.find("<model name=\"robot\">") != std::string::npos);
  assert(sdf.find("<static>true</static>") != std::string::npos);
  assert(sdf.find("<joint name=\"j1\" type=\"revolute\">") != std::string::npos);
  assert(sdf.find("<joint name=\"tool_fixed\" type=\"fixed\">") != std::string::npos);
  assert(sdf.find(stl_base) != std::string::npos);
  assert(sdf.find(stl_tool) != std::string::npos);
  assert(sdf.find("<robot_model>robot</robot_model>") != std::string::npos);
  assert(sdf.find("libsclerp_world_sdf_saver.so") != std::string::npos);
  assert(sdf.find("<output_path>world_saved.sdf</output_path>") != std::string::npos);
  assert(sdf.find("libsclerp_primitive_resizer.so") != std::string::npos);
  assert(sdf.find("libignition-gazebo-physics-system.so") != std::string::npos);
  assert(sdf.find("gz::sim::systems::SceneBroadcaster") != std::string::npos);

  std::remove(urdf_path.c_str());
  std::remove(stl_base.c_str());
  std::remove(stl_link1.c_str());
  std::remove(stl_tool.c_str());
  std::remove(sdf_path.c_str());
}

static void test_robot_from_urdf_mesh_dir_auto_discovery() {
  // Same chain as the inline export test, but rely on `link_mesh_stl_directory` auto-discovery.
  const std::string urdf_path = "sclerp_gazebo_test_robot_auto_mesh.urdf";
  {
    std::ofstream out(urdf_path);
    assert(out && "failed to create URDF file");
    out << R"(
<robot name="test_robot">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="tool0"/>

  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>

  <joint name="tool_fixed" type="fixed">
    <parent link="link1"/>
    <child link="tool0"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
  </joint>
</robot>
)";
  }

  const std::string mesh_dir = "sclerp_gazebo_test_robot_mesh_dir";
  std::filesystem::create_directory(mesh_dir);

  const std::string stl_base = mesh_dir + "/base_link.stl";
  const std::string stl_link1 = mesh_dir + "/link1.stl";
  const std::string stl_tool = mesh_dir + "/tool0.stl";
  writeAsciiStlTriangle(stl_base);
  writeAsciiStlTriangle(stl_link1);
  writeAsciiStlTriangle(stl_tool);

  ObstacleRegistry reg;

  WorldExportOptions opt;
  opt.include_ground_plane = false;
  opt.include_sun = false;

  RobotModelFromUrdf robot;
  robot.name = "robot";
  robot.urdf_path = urdf_path;
  robot.base_link = "base_link";
  robot.tip_link = "tool0";
  robot.collapse_fixed_joints = true;
  robot.link_mesh_stl_directory = mesh_dir;
  opt.robot_from_urdf = robot;

  const std::string sdf_path = "sclerp_gazebo_test_world_with_robot_auto_mesh.sdf";
  assert(ok(reg.writeSdfWorld(sdf_path, opt)));
  const std::string sdf = slurp(sdf_path);

  // The exporter writes file:// URIs for absolute paths; match basenames here.
  assert(sdf.find("base_link.stl") != std::string::npos);
  assert(sdf.find("link1.stl") != std::string::npos);
  assert(sdf.find("tool0.stl") != std::string::npos);

  std::remove(urdf_path.c_str());
  std::remove(sdf_path.c_str());
  std::filesystem::remove_all(mesh_dir);
}

static void test_sdf_import() {
  const std::string stl_path = "sclerp_gazebo_import_test_mesh.stl";
  writeAsciiStlTriangle(stl_path);

  const std::string sdf_path = "sclerp_gazebo_import_test_world.sdf";
  {
    std::ofstream out(sdf_path);
    assert(out && "failed to create SDF file");
    out << R"(<?xml version="1.0"?>
<sdf version="1.7">
  <world name="test_world">
    <model name="obstacle_box">
      <static>true</static>
      <pose>1 2 3 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <pose>0.1 0.2 0.3 0 0 0</pose>
          <geometry>
            <box><size>1 2 3</size></box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="obstacle_mesh">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh><uri>)" << stl_path << R"(</uri></mesh>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Should be ignored by default (common robot name). -->
    <model name="robot">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>10 10 10</size></box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
)";
  }

  ObstacleRegistry reg;
  assert(ok(reg.loadFromSdfWorld(sdf_path)));
  assert(reg.obstacles().size() == 2);
  assert(reg.obstacleNames().size() == 2);

  bool saw_box = false;
  bool saw_mesh = false;

  for (std::size_t i = 0; i < reg.obstacles().size(); ++i) {
    const auto& obj = reg.obstacles()[i];
    assert(obj);

    const auto geom = obj->collisionObject().collisionGeometry();
    assert(geom);

    if (const auto* box = dynamic_cast<const fcl::Boxd*>(geom.get())) {
      saw_box = true;
      assert(std::abs(box->side.x() - 1.0) < 1e-12);
      assert(std::abs(box->side.y() - 2.0) < 1e-12);
      assert(std::abs(box->side.z() - 3.0) < 1e-12);

      const auto t = obj->collisionObject().getTranslation();
      assert(std::abs(t.x() - 1.1) < 1e-12);
      assert(std::abs(t.y() - 2.2) < 1e-12);
      assert(std::abs(t.z() - 3.3) < 1e-12);
      continue;
    }

    if (const auto* mesh_obj = dynamic_cast<const sclerp::collision::MeshObject*>(obj.get())) {
      saw_mesh = true;
      assert(mesh_obj->stlPath() == stl_path);
      continue;
    }
  }

  assert(saw_box);
  assert(saw_mesh);

  std::remove(stl_path.c_str());
  std::remove(sdf_path.c_str());
}

int main() {
  test_exports();
  test_trajectory_csv();
  test_robot_from_urdf_inline_export();
  test_robot_from_urdf_mesh_dir_auto_discovery();
  test_sdf_import();
  std::cout << "sclerp_gazebo_unit_test: PASS\n";
  return 0;
}
