#include "bind_common.hpp"

#include "sclerp/gazebo/joint_trajectory_csv.hpp"
#include "sclerp/gazebo/world_registry.hpp"
#include "sclerp/gazebo/world_sdf.hpp"

namespace sclerp::python {
namespace {

using sclerp::gazebo::JointTrajectoryPlayback;
using sclerp::gazebo::Pose;
using sclerp::gazebo::PrimitiveResizerOptions;
using sclerp::gazebo::RobotModelFromUrdf;
using sclerp::gazebo::RobotModelInclude;
using sclerp::gazebo::SdfWorldImportOptions;
using sclerp::gazebo::WorldExportOptions;
using sclerp::gazebo::WorldRegistry;
using sclerp::gazebo::WorldSdfSaverOptions;

}  // namespace

void bind_gazebo(py::module_& m) {
  py::class_<Pose>(m, "Pose")
      .def(py::init<>())
      .def_readwrite("position", &Pose::position)
      .def_readwrite("orientation", &Pose::orientation);

  py::class_<RobotModelInclude>(m, "RobotModelInclude")
      .def(py::init<>())
      .def_readwrite("uri", &RobotModelInclude::uri)
      .def_readwrite("name", &RobotModelInclude::name)
      .def_readwrite("pose", &RobotModelInclude::pose)
      .def_readwrite("static_model", &RobotModelInclude::static_model);

  py::class_<RobotModelFromUrdf>(m, "RobotModelFromUrdf")
      .def(py::init<>())
      .def_readwrite("name", &RobotModelFromUrdf::name)
      .def_readwrite("pose", &RobotModelFromUrdf::pose)
      .def_readwrite("urdf_path", &RobotModelFromUrdf::urdf_path)
      .def_readwrite("base_link", &RobotModelFromUrdf::base_link)
      .def_readwrite("tip_link", &RobotModelFromUrdf::tip_link)
      .def_readwrite("collapse_fixed_joints", &RobotModelFromUrdf::collapse_fixed_joints)
      .def_readwrite("tool_offset", &RobotModelFromUrdf::tool_offset)
      .def_readwrite("link_mesh_stl_files", &RobotModelFromUrdf::link_mesh_stl_files)
      .def_readwrite("link_mesh_stl_directory", &RobotModelFromUrdf::link_mesh_stl_directory)
      .def_readwrite("mesh_offset_transforms", &RobotModelFromUrdf::mesh_offset_transforms)
      .def_readwrite("link_mass", &RobotModelFromUrdf::link_mass)
      .def_readwrite("inertia_diagonal", &RobotModelFromUrdf::inertia_diagonal)
      .def_readwrite("static_model", &RobotModelFromUrdf::static_model);

  py::class_<JointTrajectoryPlayback>(m, "JointTrajectoryPlayback")
      .def(py::init<>())
      .def_readwrite("csv_path", &JointTrajectoryPlayback::csv_path)
      .def_readwrite("loop", &JointTrajectoryPlayback::loop)
      .def_readwrite("rate", &JointTrajectoryPlayback::rate);

  py::class_<WorldSdfSaverOptions>(m, "WorldSdfSaverOptions")
      .def(py::init<>())
      .def_readwrite("output_path", &WorldSdfSaverOptions::output_path)
      .def_readwrite("timeout_ms", &WorldSdfSaverOptions::timeout_ms)
      .def_readwrite("expand_include_tags", &WorldSdfSaverOptions::expand_include_tags)
      .def_readwrite("resources_use_absolute_paths", &WorldSdfSaverOptions::resources_use_absolute_paths)
      .def_readwrite("copy_model_resources", &WorldSdfSaverOptions::copy_model_resources);

  py::class_<PrimitiveResizerOptions>(m, "PrimitiveResizerOptions")
      .def(py::init<>());

  py::class_<WorldExportOptions>(m, "WorldExportOptions")
      .def(py::init<>())
      .def_readwrite("world_name", &WorldExportOptions::world_name)
      .def_readwrite("include_sun", &WorldExportOptions::include_sun)
      .def_readwrite("include_ground_plane", &WorldExportOptions::include_ground_plane)
      .def_readwrite("robot_from_urdf", &WorldExportOptions::robot_from_urdf)
      .def_readwrite("robot", &WorldExportOptions::robot)
      .def_readwrite("joint_trajectory", &WorldExportOptions::joint_trajectory)
      .def_readwrite("world_sdf_saver", &WorldExportOptions::world_sdf_saver)
      .def_readwrite("primitive_resizer", &WorldExportOptions::primitive_resizer)
      .def_readwrite("joint_trajectory_plugin_filename", &WorldExportOptions::joint_trajectory_plugin_filename)
      .def_readwrite("joint_trajectory_plugin_name", &WorldExportOptions::joint_trajectory_plugin_name)
      .def_readwrite("world_sdf_saver_plugin_filename", &WorldExportOptions::world_sdf_saver_plugin_filename)
      .def_readwrite("world_sdf_saver_plugin_name", &WorldExportOptions::world_sdf_saver_plugin_name)
      .def_readwrite("primitive_resizer_plugin_filename", &WorldExportOptions::primitive_resizer_plugin_filename)
      .def_readwrite("primitive_resizer_plugin_name", &WorldExportOptions::primitive_resizer_plugin_name);

  py::class_<SdfWorldImportOptions>(m, "SdfWorldImportOptions")
      .def(py::init<>())
      .def_readwrite("static_only", &SdfWorldImportOptions::static_only)
      .def_readwrite("ignore_models", &SdfWorldImportOptions::ignore_models);

  py::class_<WorldRegistry>(m, "WorldRegistry")
      .def(py::init<>())
      .def("register_obstacle", &WorldRegistry::registerObstacle, py::arg("obstacle"), py::arg("name") = std::string{})
      .def("remove_obstacle", &WorldRegistry::removeObstacle, py::arg("index"))
      .def("obstacles", [](const WorldRegistry& self) { return self.obstacles(); })
      .def("obstacle_names", [](const WorldRegistry& self) { return self.obstacleNames(); })
      .def("write_json", &WorldRegistry::writeJson, py::arg("json_path"))
      .def("write_sdf_world", &WorldRegistry::writeSdfWorld, py::arg("sdf_path"), py::arg("opt") = WorldExportOptions{})
      .def("load_from_sdf_world", &WorldRegistry::loadFromSdfWorld, py::arg("sdf_path"), py::arg("opt") = SdfWorldImportOptions{});

  m.def("write_joint_trajectory_csv",
        &sclerp::gazebo::writeJointTrajectoryCsv,
        py::arg("path"),
        py::arg("dt"),
        py::arg("csv_path"));
}

}  // namespace sclerp::python

