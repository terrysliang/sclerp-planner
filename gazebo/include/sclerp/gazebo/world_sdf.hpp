#pragma once

#include "sclerp/core/math/types.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sclerp::gazebo {

using sclerp::core::Mat3;
using sclerp::core::Mat4;
using sclerp::core::Vec3;

struct Pose {
  Vec3 position{Vec3::Zero()};
  Mat3 orientation{Mat3::Identity()};
};

struct RobotModelInclude {
  std::string uri;
  std::string name{"robot"};
  Pose pose{};

  // If set, overrides the included model's `<static>` value.
  // Use this to fix the robot base to the world for visualization / kinematic playback.
  std::optional<bool> static_model;
};

// Builds a minimal (serial-chain) robot model directly into the exported world SDF.
//
// This is meant to reuse the same URDF + per-link mesh inputs typically needed by the planner:
// - URDF provides joint names/types/origins/axes (kinematics structure).
// - `link_mesh_stl_files` + `mesh_offset_transforms` provide geometry.
//
// Notes:
// - This exporter intentionally uses simple inertials so that URDFs without inertial blocks can still be visualized.
// - Only a single chain from `base_link` to `tip_link` is supported (matching `sclerp::urdf`).
struct RobotModelFromUrdf {
  std::string name{"robot"};
  Pose pose{};

  std::string urdf_path;
  std::string base_link;
  std::string tip_link;

  // If true (default), fixed joints are collapsed into the relative poses of the next exported movable joint.
  bool collapse_fixed_joints{true};

  // Optional fixed tool transform on top of `tip_link` (SE(3), row-major in the usual Eigen convention).
  Mat4 tool_offset{Mat4::Identity()};

  // Link meshes aligned with the exported kinematic frames:
  // - index 0: base frame
  // - indices 1..dof: per-joint tip frames
  // - optional last index: tool frame when present
  std::vector<std::string> link_mesh_stl_files;

  // Optional: when set (and `link_mesh_stl_files` is empty), auto-discover per-link STL meshes from this directory.
  // The exporter matches STL basenames to URDF link names (case-insensitive), and will also fall back to URDF
  // link <visual>/<collision> mesh filenames when present.
  // The directory is scanned recursively; STL basenames/stems must be unique to avoid ambiguity.
  //
  // This is meant as a convenience for generating a complete `world.sdf` without writing a custom
  // planner/export script.
  std::string link_mesh_stl_directory;

  // Optional per-link mesh offsets (same size as `link_mesh_stl_files` when provided).
  // Each offset is applied as: world_T_mesh = world_T_link * mesh_offset.
  std::vector<Mat4> mesh_offset_transforms;

  // Dummy inertial parameters (for visualization / kinematic playback).
  double link_mass{1.0};
  double inertia_diagonal{1e-2};

  // If true (default), exports `<static>true</static>` so the base is fixed to the world.
  // This is recommended when using `JointTrajectoryPlayer` for kinematic visualization.
  bool static_model{true};
};

struct JointTrajectoryPlayback {
  // Relative paths are interpreted by Gazebo relative to the current working directory.
  std::string csv_path;
  bool loop{false};
  double rate{1.0};
};

// Options for emitting the `WorldSdfSaver` Gazebo system plugin stanza in exported worlds.
struct WorldSdfSaverOptions {
  // Default output path used when the save service request is empty.
  // Relative paths are interpreted by Gazebo relative to the current working directory.
  std::string output_path{"world_saved.sdf"};

  // Timeout for the internal `generate_world_sdf` request.
  int timeout_ms{5000};

  // Generator config options passed to `/world/<name>/generate_world_sdf`.
  bool expand_include_tags{true};
  bool resources_use_absolute_paths{true};
  bool copy_model_resources{false};
};

// Options for emitting the `PrimitiveResizer` Gazebo system plugin stanza in exported worlds.
struct PrimitiveResizerOptions {
};

struct WorldExportOptions {
  std::string world_name{"sclerp_world"};
  bool include_sun{true};
  bool include_ground_plane{true};

  // If set, the robot is generated inline from URDF + link meshes.
  std::optional<RobotModelFromUrdf> robot_from_urdf;

  // If set, includes a pre-existing robot model (SDF / Fuel model).
  std::optional<RobotModelInclude> robot;

  // If set, emits a `<plugin>` stanza in the world for playing a joint CSV.
  // The plugin itself is built when `SCLERP_GAZEBO_BUILD_IGNITION_PLUGIN=ON`.
  std::optional<JointTrajectoryPlayback> joint_trajectory;

  // If set, emits a `<plugin>` stanza in the world which advertises a service for saving the
  // current (GUI-edited) world to an SDF file via `/world/<name>/generate_world_sdf`.
  std::optional<WorldSdfSaverOptions> world_sdf_saver;

  // If set, emits a `<plugin>` stanza in the world which advertises a service for resizing
  // primitive obstacle models (box / sphere / cylinder).
  std::optional<PrimitiveResizerOptions> primitive_resizer;

  // Ignition Gazebo Sim 6 system plugin config (used when `joint_trajectory` is set).
  std::string joint_trajectory_plugin_filename{"libsclerp_joint_trajectory_player.so"};
  std::string joint_trajectory_plugin_name{"sclerp::gazebo::JointTrajectoryPlayer"};

  // Ignition Gazebo Sim 6 system plugin config (used when `world_sdf_saver` is set).
  std::string world_sdf_saver_plugin_filename{"libsclerp_world_sdf_saver.so"};
  std::string world_sdf_saver_plugin_name{"sclerp::gazebo::WorldSdfSaver"};

  // Ignition Gazebo Sim 6 system plugin config (used when `primitive_resizer` is set).
  std::string primitive_resizer_plugin_filename{"libsclerp_primitive_resizer.so"};
  std::string primitive_resizer_plugin_name{"sclerp::gazebo::PrimitiveResizer"};
};

struct SdfWorldImportOptions {
  // If true (default), import only models with `<static>true</static>`.
  bool static_only{true};

  // Model names to ignore (exact match).
  // By default, ignore the common names emitted by the exporter.
  std::vector<std::string> ignore_models{"ground_plane", "robot"};
};

}  // namespace sclerp::gazebo
