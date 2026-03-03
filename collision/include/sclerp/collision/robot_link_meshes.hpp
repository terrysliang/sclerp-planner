#pragma once

#include "sclerp/collision/collision.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace sclerp::collision {

// Metadata needed to resolve per-frame collision meshes.
//
// `frame_names` must align with `KinematicsSolver::forwardKinematicsAll` output indexing:
// - index 0: base frame
// - indices 1..dof: per-joint tip frames
// - optional last index: tool frame (when solver.model().has_tool_frame() is true)
struct RobotLinkMeshSpec {
  std::vector<std::string> frame_names;

  // Optional mesh filename/URI hints per frame (same size as frame_names when provided).
  // If empty, mesh discovery falls back to stem-matching on `frame_names`.
  std::vector<std::string> frame_mesh_uris;
};

struct RobotLinkMeshBuildOptions {
  double tool_placeholder_radius = 1e-3;
};

// Builds `link_meshes` (FCL objects) aligned with FK-all output indexing from a directory of per-link STLs.
//
// Discovery rules for each frame:
// 1) If `spec.frame_mesh_uris[i]` is non-empty, resolve its basename (handles file://, package://) and match
//    against STL filenames in `stl_dir` (case-insensitive).
// 2) Otherwise, fall back to matching STL stem against `spec.frame_names[i]` (case-insensitive).
// 3) If no STL is found:
//    - if it's the tool frame (last index and solver.model().has_tool_frame()==true), a placeholder sphere is created
//    - otherwise, returns Status::InvalidParameter.
//
// `out_mesh_offset_transforms` is filled with identity transforms (same size as out_link_meshes).
Status buildRobotLinkMeshesFromStlDirectory(const sclerp::core::KinematicsSolver& solver,
                                           const std::filesystem::path& stl_dir,
                                           const RobotLinkMeshSpec& spec,
                                           std::vector<std::shared_ptr<FclObject>>* out_link_meshes,
                                           std::vector<sclerp::core::Mat4>* out_mesh_offset_transforms,
                                           const RobotLinkMeshBuildOptions& opt = {});

}  // namespace sclerp::collision

