#pragma once

#include <string>
#include <vector>

#include "sclerp/core/common/status.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/model/manipulator_model.hpp"

namespace sclerp::urdf {

// URDF -> `core::ManipulatorModel` adapter.
//
// Scope/assumptions:
// - Loads a single serial chain from `base_link` to `tip_link`.
// - Supports revolute/continuous/prismatic/fixed joints along the chain.
// - Fixed joints can be collapsed into the chain transforms (default) or kept as DOFs.
// - Optional `tool_offset` lets callers define a tool frame on top of `tip_link` at q=0.
struct LoadOptions {
  std::string base_link;
  std::string tip_link;

  // Optional tool offset applied on top of tip_link frame at q=0:
  // ee_home = T_base_tip0 * tool_offset
  sclerp::core::Transform tool_offset = sclerp::core::Transform::Identity();

  // If true (default), fixed joints are collapsed into the chain transforms.
  // If false, fixed joints are included as JointType::Fixed and will increase dof();
  // callers should provide q entries (typically zeros) for those fixed joints.
  bool collapse_fixed_joints = true;

  // If true, any unsupported joint types will trigger failure.
  bool strict = true;
};

struct LoadResult {
  sclerp::core::Status status{sclerp::core::Status::Failure};
  sclerp::core::ManipulatorModel model;
  std::string message;  // optional debug info for caller
};

LoadResult loadManipulatorModelFromFile(const std::string& urdf_path,
                                        const LoadOptions& opt);

LoadResult loadManipulatorModelFromString(const std::string& urdf_xml,
                                          const LoadOptions& opt);

}  // namespace sclerp::urdf
