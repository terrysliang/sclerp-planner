#pragma once

#include "sclerp/collision/collision.hpp"
#include "sclerp/core/common/status.hpp"
#include "sclerp/gazebo/joint_trajectory_csv.hpp"
#include "sclerp/gazebo/world_sdf.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace sclerp::gazebo {

using sclerp::core::Status;

// Simple registry that keeps obstacles in the same format required by the collision module,
// while also tracking stable names for export.
class ObstacleRegistry {
public:
  Status registerObstacle(const std::shared_ptr<sclerp::collision::FclObject>& obstacle,
                          std::string name = {});

  Status removeObstacle(std::size_t index);

  const std::vector<std::shared_ptr<sclerp::collision::FclObject>>& obstacles() const {
    return obstacles_;
  }

  const std::vector<std::string>& obstacleNames() const { return obstacle_names_; }

  // Exports obstacles to a parseable JSON file.
  Status writeJson(const std::string& json_path) const;

  // Exports a Gazebo SDF world containing the registered obstacles (and optionally a robot include and trajectory plugin).
  Status writeSdfWorld(const std::string& sdf_path, const WorldExportOptions& opt = {}) const;

  // Imports obstacles from a Gazebo SDF world (typically created interactively in Gazebo and saved via the
  // `WorldSdfSaver` Gazebo system plugin).
  // Existing obstacles are cleared.
  Status loadFromSdfWorld(const std::string& sdf_path, const SdfWorldImportOptions& opt = {});

private:
  std::vector<std::shared_ptr<sclerp::collision::FclObject>> obstacles_;
  std::vector<std::string> obstacle_names_;
};

}  // namespace sclerp::gazebo
