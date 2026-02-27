#include "sclerp/gazebo/obstacle_registry.hpp"

#include "sclerp/core/common/logger.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <sstream>
#include <string>
#include <string_view>

namespace sclerp::gazebo {
namespace {

using sclerp::core::LogLevel;
using sclerp::core::log;

static std::string sanitizeSdfName(std::string_view name) {
  std::string out;
  out.reserve(name.size());
  for (char c : name) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc) || c == '_' || c == '-') {
      out.push_back(c);
    } else if (std::isspace(uc)) {
      out.push_back('_');
    } else {
      out.push_back('_');
    }
  }
  if (out.empty()) out = "object";
  return out;
}

}  // namespace

Status ObstacleRegistry::registerObstacle(const std::shared_ptr<sclerp::collision::FclObject>& obstacle,
                                         std::string name) {
  if (!obstacle) {
    log(LogLevel::Error, "ObstacleRegistry::registerObstacle: null obstacle");
    return Status::InvalidParameter;
  }

  if (name.empty()) {
    std::ostringstream oss;
    oss << "obstacle_" << obstacles_.size();
    name = oss.str();
  }

  const std::string base = sanitizeSdfName(name);
  name = base;
  for (std::size_t suffix = 1;
       std::find(obstacle_names_.begin(), obstacle_names_.end(), name) != obstacle_names_.end();
       ++suffix) {
    name = base + "_" + std::to_string(suffix);
  }

  obstacles_.push_back(obstacle);
  obstacle_names_.push_back(std::move(name));
  return Status::Success;
}

Status ObstacleRegistry::removeObstacle(std::size_t index) {
  if (index >= obstacles_.size()) {
    log(LogLevel::Error, "ObstacleRegistry::removeObstacle: index out of range");
    return Status::InvalidParameter;
  }
  obstacles_.erase(obstacles_.begin() + static_cast<std::ptrdiff_t>(index));
  obstacle_names_.erase(obstacle_names_.begin() + static_cast<std::ptrdiff_t>(index));
  return Status::Success;
}

}  // namespace sclerp::gazebo

