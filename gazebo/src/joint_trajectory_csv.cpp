#include "sclerp/gazebo/joint_trajectory_csv.hpp"

#include "sclerp/core/common/logger.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>

namespace sclerp::gazebo {

using sclerp::core::LogLevel;
using sclerp::core::log;
using sclerp::core::ok;

Status writeJointTrajectoryCsv(const sclerp::core::JointPath& path,
                               double dt,
                               const std::string& csv_path) {
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    log(LogLevel::Error, "writeJointTrajectoryCsv: dt must be > 0");
    return Status::InvalidParameter;
  }
  if (path.positions.empty()) {
    log(LogLevel::Error, "writeJointTrajectoryCsv: path is empty");
    return Status::InvalidParameter;
  }

  const int dof = static_cast<int>(path.positions.front().size());
  if (dof <= 0) {
    log(LogLevel::Error, "writeJointTrajectoryCsv: invalid dof");
    return Status::InvalidParameter;
  }

  if (path.joint_names.size() != static_cast<std::size_t>(dof)) {
    log(LogLevel::Error, "writeJointTrajectoryCsv: joint_names must be provided and match dof");
    return Status::InvalidParameter;
  }
  for (const auto& name : path.joint_names) {
    if (name.find(',') != std::string::npos) {
      log(LogLevel::Error, "writeJointTrajectoryCsv: joint names may not contain commas");
      return Status::InvalidParameter;
    }
  }

  for (const auto& q : path.positions) {
    if (q.size() != dof) {
      log(LogLevel::Error, "writeJointTrajectoryCsv: inconsistent dof across path");
      return Status::InvalidParameter;
    }
    if (!q.allFinite()) {
      log(LogLevel::Error, "writeJointTrajectoryCsv: NaN/Inf joint position in path");
      return Status::InvalidParameter;
    }
  }

  std::ofstream out(csv_path);
  if (!out) {
    log(LogLevel::Error, "writeJointTrajectoryCsv: failed to open output file");
    return Status::Failure;
  }

  out << std::setprecision(17);
  out << "time";
  for (const auto& name : path.joint_names) out << "," << name;
  out << "\n";

  for (std::size_t i = 0; i < path.positions.size(); ++i) {
    const double t = static_cast<double>(i) * dt;
    out << t;
    const auto& q = path.positions[i];
    for (int j = 0; j < dof; ++j) out << "," << q(j);
    out << "\n";
  }

  return Status::Success;
}

}  // namespace sclerp::gazebo

