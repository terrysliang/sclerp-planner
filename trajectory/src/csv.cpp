#include "sclerp/trajectory/interpolator.hpp"

#include "sclerp/core/common/logger.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>

namespace sclerp::trajectory {
namespace {

using sclerp::core::LogLevel;
using sclerp::core::log;

}  // namespace

Status writeTrajectoryCsv(const PlannedTrajectory& traj,
                          const std::string& csv_path,
                          CsvMode mode) {
  if (traj.table.empty()) {
    log(LogLevel::Error, "writeTrajectoryCsv: trajectory table is empty");
    return Status::InvalidParameter;
  }

  const int dof = static_cast<int>(traj.table.front().q.size());
  if (dof <= 0) {
    log(LogLevel::Error, "writeTrajectoryCsv: invalid dof");
    return Status::InvalidParameter;
  }

  if (traj.joint_names.size() != static_cast<std::size_t>(dof)) {
    log(LogLevel::Error, "writeTrajectoryCsv: joint_names must be provided and match dof");
    return Status::InvalidParameter;
  }

  for (const auto& smp : traj.table) {
    if (smp.q.size() != dof || smp.qd.size() != dof || smp.qdd.size() != dof) {
      log(LogLevel::Error, "writeTrajectoryCsv: inconsistent dof across samples");
      return Status::InvalidParameter;
    }
    if (!smp.q.allFinite() || !smp.qd.allFinite() || !smp.qdd.allFinite()) {
      log(LogLevel::Error, "writeTrajectoryCsv: NaN/Inf in trajectory sample");
      return Status::InvalidParameter;
    }
    if (!std::isfinite(smp.t)) {
      log(LogLevel::Error, "writeTrajectoryCsv: non-finite sample time");
      return Status::InvalidParameter;
    }
  }

  std::ofstream out(csv_path);
  if (!out) {
    log(LogLevel::Error, "writeTrajectoryCsv: failed to open output file");
    return Status::Failure;
  }

  out << std::setprecision(17);
  out << "time";

  if (mode == CsvMode::PositionOnly) {
    for (const auto& name : traj.joint_names) out << "," << name;
    out << "\n";
    for (const auto& smp : traj.table) {
      out << smp.t;
      for (int j = 0; j < dof; ++j) out << "," << smp.q[j];
      out << "\n";
    }
    return Status::Success;
  }

  for (const auto& name : traj.joint_names) out << "," << name;
  for (const auto& name : traj.joint_names) out << ",d" << name;
  for (const auto& name : traj.joint_names) out << ",dd" << name;
  out << "\n";

  for (const auto& smp : traj.table) {
    out << smp.t;
    for (int j = 0; j < dof; ++j) out << "," << smp.q[j];
    for (int j = 0; j < dof; ++j) out << "," << smp.qd[j];
    for (int j = 0; j < dof; ++j) out << "," << smp.qdd[j];
    out << "\n";
  }

  return Status::Success;
}

}  // namespace sclerp::trajectory

