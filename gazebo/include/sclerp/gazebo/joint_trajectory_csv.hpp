#pragma once

#include "sclerp/core/common/status.hpp"
#include "sclerp/core/path/joint_path.hpp"

#include <string>

namespace sclerp::gazebo {

using sclerp::core::Status;

// Writes a simple CSV: header `time,<joint1>,...,<jointN>` and rows `t,q1,...,qN`.
Status writeJointTrajectoryCsv(const sclerp::core::JointPath& path,
                               double dt,
                               const std::string& csv_path);

}  // namespace sclerp::gazebo

