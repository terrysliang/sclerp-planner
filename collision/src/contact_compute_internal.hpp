#pragma once

#include "sclerp/collision/collision.hpp"

#include <cstddef>

namespace sclerp::collision::detail {

class ObstacleBroadphaseCache;

struct ComputeContactsStats {
  std::size_t exact_obstacle_narrowphase_checks = 0;
};

Status computeContactsImpl(const sclerp::core::KinematicsSolver& solver,
                           const Eigen::VectorXd& q,
                           const CollisionContext& ctx,
                           const CollisionQueryOptions& opt,
                           const ObstacleBroadphaseCache* obstacle_broadphase,
                           ContactSet* out,
                           ComputeContactsStats* stats = nullptr);

}  // namespace sclerp::collision::detail
