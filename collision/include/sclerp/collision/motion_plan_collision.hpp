#pragma once

#include "sclerp/core/planning/motion_plan.hpp"
#include "sclerp/collision/avoidance.hpp"
#include "sclerp/collision/collision.hpp"

#include <memory>
#include <vector>

namespace sclerp::collision {

// Collision-aware wrapper around `core::planMotionSclerp`.
//
// The structure mirrors the core planner loop, but inserts:
// - kinematics -> mesh transform sync
// - closest-contact extraction (FCL)
// - LCP-based joint correction (`adjustJoints`)
struct CollisionMotionPlanOptions {
  sclerp::core::MotionPlanOptions motion{};
  CollisionQueryOptions query{};
  CollisionAvoidanceOptions avoidance{};
};

struct CollisionScene {
  const CollisionContext& ctx;
  const std::vector<sclerp::core::Mat4>& mesh_offset_transforms;
};

sclerp::core::MotionPlanResult planMotionSclerpWithCollision(
    const sclerp::core::KinematicsSolver& solver,
    const sclerp::core::MotionPlanRequest& req,
    CollisionScene& scene,
    const CollisionMotionPlanOptions& opt = {});

}  // namespace sclerp::collision
