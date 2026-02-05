#pragma once

#include "sclerp/core/planning/motion_plan.hpp"
#include "sclerp/collision/avoidance.hpp"
#include "sclerp/collision/collision.hpp"

#include <memory>
#include <vector>

namespace sclerp::collision {

struct CollisionMotionPlanOptions {
  sclerp::core::MotionPlanOptions motion{};
  CollisionQueryOptions query{};
  CollisionAvoidanceOptions avoidance{};
};

struct CollisionScene {
  std::vector<std::shared_ptr<FclObject>>& link_meshes;
  const std::vector<sclerp::core::Mat4>& mesh_offset_transforms;
  const std::vector<std::shared_ptr<FclObject>>& obstacles;
  const std::shared_ptr<FclObject>& grasped_object;
};

sclerp::core::MotionPlanResult planMotionSclerpWithCollision(
    const sclerp::core::KinematicsSolver& solver,
    const sclerp::core::MotionPlanRequest& req,
    CollisionScene& scene,
    const CollisionMotionPlanOptions& opt = {});

}  // namespace sclerp::collision
