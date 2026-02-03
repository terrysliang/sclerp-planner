#pragma once

#include "sclerp/core/planning/motion_plan.hpp"
#include "sclerp/collision/collision_utils.hpp"

namespace sclerp::collision {

struct CollisionMotionPlanOptions {
  sclerp::core::MotionPlanOptions motion{};
  bool check_self_collision = false;
  int num_links_ignore = 0;
  double safe_dist = 0.01;
  double collision_dt = 0.001;
};

sclerp::core::MotionPlanResult planMotionSclerpWithCollision(
    const sclerp::core::KinematicsSolver& solver,
    const sclerp::core::MotionPlanRequest& req,
    std::vector<std::shared_ptr<ObstacleBase>>& link_meshes,
    const std::vector<sclerp::core::Mat4>& mesh_offset_transforms,
    const std::vector<std::shared_ptr<ObstacleBase>>& obstacles,
    const std::shared_ptr<ObstacleBase>& grasped_object,
    const CollisionMotionPlanOptions& opt = {});

}  // namespace sclerp::collision
