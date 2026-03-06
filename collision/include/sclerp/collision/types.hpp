#pragma once

#include "sclerp/core/math/types.hpp"

#include <Eigen/Dense>
#include <vector>

namespace sclerp::collision {

using sclerp::core::Vec3;

// Collision contact data used by the avoidance layer.
//
// Conventions:
// - `distance` is non-negative (penetration is treated as distance 0 in this stack).
// - `normal` points from the obstacle/grasped object toward the robot link (push-out direction).
// - `J_contact` is a reduced 3×k point Jacobian in the world frame:
//     p_dot = J_contact * q_dot.head(k)
struct Contact {
  int link_index = -1;  // 0-based joint link index; -1 for grasped object
  bool is_grasped = false;
  double distance = 0.0;
  Vec3 normal = Vec3::Zero();     // in base/world frame
  Vec3 point_obj = Vec3::Zero();  // on obstacle/grasped object
  Vec3 point_link = Vec3::Zero(); // on robot link
  Eigen::MatrixXd J_contact;
};

struct ContactSet {
  std::vector<Contact> contacts;
};

struct CollisionQueryOptions {
  bool check_self_collision = false;
  int num_links_ignore = 0;
  // Prunes obstacle-vs-link and obstacle-vs-grasped-object environment queries with an obstacle
  // broadphase. Self-collision remains pairwise. When reused by the collision planner, this
  // assumes the obstacle set and obstacle transforms stay fixed for that planning call.
  bool use_obstacle_broadphase = true;
};

}  // namespace sclerp::collision
