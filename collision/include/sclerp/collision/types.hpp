#pragma once

#include "sclerp/core/math/types.hpp"

#include <Eigen/Dense>
#include <vector>

namespace sclerp::collision {

using sclerp::core::Vec3;

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
};

}  // namespace sclerp::collision
