#pragma once

#include "sclerp/collision/collision.hpp"

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace sclerp::collision::detail {

struct ObstacleHit {
  Status status = Status::Success;
  bool found = false;
  double distance = 0.0;
  Vec3 point_obj = Vec3::Zero();
  Vec3 point_query = Vec3::Zero();
};

class ObstacleBroadphaseCache {
public:
  Status build(const std::vector<std::shared_ptr<FclObject>>& obstacles);

  Status findNearestObstacleContact(FclObject& query,
                                    ObstacleHit* out,
                                    std::size_t* exact_query_count = nullptr) const;

  bool empty() const { return obstacle_objects_.empty(); }

private:
  struct DistanceCallbackData {
    const ObstacleBroadphaseCache* cache = nullptr;
    FclObject* query = nullptr;
    ObstacleHit* out = nullptr;
    std::size_t* exact_query_count = nullptr;
  };

  static bool distanceCallback(fcl::CollisionObjectd* obj1,
                               fcl::CollisionObjectd* obj2,
                               void* cdata,
                               double& dist);

  std::vector<fcl::CollisionObjectd*> obstacle_objects_;
  std::unordered_map<fcl::CollisionObjectd*, FclObject*> obstacle_lookup_;
  fcl::DynamicAABBTreeCollisionManagerd manager_;
};

}  // namespace sclerp::collision::detail
