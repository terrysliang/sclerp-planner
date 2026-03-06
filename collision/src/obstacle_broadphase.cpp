// Obstacle broadphase cache for pruning environment-vs-query distance checks.
//
// The cache owns an FCL DynamicAABBTree over static obstacles and reuses the existing
// `checkCollision()` narrowphase to preserve exact contact semantics.
#include "obstacle_broadphase.hpp"

#include "sclerp/core/common/logger.hpp"

#include <limits>

namespace sclerp::collision::detail {

using sclerp::core::LogLevel;
using sclerp::core::log;
using sclerp::core::ok;

namespace {

void resetHit(ObstacleHit* out) {
  out->status = Status::Success;
  out->found = false;
  out->distance = std::numeric_limits<double>::infinity();
  out->point_obj.setZero();
  out->point_query.setZero();
}

}  // namespace

Status ObstacleBroadphaseCache::build(const std::vector<std::shared_ptr<FclObject>>& obstacles) {
  manager_.clear();
  obstacle_objects_.clear();
  obstacle_lookup_.clear();

  obstacle_objects_.reserve(obstacles.size());
  obstacle_lookup_.reserve(obstacles.size());

  // This cache is intended to live for one planner call. It stores raw FCL object pointers in the
  // broadphase manager, so callers must keep the obstacle set and obstacle transforms fixed while
  // the cache is in use.
  for (const auto& obstacle : obstacles) {
    if (!obstacle) {
      log(LogLevel::Error, "ObstacleBroadphaseCache: null obstacle");
      return Status::InvalidParameter;
    }

    obstacle->computeAABB();
    auto* raw = &obstacle->collisionObjectMutable();
    obstacle_objects_.push_back(raw);
    obstacle_lookup_[raw] = obstacle.get();
  }

  if (!obstacle_objects_.empty()) {
    manager_.registerObjects(obstacle_objects_);
    manager_.setup();
  }

  return Status::Success;
}

bool ObstacleBroadphaseCache::distanceCallback(fcl::CollisionObjectd* obj1,
                                               fcl::CollisionObjectd* obj2,
                                               void* cdata,
                                               double& dist) {
  auto* data = static_cast<DistanceCallbackData*>(cdata);
  if (!data || !data->cache || !data->query || !data->out) {
    return true;
  }

  FclObject* obstacle = nullptr;
  if (const auto it = data->cache->obstacle_lookup_.find(obj1);
      it != data->cache->obstacle_lookup_.end()) {
    obstacle = it->second;
  } else if (const auto it = data->cache->obstacle_lookup_.find(obj2);
             it != data->cache->obstacle_lookup_.end()) {
    obstacle = it->second;
  } else {
    log(LogLevel::Error, "ObstacleBroadphaseCache: callback object lookup failed");
    data->out->status = Status::Failure;
    return true;
  }

  if (data->exact_query_count) {
    ++(*data->exact_query_count);
  }

  double exact_dist = 0.0;
  Vec3 point_obj = Vec3::Zero();
  Vec3 point_query = Vec3::Zero();
  const Status st = checkCollision(*obstacle, *data->query, &exact_dist, &point_obj, &point_query);
  if (!ok(st)) {
    data->out->status = st;
    return true;
  }

  if (!data->out->found || exact_dist < data->out->distance) {
    data->out->found = true;
    data->out->distance = exact_dist;
    data->out->point_obj = point_obj;
    data->out->point_query = point_query;
  }

  dist = std::min(dist, data->out->distance);
  return data->out->distance <= 0.0;
}

Status ObstacleBroadphaseCache::findNearestObstacleContact(FclObject& query,
                                                           ObstacleHit* out,
                                                           std::size_t* exact_query_count) const {
  if (!out) {
    log(LogLevel::Error, "ObstacleBroadphaseCache: null output");
    return Status::InvalidParameter;
  }

  resetHit(out);
  if (empty()) {
    return Status::Success;
  }

  query.computeAABB();
  auto* raw_query = &query.collisionObjectMutable();

  DistanceCallbackData data;
  data.cache = this;
  data.query = &query;
  data.out = out;
  data.exact_query_count = exact_query_count;

  manager_.distance(raw_query, &data, &ObstacleBroadphaseCache::distanceCallback);
  return out->status;
}

}  // namespace sclerp::collision::detail
