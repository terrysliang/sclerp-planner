#pragma once

#include "sclerp/core/common/status.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/collision/types.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <fcl/fcl.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace sclerp::collision {

// FCL-backed collision queries + contact extraction.
//
// The collision module assumes `link_meshes` includes base at index 0, followed by one mesh per
// kinematic link. When the model exposes a tool frame, an additional tool mesh may be appended.
// `mesh_offset_transforms` (in the planner wrapper) aligns meshes with the kinematic frames 1:1.
using sclerp::core::Status;
using sclerp::core::Vec3;
using sclerp::core::Mat3;
using sclerp::core::Mat4;

class FclObject {
public:
  virtual ~FclObject() = default;

  virtual const fcl::CollisionObjectd& collisionObject() const;
  virtual void setTransform(const Vec3& position, const Mat3& orientation);
  void computeAABB();

protected:
  FclObject(std::shared_ptr<fcl::CollisionGeometryd> geometry,
            const Vec3& position,
            const Mat3& orientation);

  std::shared_ptr<fcl::CollisionGeometryd> geometry_;
  fcl::CollisionObjectd collision_object_;
};

class BoxObject : public FclObject {
public:
  BoxObject(const Vec3& dimensions,
              const Vec3& position,
              const Mat3& orientation = Mat3::Identity());
};

class SphereObject : public FclObject {
public:
  SphereObject(double radius,
                 const Vec3& position,
                 const Mat3& orientation = Mat3::Identity());
};

class CylinderObject : public FclObject {
public:
  CylinderObject(double radius,
                   double height,
                   const Vec3& position,
                   const Mat3& orientation = Mat3::Identity());
};

class MeshObject : public FclObject {
public:
  MeshObject(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>>& bvh_model,
               const Vec3& position,
               const Mat3& orientation);
};

class PlaneObject : public FclObject {
public:
  PlaneObject(const Vec3& normal, double offset);
};

Status createBox(const Vec3& dimensions,
                 const Vec3& position,
                 const Mat3& orientation,
                 std::shared_ptr<FclObject>* out);

Status createSphere(double radius,
                    const Vec3& position,
                    const Mat3& orientation,
                    std::shared_ptr<FclObject>* out);

Status createCylinder(double radius,
                      double height,
                      const Vec3& position,
                      const Mat3& orientation,
                      std::shared_ptr<FclObject>* out);

Status createPlane(const Vec3& normal,
                   double offset,
                   std::shared_ptr<FclObject>* out);

Status removeObstacle(std::vector<std::shared_ptr<FclObject>>& obstacles, size_t index);

Status createGraspObject(const std::string& type,
                         const Vec3& size,
                         const Mat4& g_base_tool,
                         std::shared_ptr<FclObject>* out);

Status createMeshFromSTL(const std::string& stl_path,
                         const Mat4& transform,
                         std::shared_ptr<FclObject>* out);

Status buildLinkMeshes(const std::vector<std::string>& stl_files,
                       std::vector<std::shared_ptr<FclObject>>* out);

Status updateLinkMeshTransforms(const std::vector<std::shared_ptr<FclObject>>& link_meshes,
                                const std::vector<Mat4>& g_intermediate,
                                // mesh_offset_transforms aligns 1:1 with link_meshes/g_intermediate (index 0 is base).
                                const std::vector<Mat4>& mesh_offset_transforms);

Status checkCollision(const FclObject& obj1,
                      const FclObject& obj2,
                      double* min_dist,
                      Vec3* contact_point_obj1,
                      Vec3* contact_point_obj2);

struct CollisionContext {
  const std::vector<std::shared_ptr<FclObject>>& link_meshes;  // includes base at index 0
  const std::vector<std::shared_ptr<FclObject>>& obstacles;
  const std::shared_ptr<FclObject>& grasped_object;
};

Status computeContacts(const sclerp::core::KinematicsSolver& solver,
                       const Eigen::VectorXd& q,
                       const CollisionContext& ctx,
                       const CollisionQueryOptions& opt,
                       ContactSet* out);

}  // namespace sclerp::collision
