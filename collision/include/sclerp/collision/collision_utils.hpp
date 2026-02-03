#pragma once

#include "sclerp/core/common/status.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/collision/types.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <fcl/fcl.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace sclerp::collision {

using sclerp::core::Status;
using sclerp::core::Vec3;
using sclerp::core::Mat3;
using sclerp::core::Mat4;

class ObstacleBase {
public:
  virtual ~ObstacleBase() = default;

  virtual const fcl::CollisionObjectd& collisionObject() const = 0;
  virtual void setTransform(const Vec3& position, const Mat3& orientation) = 0;
};

class BoxObstacle : public ObstacleBase {
public:
  BoxObstacle(const Vec3& dimensions,
              const Vec3& position,
              const Mat3& orientation = Mat3::Identity());

  const fcl::CollisionObjectd& collisionObject() const override;
  void setTransform(const Vec3& position, const Mat3& orientation) override;

private:
  std::shared_ptr<fcl::Boxd> geometry_;
  fcl::CollisionObjectd collision_object_;
};

class SphereObstacle : public ObstacleBase {
public:
  SphereObstacle(double radius,
                 const Vec3& position,
                 const Mat3& orientation = Mat3::Identity());

  const fcl::CollisionObjectd& collisionObject() const override;
  void setTransform(const Vec3& position, const Mat3& orientation) override;

private:
  std::shared_ptr<fcl::Sphered> geometry_;
  fcl::CollisionObjectd collision_object_;
};

class CylinderObstacle : public ObstacleBase {
public:
  CylinderObstacle(double radius,
                   double height,
                   const Vec3& position,
                   const Mat3& orientation = Mat3::Identity());

  const fcl::CollisionObjectd& collisionObject() const override;
  void setTransform(const Vec3& position, const Mat3& orientation) override;

private:
  std::shared_ptr<fcl::Cylinderd> geometry_;
  fcl::CollisionObjectd collision_object_;
};

class MeshObstacle : public ObstacleBase {
public:
  MeshObstacle(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>>& bvh_model,
               const Vec3& position,
               const Mat3& orientation);

  const fcl::CollisionObjectd& collisionObject() const override;
  void setTransform(const Vec3& position, const Mat3& orientation) override;

private:
  std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>> geometry_;
  fcl::CollisionObjectd collision_object_;
};

Status createBox(const Vec3& dimensions,
                 const Vec3& position,
                 const Mat3& orientation,
                 std::shared_ptr<ObstacleBase>* out);

Status createSphere(double radius,
                    const Vec3& position,
                    const Mat3& orientation,
                    std::shared_ptr<ObstacleBase>* out);

Status createCylinder(double radius,
                      double height,
                      const Vec3& position,
                      const Mat3& orientation,
                      std::shared_ptr<ObstacleBase>* out);

Status removeObstacle(std::vector<std::shared_ptr<ObstacleBase>>& obstacles, size_t index);

Status createGraspObject(const std::string& type,
                         const Vec3& size,
                         const Mat4& g_base_tool,
                         std::shared_ptr<ObstacleBase>* out);

Status createMeshFromSTL(const std::string& stl_path,
                         const Mat4& transform,
                         std::shared_ptr<ObstacleBase>* out);

Status buildLinkMeshes(const std::vector<std::string>& stl_files,
                       std::vector<std::shared_ptr<ObstacleBase>>* out);

Status updateLinkMeshTransforms(std::vector<std::shared_ptr<ObstacleBase>>& link_meshes,
                                const std::vector<Mat4>& g_intermediate,
                                const std::vector<Mat4>& mesh_offset_transforms);

Status checkCollision(const ObstacleBase& obj1,
                      const ObstacleBase& obj2,
                      double* min_dist,
                      Vec3* contact_point_obj1,
                      Vec3* contact_point_obj2);

// link_index is 0-based joint index; columns [0..link_index] are used.
Status getContactJacobian(int link_index,
                          const Vec3& contact_point,
                          const Eigen::MatrixXd& spatial_jacobian,
                          Eigen::MatrixXd* contact_jacobian);

Status getCollisionInfo(
    const std::vector<std::shared_ptr<ObstacleBase>>& link_cylinders,
    const std::vector<std::shared_ptr<ObstacleBase>>& obstacles,
    const std::shared_ptr<ObstacleBase>& grasped_object,
    const Eigen::MatrixXd& spatial_jacobian,
    bool check_self_collision,
    int num_links_ignore,
    int dof,
    Eigen::MatrixXd* contact_normal_array,
    std::vector<double>* dist_array,
    std::vector<Eigen::MatrixXd>* contact_points_array,
    std::vector<Eigen::MatrixXd>* j_contact_array);

Status computeContacts(
    const std::vector<std::shared_ptr<ObstacleBase>>& link_cylinders,
    const std::vector<std::shared_ptr<ObstacleBase>>& obstacles,
    const std::shared_ptr<ObstacleBase>& grasped_object,
    const Eigen::MatrixXd& spatial_jacobian,
    const CollisionQueryOptions& opt,
    ContactSet* out);

}  // namespace sclerp::collision
