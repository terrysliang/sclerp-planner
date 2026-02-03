#include "sclerp/collision/collision_utils.hpp"

#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/math/so3.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace sclerp::collision {

using sclerp::core::LogLevel;
using sclerp::core::log;

static inline bool validOut(const void* out, const char* msg) {
  if (out) return true;
  log(LogLevel::Error, msg);
  return false;
}

BoxObstacle::BoxObstacle(const Vec3& dimensions,
                         const Vec3& position,
                         const Mat3& orientation)
  : geometry_(std::make_shared<fcl::Boxd>(dimensions.x(), dimensions.y(), dimensions.z())),
    collision_object_(geometry_, fcl::Transform3d::Identity()) {
  collision_object_.setTranslation(position);
  collision_object_.setRotation(orientation);
}

const fcl::CollisionObjectd& BoxObstacle::collisionObject() const { return collision_object_; }

void BoxObstacle::setTransform(const Vec3& position, const Mat3& orientation) {
  collision_object_.setTranslation(position);
  collision_object_.setRotation(orientation);
}

SphereObstacle::SphereObstacle(double radius,
                               const Vec3& position,
                               const Mat3& orientation)
  : geometry_(std::make_shared<fcl::Sphered>(radius)),
    collision_object_(geometry_, fcl::Transform3d::Identity()) {
  collision_object_.setTranslation(position);
  collision_object_.setRotation(orientation);
}

const fcl::CollisionObjectd& SphereObstacle::collisionObject() const { return collision_object_; }

void SphereObstacle::setTransform(const Vec3& position, const Mat3& orientation) {
  collision_object_.setTranslation(position);
  collision_object_.setRotation(orientation);
}

CylinderObstacle::CylinderObstacle(double radius,
                                   double height,
                                   const Vec3& position,
                                   const Mat3& orientation)
  : geometry_(std::make_shared<fcl::Cylinderd>(radius, height)),
    collision_object_(geometry_, fcl::Transform3d::Identity()) {
  collision_object_.setTranslation(position);
  collision_object_.setRotation(orientation);
}

const fcl::CollisionObjectd& CylinderObstacle::collisionObject() const { return collision_object_; }

void CylinderObstacle::setTransform(const Vec3& position, const Mat3& orientation) {
  collision_object_.setTranslation(position);
  collision_object_.setRotation(orientation);
}

MeshObstacle::MeshObstacle(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>>& bvh_model,
                           const Vec3& position,
                           const Mat3& orientation)
  : geometry_(bvh_model),
    collision_object_(geometry_, fcl::Transform3d::Identity()) {
  setTransform(position, orientation);
}

const fcl::CollisionObjectd& MeshObstacle::collisionObject() const { return collision_object_; }

void MeshObstacle::setTransform(const Vec3& position, const Mat3& orientation) {
  collision_object_.setTranslation(position);
  collision_object_.setRotation(orientation);
}

Status createBox(const Vec3& dimensions,
                 const Vec3& position,
                 const Mat3& orientation,
                 std::shared_ptr<ObstacleBase>* out) {
  if (!validOut(out, "createBox: null output")) return Status::InvalidParameter;
  *out = std::make_shared<BoxObstacle>(dimensions, position, orientation);
  return Status::Success;
}

Status createSphere(double radius,
                    const Vec3& position,
                    const Mat3& orientation,
                    std::shared_ptr<ObstacleBase>* out) {
  if (!validOut(out, "createSphere: null output")) return Status::InvalidParameter;
  if (!(radius > 0.0)) {
    log(LogLevel::Error, "createSphere: radius must be > 0");
    return Status::InvalidParameter;
  }
  *out = std::make_shared<SphereObstacle>(radius, position, orientation);
  return Status::Success;
}

Status createCylinder(double radius,
                      double height,
                      const Vec3& position,
                      const Mat3& orientation,
                      std::shared_ptr<ObstacleBase>* out) {
  if (!validOut(out, "createCylinder: null output")) return Status::InvalidParameter;
  if (!(radius > 0.0) || !(height > 0.0)) {
    log(LogLevel::Error, "createCylinder: radius/height must be > 0");
    return Status::InvalidParameter;
  }
  *out = std::make_shared<CylinderObstacle>(radius, height, position, orientation);
  return Status::Success;
}

Status removeObstacle(std::vector<std::shared_ptr<ObstacleBase>>& obstacles, size_t index) {
  if (index >= obstacles.size()) {
    log(LogLevel::Error, "removeObstacle: index out of range");
    return Status::InvalidParameter;
  }
  obstacles.erase(obstacles.begin() + static_cast<std::ptrdiff_t>(index));
  return Status::Success;
}

Status createGraspObject(const std::string& type,
                         const Vec3& size,
                         const Mat4& g_base_tool,
                         std::shared_ptr<ObstacleBase>* out) {
  if (!validOut(out, "createGraspObject: null output")) return Status::InvalidParameter;

  const Vec3 position = g_base_tool.block<3,1>(0, 3);
  const Mat3 orientation = g_base_tool.block<3,3>(0, 0);

  if (type == "box") {
    return createBox(size, position, orientation, out);
  }
  if (type == "sphere") {
    return createSphere(size.x(), position, orientation, out);
  }
  if (type == "cylinder") {
    return createCylinder(size.x(), size.y(), position, orientation, out);
  }

  log(LogLevel::Error, "createGraspObject: unknown type");
  return Status::InvalidParameter;
}

Status createMeshFromSTL(const std::string& stl_path,
                         const Mat4& transform,
                         std::shared_ptr<ObstacleBase>* out) {
  if (!validOut(out, "createMeshFromSTL: null output")) return Status::InvalidParameter;

  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(stl_path, aiProcess_Triangulate);
  if (!scene || !scene->mMeshes) {
    log(LogLevel::Error, "createMeshFromSTL: failed to load STL");
    return Status::Failure;
  }

  aiMesh* mesh = scene->mMeshes[0];
  auto bvh_model = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();

  std::vector<fcl::Vector3d> vertices;
  std::vector<fcl::Triangle> triangles;
  vertices.reserve(mesh->mNumVertices);
  triangles.reserve(mesh->mNumFaces);

  for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
    vertices.emplace_back(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
  }

  for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
    aiFace face = mesh->mFaces[i];
    if (face.mNumIndices == 3) {
      triangles.emplace_back(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
    }
  }

  bvh_model->beginModel();
  bvh_model->addSubModel(vertices, triangles);
  bvh_model->endModel();

  const Vec3 position = transform.block<3,1>(0, 3);
  const Mat3 orientation = transform.block<3,3>(0, 0);
  *out = std::make_shared<MeshObstacle>(bvh_model, position, orientation);
  return Status::Success;
}

Status buildLinkMeshes(const std::vector<std::string>& stl_files,
                       std::vector<std::shared_ptr<ObstacleBase>>* out) {
  if (!validOut(out, "buildLinkMeshes: null output")) return Status::InvalidParameter;

  std::vector<std::shared_ptr<ObstacleBase>> link_meshes;
  link_meshes.reserve(stl_files.size());
  for (const auto& mesh_path : stl_files) {
    std::shared_ptr<ObstacleBase> mesh_obj;
    const Status st = createMeshFromSTL(mesh_path, Mat4::Identity(), &mesh_obj);
    if (!ok(st)) return st;
    link_meshes.push_back(std::move(mesh_obj));
  }

  *out = std::move(link_meshes);
  return Status::Success;
}

Status updateLinkMeshTransforms(std::vector<std::shared_ptr<ObstacleBase>>& link_meshes,
                                const std::vector<Mat4>& g_intermediate,
                                const std::vector<Mat4>& mesh_offset_transforms) {
  if (link_meshes.size() != g_intermediate.size()) {
    log(LogLevel::Error, "updateLinkMeshTransforms: size mismatch");
    return Status::InvalidParameter;
  }
  if (mesh_offset_transforms.size() + 1 != g_intermediate.size()) {
    log(LogLevel::Error, "updateLinkMeshTransforms: mesh offset size mismatch");
    return Status::InvalidParameter;
  }

  for (size_t i = 1; i < link_meshes.size(); ++i) {
    if (!link_meshes[i]) continue;
    const Mat4 T_world_to_mesh = g_intermediate[i] * mesh_offset_transforms[i - 1];
    const Vec3 position = T_world_to_mesh.block<3,1>(0, 3);
    const Mat3 rotation = T_world_to_mesh.block<3,3>(0, 0);
    link_meshes[i]->setTransform(position, rotation);
  }
  return Status::Success;
}

Status checkCollision(const ObstacleBase& obj1,
                      const ObstacleBase& obj2,
                      double* min_dist,
                      Vec3* contact_point_obj1,
                      Vec3* contact_point_obj2) {
  if (!validOut(min_dist, "checkCollision: null min_dist")) return Status::InvalidParameter;
  if (!validOut(contact_point_obj1, "checkCollision: null contact_point_obj1")) return Status::InvalidParameter;
  if (!validOut(contact_point_obj2, "checkCollision: null contact_point_obj2")) return Status::InvalidParameter;

  fcl::DistanceRequestd request;
  fcl::DistanceResultd result;
  request.enable_nearest_points = true;
  request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
  request.distance_tolerance = 1e-6;

  *min_dist = fcl::distance(&obj1.collisionObject(), &obj2.collisionObject(), request, result);
  if (std::isnan(*min_dist)) {
    log(LogLevel::Error, "checkCollision: min_dist is NaN");
    return Status::Failure;
  }

  if (*min_dist >= 0.0) {
    *contact_point_obj1 = result.nearest_points[0];
    *contact_point_obj2 = result.nearest_points[1];
    return Status::Success;
  }

  log(LogLevel::Error, "checkCollision: penetration detected");
  return Status::Failure;
}

Status getContactJacobian(int link_index,
                          const Vec3& contact_point,
                          const Eigen::MatrixXd& spatial_jacobian,
                          Eigen::MatrixXd* contact_jacobian) {
  if (!validOut(contact_jacobian, "getContactJacobian: null output")) return Status::InvalidParameter;

  const int cols = spatial_jacobian.cols();
  if (link_index < 0 || link_index >= cols) {
    log(LogLevel::Error, "getContactJacobian: link_index out of range");
    return Status::InvalidParameter;
  }

  const Mat3 P_hat = sclerp::core::hat3(contact_point);
  Eigen::MatrixXd Js = Eigen::MatrixXd::Zero(6, cols);
  // link_index is 0-based; include columns up to and including link_index.
  Js.leftCols(link_index + 1) = spatial_jacobian.leftCols(link_index + 1);

  Eigen::MatrixXd temp = (Eigen::MatrixXd(3, 6) << Mat3::Identity(), -P_hat).finished() * Js;

  contact_jacobian->resize(6, temp.cols());
  *contact_jacobian << temp, Eigen::MatrixXd::Zero(3, temp.cols());

  return Status::Success;
}

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
    std::vector<Eigen::MatrixXd>* j_contact_array) {
  if (!validOut(contact_normal_array, "getCollisionInfo: null contact_normal_array")) return Status::InvalidParameter;
  if (!validOut(dist_array, "getCollisionInfo: null dist_array")) return Status::InvalidParameter;
  if (!validOut(contact_points_array, "getCollisionInfo: null contact_points_array")) return Status::InvalidParameter;
  if (!validOut(j_contact_array, "getCollisionInfo: null j_contact_array")) return Status::InvalidParameter;

  const int grasped_obj_con = grasped_object ? 1 : 0;
  const int total_links = dof - num_links_ignore;
  if (total_links <= 0) {
    log(LogLevel::Error, "getCollisionInfo: invalid total_links");
    return Status::InvalidParameter;
  }

  dist_array->assign(total_links + grasped_obj_con, 1000.0);
  *contact_normal_array = Eigen::MatrixXd::Zero(6, total_links + grasped_obj_con);
  contact_points_array->assign(total_links + grasped_obj_con, Eigen::MatrixXd(3, 2));
  j_contact_array->assign(total_links + grasped_obj_con, Eigen::MatrixXd());

  for (const auto& obstacle : obstacles) {
    for (int itr_index = 1; itr_index <= total_links; ++itr_index) {
      const int num_link = num_links_ignore + itr_index;
      if (num_link < 0 || static_cast<size_t>(num_link) >= link_cylinders.size()) {
        log(LogLevel::Error, "getCollisionInfo: link cylinder index out of range");
        return Status::InvalidParameter;
      }
      const auto& current_cylinder = link_cylinders[num_link];
      if (!current_cylinder) {
        log(LogLevel::Error, "getCollisionInfo: link cylinder is null");
        return Status::Failure;
      }

      Vec3 cp_obj, cp_cylinder;
      double min_d = 0.0;
      const Status st = checkCollision(*obstacle, *current_cylinder, &min_d, &cp_obj, &cp_cylinder);
      if (!ok(st)) {
        log(LogLevel::Error, "getCollisionInfo: collision check failed (environment)");
        return st;
      }

      if (min_d < (*dist_array)[itr_index - 1]) {
        (*dist_array)[itr_index - 1] = min_d;
        contact_normal_array->block<3,1>(0, itr_index - 1) = (cp_cylinder - cp_obj).normalized();
        (*contact_points_array)[itr_index - 1] << cp_obj, cp_cylinder;
      }
    }
  }

  if (grasped_object) {
    for (const auto& obstacle : obstacles) {
      Vec3 cp_obstacle, cp_grasped;
      double min_d = 0.0;
      const Status st = checkCollision(*obstacle, *grasped_object, &min_d, &cp_obstacle, &cp_grasped);
      if (!ok(st)) {
        log(LogLevel::Error, "getCollisionInfo: collision check failed (grasped object)");
        return st;
      }

      if (min_d < (*dist_array)[total_links]) {
        (*dist_array)[total_links] = min_d;
        contact_normal_array->block<3,1>(0, total_links) = (cp_grasped - cp_obstacle).normalized();
        (*contact_points_array)[total_links] << cp_obstacle, cp_grasped;
      }
    }
  }

  if (check_self_collision) {
    for (int i = 1; i < total_links; ++i) {
      for (int j = 0; j < total_links; ++j) {
        if (j == i || j == i - 1 || j == i + 1) continue;

        const auto& link1 = link_cylinders[i];
        const auto& link2 = link_cylinders[j];
        if (!link1 || !link2) {
          log(LogLevel::Error, "getCollisionInfo: self-collision link null");
          return Status::Failure;
        }
        Vec3 cp_link1, cp_link2;
        double min_d = 0.0;
        const Status st = checkCollision(*link2, *link1, &min_d, &cp_link2, &cp_link1);
        if (!ok(st)) {
          log(LogLevel::Error, "getCollisionInfo: self-collision check failed");
          return st;
        }

        if (min_d < (*dist_array)[i - 1]) {
          (*dist_array)[i - 1] = min_d;
          contact_normal_array->block<3,1>(0, i - 1) = (cp_link1 - cp_link2).normalized();
          (*contact_points_array)[i - 1] << cp_link2, cp_link1;
        }
      }
    }
  }

  for (int i = 0; i < total_links + grasped_obj_con; ++i) {
    const Vec3 contact_point = (*contact_points_array)[i].col(1);
    const int idx = grasped_object && i == total_links
      ? i + num_links_ignore - 1
      : i + num_links_ignore;
    const Status st = getContactJacobian(idx, contact_point, spatial_jacobian, &(*j_contact_array)[i]);
    if (!ok(st)) {
      log(LogLevel::Error, "getCollisionInfo: getContactJacobian failed");
      return st;
    }
  }

  return Status::Success;
}

Status computeContacts(
    const std::vector<std::shared_ptr<ObstacleBase>>& link_cylinders,
    const std::vector<std::shared_ptr<ObstacleBase>>& obstacles,
    const std::shared_ptr<ObstacleBase>& grasped_object,
    const Eigen::MatrixXd& spatial_jacobian,
    const CollisionQueryOptions& opt,
    ContactSet* out) {
  if (!validOut(out, "computeContacts: null output")) return Status::InvalidParameter;

  Eigen::MatrixXd contact_normal_array;
  std::vector<double> dist_array;
  std::vector<Eigen::MatrixXd> contact_points_array;
  std::vector<Eigen::MatrixXd> j_contact_array;

  const Status st = getCollisionInfo(
      link_cylinders,
      obstacles,
      grasped_object,
      spatial_jacobian,
      opt.check_self_collision,
      opt.num_links_ignore,
      opt.dof,
      &contact_normal_array,
      &dist_array,
      &contact_points_array,
      &j_contact_array);
  if (!ok(st)) return st;

  const int grasped_obj_con = grasped_object ? 1 : 0;
  const int total_links = opt.dof - opt.num_links_ignore;
  const int total_contacts = total_links + grasped_obj_con;

  out->contacts.clear();
  out->contacts.reserve(dist_array.size());

  for (int i = 0; i < total_contacts; ++i) {
    Contact contact;
    contact.distance = dist_array[i];
    contact.normal = contact_normal_array.block<3,1>(0, i);

    if (static_cast<size_t>(i) < contact_points_array.size() &&
        contact_points_array[i].cols() == 2) {
      contact.point_obj = contact_points_array[i].col(0);
      contact.point_link = contact_points_array[i].col(1);
    }

    if (static_cast<size_t>(i) < j_contact_array.size()) {
      contact.J_contact = j_contact_array[i];
    }

    if (grasped_object && i == total_links) {
      contact.is_grasped = true;
      contact.link_index = -1;
    } else {
      contact.link_index = i + opt.num_links_ignore;
    }

    out->contacts.push_back(std::move(contact));
  }

  return Status::Success;
}

}  // namespace sclerp::collision
