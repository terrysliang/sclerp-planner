#include "sclerp/collision/collision.hpp"

#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/math/so3.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <algorithm>
#include <cmath>

namespace sclerp::collision {

using sclerp::core::LogLevel;
using sclerp::core::log;

static inline bool validOut(const void* out, const char* msg) {
  if (out) return true;
  log(LogLevel::Error, msg);
  return false;
}

static inline Vec3 safeNormal(const Vec3& from, const Vec3& to) {
  const Vec3 diff = to - from;
  const double n = diff.norm();
  if (n > 1e-12) return diff / n;
  return Vec3::Zero();
}

FclObject::FclObject(std::shared_ptr<fcl::CollisionGeometryd> geometry,
                         const Vec3& position,
                         const Mat3& orientation)
  : geometry_(std::move(geometry)),
    collision_object_(geometry_, fcl::Transform3d::Identity()) {
  setTransform(position, orientation);
}

const fcl::CollisionObjectd& FclObject::collisionObject() const {
  return collision_object_;
}

void FclObject::setTransform(const Vec3& position, const Mat3& orientation) {
  collision_object_.setTranslation(position);
  collision_object_.setRotation(orientation);
}

void FclObject::computeAABB() {
  collision_object_.computeAABB();
}

BoxObject::BoxObject(const Vec3& dimensions,
                     const Vec3& position,
                     const Mat3& orientation)
  : FclObject(std::make_shared<fcl::Boxd>(dimensions.x(), dimensions.y(), dimensions.z()),
                position,
                orientation) {}

SphereObject::SphereObject(double radius,
                           const Vec3& position,
                           const Mat3& orientation)
  : FclObject(std::make_shared<fcl::Sphered>(radius), position, orientation) {}

CylinderObject::CylinderObject(double radius,
                               double height,
                               const Vec3& position,
                               const Mat3& orientation)
  : FclObject(std::make_shared<fcl::Cylinderd>(radius, height), position, orientation) {}

MeshObject::MeshObject(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>>& bvh_model,
                       const Vec3& position,
                       const Mat3& orientation)
  : FclObject(bvh_model, position, orientation) {}

PlaneObject::PlaneObject(const Vec3& normal, double offset)
  : FclObject(std::make_shared<fcl::Planed>(normal, offset),
              Vec3::Zero(),
              Mat3::Identity()) {}

Status createBox(const Vec3& dimensions,
                 const Vec3& position,
                 const Mat3& orientation,
                 std::shared_ptr<FclObject>* out) {
  if (!validOut(out, "createBox: null output")) return Status::InvalidParameter;
  *out = std::make_shared<BoxObject>(dimensions, position, orientation);
  return Status::Success;
}

Status createSphere(double radius,
                    const Vec3& position,
                    const Mat3& orientation,
                    std::shared_ptr<FclObject>* out) {
  if (!validOut(out, "createSphere: null output")) return Status::InvalidParameter;
  if (!(radius > 0.0)) {
    log(LogLevel::Error, "createSphere: radius must be > 0");
    return Status::InvalidParameter;
  }
  *out = std::make_shared<SphereObject>(radius, position, orientation);
  return Status::Success;
}

Status createCylinder(double radius,
                      double height,
                      const Vec3& position,
                      const Mat3& orientation,
                      std::shared_ptr<FclObject>* out) {
  if (!validOut(out, "createCylinder: null output")) return Status::InvalidParameter;
  if (!(radius > 0.0) || !(height > 0.0)) {
    log(LogLevel::Error, "createCylinder: radius/height must be > 0");
    return Status::InvalidParameter;
  }
  *out = std::make_shared<CylinderObject>(radius, height, position, orientation);
  return Status::Success;
}

Status createPlane(const Vec3& normal,
                   double offset,
                   std::shared_ptr<FclObject>* out) {
  if (!validOut(out, "createPlane: null output")) return Status::InvalidParameter;
  const double n_norm = normal.norm();
  if (!(n_norm > 0.0)) {
    log(LogLevel::Error, "createPlane: normal must be non-zero");
    return Status::InvalidParameter;
  }
  *out = std::make_shared<PlaneObject>(normal / n_norm, offset);
  return Status::Success;
}

Status removeObstacle(std::vector<std::shared_ptr<FclObject>>& obstacles, size_t index) {
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
                         std::shared_ptr<FclObject>* out) {
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
                         std::shared_ptr<FclObject>* out) {
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
  *out = std::make_shared<MeshObject>(bvh_model, position, orientation);
  return Status::Success;
}

Status buildLinkMeshes(const std::vector<std::string>& stl_files,
                       std::vector<std::shared_ptr<FclObject>>* out) {
  if (!validOut(out, "buildLinkMeshes: null output")) return Status::InvalidParameter;

  std::vector<std::shared_ptr<FclObject>> link_meshes;
  link_meshes.reserve(stl_files.size());
  for (const auto& mesh_path : stl_files) {
    std::shared_ptr<FclObject> mesh_obj;
    const Status st = createMeshFromSTL(mesh_path, Mat4::Identity(), &mesh_obj);
    if (!ok(st)) return st;
    link_meshes.push_back(std::move(mesh_obj));
  }

  *out = std::move(link_meshes);
  return Status::Success;
}

Status updateLinkMeshTransforms(std::vector<std::shared_ptr<FclObject>>& link_meshes,
                                const std::vector<Mat4>& g_intermediate,
                                const std::vector<Mat4>& mesh_offset_transforms) {
  if (link_meshes.size() != g_intermediate.size()) {
    log(LogLevel::Error, "updateLinkMeshTransforms: size mismatch");
    return Status::InvalidParameter;
  }
  if (mesh_offset_transforms.size() != g_intermediate.size()) {
    log(LogLevel::Error, "updateLinkMeshTransforms: mesh offset size mismatch");
    return Status::InvalidParameter;
  }

  for (size_t i = 0; i < link_meshes.size(); ++i) {
    if (!link_meshes[i]) continue;
    const Mat4 T_world_to_mesh = g_intermediate[i] * mesh_offset_transforms[i];
    const Vec3 position = T_world_to_mesh.block<3,1>(0, 3);
    const Mat3 rotation = T_world_to_mesh.block<3,3>(0, 0);
    link_meshes[i]->setTransform(position, rotation);
  }
  return Status::Success;
}

Status checkCollision(const FclObject& obj1,
                      const FclObject& obj2,
                      double* min_dist,
                      Vec3* contact_point_obj1,
                      Vec3* contact_point_obj2) {
  if (!validOut(min_dist, "checkCollision: null min_dist")) return Status::InvalidParameter;
  if (!validOut(contact_point_obj1, "checkCollision: null contact_point_obj1")) return Status::InvalidParameter;
  if (!validOut(contact_point_obj2, "checkCollision: null contact_point_obj2")) return Status::InvalidParameter;

  fcl::DistanceRequestd request;
  request.enable_nearest_points = true;
  // Keep signed distance disabled here: FCL's signed-distance path with
  // GST_INDEP can assert on some mesh/contact cases. We recover penetration
  // depth robustly below via an explicit collision query.
  request.enable_signed_distance = false;
  request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
  request.distance_tolerance = 1e-6;
  fcl::DistanceResultd result;

  *min_dist = fcl::distance(&obj1.collisionObject(),
                            &obj2.collisionObject(),
                            request,
                            result);
  if (std::isnan(*min_dist)) {
    log(LogLevel::Error, "checkCollision: min_dist is NaN");
    return Status::Failure;
  }

  *contact_point_obj1 = result.nearest_points[0];
  *contact_point_obj2 = result.nearest_points[1];

  if (*min_dist < 0.0) {
    fcl::CollisionRequestd collision_request;
    collision_request.enable_contact = true;
    collision_request.num_max_contacts = 16;
    collision_request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    fcl::CollisionResultd collision_result;
    fcl::collide(&obj1.collisionObject(),
                 &obj2.collisionObject(),
                 collision_request,
                 collision_result);

    double max_depth = 0.0;
    const std::size_t num_contacts = collision_result.numContacts();
    for (std::size_t i = 0; i < num_contacts; ++i) {
      const auto& contact = collision_result.getContact(i);
      if (std::isfinite(contact.penetration_depth)) {
        max_depth = std::max(max_depth, contact.penetration_depth);
      }
    }

    if (max_depth > 0.0 && std::isfinite(max_depth)) {
      *min_dist = -max_depth;
      if (num_contacts > 0) {
        const auto& contact = collision_result.getContact(0);
        const Vec3 normal = contact.normal;
        const Vec3 point = contact.pos;
        *contact_point_obj1 = point - 0.5 * max_depth * normal;
        *contact_point_obj2 = point + 0.5 * max_depth * normal;
      }
    } else if (*min_dist < -1e-6) {
      // Fallback guard against implementation-defined signed distance (often -1).
      *min_dist = -1e-6;
    }
    log(LogLevel::Warn, "checkCollision: penetration detected");
  }

  return Status::Success;
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

static Status computeContactArrays(
    const std::vector<std::shared_ptr<FclObject>>& link_cylinders,
    const std::vector<std::shared_ptr<FclObject>>& obstacles,
    const std::shared_ptr<FclObject>& grasped_object,
    const Eigen::MatrixXd& spatial_jacobian,
    bool check_self_collision,
    int num_links_ignore,
    int dof,
    Eigen::MatrixXd* contact_normal_array,
    std::vector<double>* dist_array,
    std::vector<Eigen::MatrixXd>* contact_points_array,
    std::vector<Eigen::MatrixXd>* j_contact_array) {
  if (!validOut(contact_normal_array, "computeContacts: null contact_normal_array")) return Status::InvalidParameter;
  if (!validOut(dist_array, "computeContacts: null dist_array")) return Status::InvalidParameter;
  if (!validOut(contact_points_array, "computeContacts: null contact_points_array")) return Status::InvalidParameter;
  if (!validOut(j_contact_array, "computeContacts: null j_contact_array")) return Status::InvalidParameter;

  const int grasped_obj_con = grasped_object ? 1 : 0;
  const int total_links = dof - num_links_ignore;
  if (total_links <= 0) {
    log(LogLevel::Error, "computeContacts: invalid total_links");
    return Status::InvalidParameter;
  }

  const int total_contacts = total_links + grasped_obj_con;
  dist_array->assign(total_contacts, 1000.0);
  *contact_normal_array = Eigen::MatrixXd::Zero(6, total_contacts);
  contact_points_array->assign(total_contacts, Eigen::MatrixXd::Zero(3, 2));
  j_contact_array->assign(total_contacts,
                          Eigen::MatrixXd::Zero(6, spatial_jacobian.cols()));
  std::vector<bool> has_contact(static_cast<size_t>(total_contacts), false);
  for (const auto& obstacle : obstacles) {
    if (!obstacle) {
      log(LogLevel::Error, "computeContacts: obstacle is null");
      return Status::InvalidParameter;
    }
    for (int itr_index = 1; itr_index <= total_links; ++itr_index) {
      const int num_link = num_links_ignore + itr_index;
      if (num_link < 0 || static_cast<size_t>(num_link) >= link_cylinders.size()) {
        log(LogLevel::Error, "computeContacts: link cylinder index out of range");
        return Status::InvalidParameter;
      }
      const auto& current_cylinder = link_cylinders[num_link];
      if (!current_cylinder) {
        log(LogLevel::Error, "computeContacts: link cylinder is null");
        return Status::Failure;
      }

      Vec3 cp_obj, cp_cylinder;
      double min_d = 0.0;
      const Status st = checkCollision(*obstacle,
                                       *current_cylinder,
                                       &min_d,
                                       &cp_obj,
                                       &cp_cylinder);
      if (!ok(st)) {
        log(LogLevel::Error, "computeContacts: collision check failed (environment)");
        return st;
      }

      if (min_d < (*dist_array)[itr_index - 1]) {
        (*dist_array)[itr_index - 1] = min_d;
        contact_normal_array->block<3,1>(0, itr_index - 1) = safeNormal(cp_obj, cp_cylinder);
        (*contact_points_array)[itr_index - 1] << cp_obj, cp_cylinder;
        has_contact[static_cast<size_t>(itr_index - 1)] = true;
      }
    }
  }

  if (grasped_object) {
    for (const auto& obstacle : obstacles) {
      if (!obstacle) {
        log(LogLevel::Error, "computeContacts: obstacle is null");
        return Status::InvalidParameter;
      }
      Vec3 cp_obstacle, cp_grasped;
      double min_d = 0.0;
      const Status st = checkCollision(*obstacle,
                                       *grasped_object,
                                       &min_d,
                                       &cp_obstacle,
                                       &cp_grasped);
      if (!ok(st)) {
        log(LogLevel::Error, "computeContacts: collision check failed (grasped object)");
        return st;
      }

      if (min_d < (*dist_array)[total_links]) {
        (*dist_array)[total_links] = min_d;
        contact_normal_array->block<3,1>(0, total_links) = safeNormal(cp_obstacle, cp_grasped);
        (*contact_points_array)[total_links] << cp_obstacle, cp_grasped;
        has_contact[static_cast<size_t>(total_links)] = true;
      }
    }
  }

  if (check_self_collision) {
    // Self-collision indices are in "active link" space:
    // active itr_index 1 maps to cylinder num_links_ignore + 1.
    for (int itr_index = 1; itr_index <= total_links; ++itr_index) {
      const int moving_link_index = num_links_ignore + itr_index;
      if (moving_link_index < 0 || static_cast<size_t>(moving_link_index) >= link_cylinders.size()) {
        log(LogLevel::Error, "computeContacts: moving link index out of range");
        return Status::InvalidParameter;
      }
      const auto& link1 = link_cylinders[moving_link_index];
      if (!link1) {
        log(LogLevel::Error, "computeContacts: self-collision moving link is null");
        return Status::Failure;
      }

      const bool is_last_link = (itr_index == total_links);
      for (int obs_itr = 0; obs_itr <= total_links; ++obs_itr) {
        const int obstacle_link_index = (obs_itr == 0) ? 0 : (num_links_ignore + obs_itr);
        if (obstacle_link_index < 0 || static_cast<size_t>(obstacle_link_index) >= link_cylinders.size()) {
          log(LogLevel::Error, "computeContacts: obstacle link index out of range");
          return Status::InvalidParameter;
        }

        if (obstacle_link_index == moving_link_index) {
          continue;
        }
        if (!is_last_link &&
            (obs_itr == itr_index - 1 || obs_itr == itr_index || obs_itr == itr_index + 1)) {
          continue;
        }

        const auto& link2 = link_cylinders[obstacle_link_index];
        if (!link2) {
          log(LogLevel::Error, "computeContacts: self-collision obstacle link is null");
          return Status::Failure;
        }
        Vec3 cp_link1, cp_link2;
        double min_d = 0.0;
        const Status st = checkCollision(*link2,
                                         *link1,
                                         &min_d,
                                         &cp_link2,
                                         &cp_link1);
        if (!ok(st)) {
          log(LogLevel::Error, "computeContacts: self-collision check failed");
          return st;
        }

        if (min_d < (*dist_array)[itr_index - 1]) {
          (*dist_array)[itr_index - 1] = min_d;
          contact_normal_array->block<3,1>(0, itr_index - 1) = safeNormal(cp_link2, cp_link1);
          (*contact_points_array)[itr_index - 1] << cp_link2, cp_link1;
          has_contact[static_cast<size_t>(itr_index - 1)] = true;
        }
      }
    }
  }

  for (int i = 0; i < total_contacts; ++i) {
    if (!has_contact[static_cast<size_t>(i)]) {
      continue;
    }
    const Vec3 contact_point = (*contact_points_array)[i].col(1);
    const int idx = grasped_object && i == total_links
      ? i + num_links_ignore - 1
      : i + num_links_ignore;
    const Status st = getContactJacobian(idx, contact_point, spatial_jacobian, &(*j_contact_array)[i]);
    if (!ok(st)) {
      log(LogLevel::Error, "computeContacts: getContactJacobian failed");
      return st;
    }
  }

  return Status::Success;
}

Status computeContacts(const CollisionContext& ctx,
                       const CollisionQueryOptions& opt,
                       ContactSet* out) {
  if (!validOut(out, "computeContacts: null output")) return Status::InvalidParameter;

  if (ctx.spatial_jacobian.rows() != 6) {
    log(LogLevel::Error, "computeContacts: spatial_jacobian must have 6 rows");
    return Status::InvalidParameter;
  }
  const int dof = static_cast<int>(ctx.spatial_jacobian.cols());
  if (dof <= 0) {
    log(LogLevel::Error, "computeContacts: invalid dof");
    return Status::InvalidParameter;
  }
  if (ctx.link_cylinders.size() < static_cast<size_t>(dof + 1)) {
    log(LogLevel::Error, "computeContacts: insufficient link objects for dof");
    return Status::InvalidParameter;
  }

  Eigen::MatrixXd contact_normal_array;
  std::vector<double> dist_array;
  std::vector<Eigen::MatrixXd> contact_points_array;
  std::vector<Eigen::MatrixXd> j_contact_array;

  const Status st = computeContactArrays(
      ctx.link_cylinders,
      ctx.obstacles,
      ctx.grasped_object,
      ctx.spatial_jacobian,
      opt.check_self_collision,
      opt.num_links_ignore,
      dof,
      &contact_normal_array,
      &dist_array,
      &contact_points_array,
      &j_contact_array);
  if (!ok(st)) return st;

  const bool has_grasped = static_cast<bool>(ctx.grasped_object);
  const int grasped_obj_con = has_grasped ? 1 : 0;
  const int total_links = dof - opt.num_links_ignore;
  const int total_contacts = total_links + grasped_obj_con;
  if (total_links <= 0 || total_contacts < 0) {
    log(LogLevel::Error, "computeContacts: invalid contact dimensions");
    return Status::InvalidParameter;
  }

  out->contacts.clear();
  out->contacts.reserve(static_cast<size_t>(total_contacts));

  for (int i = 0; i < total_contacts; ++i) {
    Contact contact;
    if (static_cast<size_t>(i) < dist_array.size()) {
      contact.distance = dist_array[static_cast<size_t>(i)];
    }
    if (i < contact_normal_array.cols()) {
      contact.normal = contact_normal_array.block<3,1>(0, i);
    }

    if (static_cast<size_t>(i) < contact_points_array.size() &&
        contact_points_array[static_cast<size_t>(i)].cols() == 2) {
      contact.point_obj = contact_points_array[static_cast<size_t>(i)].col(0);
      contact.point_link = contact_points_array[static_cast<size_t>(i)].col(1);
    }

    if (static_cast<size_t>(i) < j_contact_array.size()) {
      contact.J_contact = j_contact_array[static_cast<size_t>(i)];
    }

    if (has_grasped && i == total_links) {
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
