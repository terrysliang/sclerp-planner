// Collision/contact extraction using FCL.
//
// Design notes:
// - We compute a nearest contact per active link-slot (and optional grasped object slot) to keep
//   the avoidance LCP small and predictable.
// - We do not use signed distance; penetrations are treated as distance 0 with a robust normal.
// - When nearest points are degenerate near touch, we fall back to an FCL collide() query to
//   recover a usable contact normal for push-out.
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

static inline int activeSlotToCylinderIndex(int slot, int num_links_ignore) {
  // slot 0 corresponds to link1 (link mesh index 1) if num_links_ignore == 0
  // base is index 0 and is always excluded from env collision checks
  return (1 + num_links_ignore) + slot;  // in [1 .. dof]
}

static inline int cylinderIndexToSolverLinkIndex(int cylinder_index) {
  // cylinder index 1..dof => solver link_index 0..dof-1
  return cylinder_index - 1;
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

Status updateLinkMeshTransforms(const std::vector<std::shared_ptr<FclObject>>& link_meshes,
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
  request.enable_signed_distance = false; // keep as you had
  request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
  request.distance_tolerance = 1e-6;
  fcl::DistanceResultd result;

  double d = fcl::distance(&obj1.collisionObject(),
                           &obj2.collisionObject(),
                           request,
                           result);
  if (!std::isfinite(d)) {
    log(LogLevel::Error, "checkCollision: distance is NaN/Inf");
    return Status::Failure;
  }

  Vec3 p1 = result.nearest_points[0];
  Vec3 p2 = result.nearest_points[1];

  // if distance is ~0, we need a robust normal (avoid safeNormal=0).
  const double kNearTol = 1e-9;
  const bool points_degenerate = ((p2 - p1).squaredNorm() <= 1e-24); // ~1e-12 meters

  if (d <= kNearTol || points_degenerate) {
    // Use collision query to get a stable normal in touch/penetration
    fcl::CollisionRequestd creq;
    creq.enable_contact = true;
    creq.num_max_contacts = 16;
    creq.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    fcl::CollisionResultd cres;

    fcl::collide(&obj1.collisionObject(),
                 &obj2.collisionObject(),
                 creq,
                 cres);

    Vec3 n = Vec3::Zero();
    Vec3 pos = Vec3::Zero();

    if (cres.numContacts() > 0) {
      const auto& c = cres.getContact(0);
      n = c.normal;
      pos = c.pos;

      double nn = n.norm();
      if (!std::isfinite(nn) || nn < 1e-12) {
        n = Vec3::Zero();
      } else {
        n /= nn;
      }
    }

    if (n.norm() < 1e-12) {
      // Fallback direction: center-to-center (never zero unless identical transforms)
      const auto t1 = obj1.collisionObject().getTranslation();
      const auto t2 = obj2.collisionObject().getTranslation();
      Vec3 dir = t2 - t1;
      const double nd = dir.norm();
      if (nd > 1e-12) {
        n = dir / nd;
      } else {
        n = Vec3::UnitX();
      }
      pos = 0.5 * (t1 + t2);
    }

    // Manufacture two distinct points separated by eps along n
    constexpr double eps = 1e-6; // 1 micron; enough to make safeNormal non-zero
    p1 = pos - eps * n; // on obj1 side
    p2 = pos + eps * n; // on obj2 side

    d = 0.0; // clamp to non-negative; penetration depth not returned
  }

  *min_dist = std::max(d, 0.0);
  *contact_point_obj1 = p1;
  *contact_point_obj2 = p2;
  return Status::Success;
}

static Status computeContactArrays(
    const sclerp::core::KinematicsSolver& solver,
    const Eigen::VectorXd& q,
    const std::vector<std::shared_ptr<FclObject>>& link_meshes,  // includes base at 0
    const std::vector<std::shared_ptr<FclObject>>& obstacles,
    const std::shared_ptr<FclObject>& grasped_object,
    bool check_self_collision,
    int num_links_ignore,
    int dof,
    Eigen::MatrixXd* contact_normal_array_3xN,
    std::vector<double>* dist_array,
    std::vector<Eigen::MatrixXd>* contact_points_array_3x2,
    std::vector<Eigen::MatrixXd>* j_contact_array_3xk,   // reduced (3×k)
    std::vector<bool>* has_contact
) {
  if (!validOut(contact_normal_array_3xN, "computeContactArrays: null contact_normal_array")) return Status::InvalidParameter;
  if (!validOut(dist_array, "computeContactArrays: null dist_array")) return Status::InvalidParameter;
  if (!validOut(contact_points_array_3x2, "computeContactArrays: null contact_points_array")) return Status::InvalidParameter;
  if (!validOut(j_contact_array_3xk, "computeContactArrays: null j_contact_array")) return Status::InvalidParameter;
  if (!validOut(has_contact, "computeContactArrays: null has_contact")) return Status::InvalidParameter;

  if (solver.model().dof() != dof || q.size() != dof) {
    log(LogLevel::Error, "computeContactArrays: solver/q dof mismatch");
    return Status::InvalidParameter;
  }

  const int grasped_obj_con = grasped_object ? 1 : 0;
  const int total_links = dof - num_links_ignore;
  if (total_links <= 0) {
    log(LogLevel::Error, "computeContactArrays: invalid total_links");
    return Status::InvalidParameter;
  }
  const int total_contacts = total_links + grasped_obj_con;

  // Need at least base + dof link meshes: indices [0..dof]
  if (static_cast<int>(link_meshes.size()) < dof + 1) {
    log(LogLevel::Error, "computeContactArrays: link_meshes size insufficient");
    return Status::InvalidParameter;
  }

  // Init
  dist_array->assign(total_contacts, 1e9);  // far away
  *contact_normal_array_3xN = Eigen::MatrixXd::Zero(3, total_contacts);
  contact_points_array_3x2->assign(total_contacts, Eigen::MatrixXd::Zero(3, 2));
  j_contact_array_3xk->clear();
  j_contact_array_3xk->resize(static_cast<size_t>(total_contacts));
  has_contact->assign(static_cast<size_t>(total_contacts), false);

  // Pre-size reduced Jacobians (keep zero if we never get a closest point)
  for (int slot = 0; slot < total_links; ++slot) {
    const int cyl_idx = activeSlotToCylinderIndex(slot, num_links_ignore); // 1..dof
    const int link_index = cylinderIndexToSolverLinkIndex(cyl_idx);        // 0..dof-1
    (*j_contact_array_3xk)[static_cast<size_t>(slot)] = Eigen::MatrixXd::Zero(3, link_index + 1);
  }
  if (grasped_object) {
    (*j_contact_array_3xk)[static_cast<size_t>(total_links)] = Eigen::MatrixXd::Zero(3, dof);
  }

  // Helper: accept an update only if we get a valid normal (avoid infeasible LCP rows)
  auto tryUpdateSlot = [&](int slot, double d, const Vec3& p_obj, const Vec3& p_link) {
    d = std::max(d, 0.0);
    if (d >= (*dist_array)[slot]) return;

    const Vec3 n = safeNormal(p_obj, p_link);
    if (n.squaredNorm() <= 1e-24) {
      // Degenerate nearest points: don't lock in a zero normal.
      return;
    }

    (*dist_array)[slot] = d;
    contact_normal_array_3xN->col(slot) = n;
    (*contact_points_array_3x2)[slot] << p_obj, p_link;
    (*has_contact)[static_cast<size_t>(slot)] = true;
  };

  // Environment collision checks
  for (const auto& obstacle : obstacles) {
    if (!obstacle) return Status::InvalidParameter;

    for (int slot = 0; slot < total_links; ++slot) {
      const int cyl_idx = activeSlotToCylinderIndex(slot, num_links_ignore); // 1..dof
      const auto& cyl = link_meshes[cyl_idx];
      if (!cyl) return Status::Failure;

      Vec3 cp_obj, cp_link;
      double min_d = 0.0;
      const Status st = checkCollision(*obstacle, *cyl, &min_d, &cp_obj, &cp_link);
      if (!ok(st)) return st;

      tryUpdateSlot(slot, min_d, cp_obj, cp_link);
    }
  }

  // Grasped object vs. environment
  if (grasped_object) {
    const int grasp_slot = total_links;

    for (const auto& obstacle : obstacles) {
      if (!obstacle) return Status::InvalidParameter;

      Vec3 cp_obj, cp_grasp;
      double min_d = 0.0;
      const Status st = checkCollision(*obstacle, *grasped_object, &min_d, &cp_obj, &cp_grasp);
      if (!ok(st)) return st;

      tryUpdateSlot(grasp_slot, min_d, cp_obj, cp_grasp);
    }
  }

  // Self-collision (optional)
  if (check_self_collision) {
    for (int slot = 0; slot < total_links; ++slot) {
      const int cyl_idx1 = activeSlotToCylinderIndex(slot, num_links_ignore); // moving
      const auto& link1 = link_meshes[cyl_idx1];
      if (!link1) return Status::Failure;

      const bool is_last = (slot == total_links - 1);

      // Compare against base(0) and other active links.
      for (int obs_slot = -1; obs_slot < total_links; ++obs_slot) {
        const int cyl_idx2 =
            (obs_slot < 0) ? 0 : activeSlotToCylinderIndex(obs_slot, num_links_ignore);
        if (cyl_idx2 == cyl_idx1) continue;

        // Skip adjacent links (same policy as your original; keep if you like)
        if (!is_last && obs_slot >= 0) {
          if (obs_slot == slot - 1 || obs_slot == slot || obs_slot == slot + 1) continue;
        }

        const auto& link2 = link_meshes[cyl_idx2];
        if (!link2) return Status::Failure;

        Vec3 cp2, cp1;
        double min_d = 0.0;
        const Status st = checkCollision(*link2, *link1, &min_d, &cp2, &cp1);
        if (!ok(st)) return st;

        // Here: p_obj is on link2, p_link is on link1 (moving link)
        tryUpdateSlot(slot, min_d, cp2, cp1);
      }
    }
  }

  // Per-contact reduced point Jacobians using solver
  for (int slot = 0; slot < total_links; ++slot) {
    if (!(*has_contact)[static_cast<size_t>(slot)]) {
      continue; // keep zero Jacobian; distance is huge
    }
    if (contact_normal_array_3xN->col(slot).squaredNorm() <= 1e-24) {
      continue; // extra guard
    }

    const int cyl_idx = activeSlotToCylinderIndex(slot, num_links_ignore); // 1..dof
    const int link_index = cylinderIndexToSolverLinkIndex(cyl_idx);        // 0..dof-1

    const Vec3 p_link = (*contact_points_array_3x2)[slot].col(1); // point on robot link
    Eigen::MatrixXd Jp(3, link_index + 1);
    const Status st = solver.pointJacobianUpToLink(q, link_index, p_link, Jp);
    if (!ok(st)) return st;

    (*j_contact_array_3xk)[static_cast<size_t>(slot)] = std::move(Jp);
  }

  if (grasped_object) {
    const int grasp_slot = total_links;
    if ((*has_contact)[static_cast<size_t>(grasp_slot)] &&
        contact_normal_array_3xN->col(grasp_slot).squaredNorm() > 1e-24) {

      const Vec3 p_grasp = (*contact_points_array_3x2)[grasp_slot].col(1); // point on grasped object
      // Treat grasped object as attached to last joint: link_index = dof-1 => k=dof
      Eigen::MatrixXd Jp(3, dof);
      const Status st = solver.pointJacobianUpToLink(q, dof - 1, p_grasp, Jp);
      if (!ok(st)) return st;

      (*j_contact_array_3xk)[static_cast<size_t>(grasp_slot)] = std::move(Jp);
    }
  }

  return Status::Success;
}


Status computeContacts(
    const sclerp::core::KinematicsSolver& solver,
    const Eigen::VectorXd& q,
    const CollisionContext& ctx,
    const CollisionQueryOptions& opt,
    ContactSet* out) {
  if (!validOut(out, "computeContacts: null output")) return Status::InvalidParameter;

  const int dof = solver.model().dof();
  if (dof <= 0) return Status::InvalidParameter;
  if (q.size() != dof) return Status::InvalidParameter;

  if (opt.num_links_ignore < 0 || opt.num_links_ignore >= dof) return Status::InvalidParameter;
  if (static_cast<int>(ctx.link_meshes.size()) < dof + 1) {
    log(LogLevel::Error, "computeContacts: link_meshes size insufficient");
    return Status::InvalidParameter;
  }

  Eigen::MatrixXd normals_3xN;
  std::vector<double> dists;
  std::vector<Eigen::MatrixXd> points_3x2;
  std::vector<Eigen::MatrixXd> Jc_3xk;
  std::vector<bool> has_contact;

  const Status st = computeContactArrays(
      solver,
      q,
      ctx.link_meshes,
      ctx.obstacles,
      ctx.grasped_object,
      opt.check_self_collision,
      opt.num_links_ignore,
      dof,
      &normals_3xN,
      &dists,
      &points_3x2,
      &Jc_3xk,
      &has_contact);
  if (!ok(st)) return st;

  const bool has_grasped = static_cast<bool>(ctx.grasped_object);
  const int grasped_obj_con = has_grasped ? 1 : 0;
  const int total_links = dof - opt.num_links_ignore;
  const int total_contacts = total_links + grasped_obj_con;

  if (static_cast<int>(dists.size()) != total_contacts ||
      normals_3xN.cols() != total_contacts ||
      static_cast<int>(points_3x2.size()) != total_contacts ||
      static_cast<int>(Jc_3xk.size()) != total_contacts) {
    log(LogLevel::Error, "computeContacts: returned array size mismatch");
    return Status::Failure;
  }

  out->contacts.clear();
  out->contacts.reserve(static_cast<size_t>(total_contacts));

  // Robot link contact slots
  for (int slot = 0; slot < total_links; ++slot) {
    const int cyl_idx = activeSlotToCylinderIndex(slot, opt.num_links_ignore); // 1..dof
    const int link_index = cylinderIndexToSolverLinkIndex(cyl_idx);            // 0..dof-1

    Contact c;
    c.link_index = link_index;
    c.is_grasped = false;
    c.distance = std::max(dists[static_cast<size_t>(slot)], 0.0);
    c.normal = normals_3xN.col(slot);
    c.point_obj = points_3x2[static_cast<size_t>(slot)].col(0);
    c.point_link = points_3x2[static_cast<size_t>(slot)].col(1);
    c.J_contact = Jc_3xk[static_cast<size_t>(slot)]; // 3×(link_index+1), or zeros if no_contact
    out->contacts.push_back(std::move(c));
  }

  // Grasped object slot (optional)
  if (has_grasped) {
    const int slot = total_links;

    Contact c;
    c.link_index = -1;
    c.is_grasped = true;
    c.distance = std::max(dists[static_cast<size_t>(slot)], 0.0);
    c.normal = normals_3xN.col(slot);
    c.point_obj = points_3x2[static_cast<size_t>(slot)].col(0);
    c.point_link = points_3x2[static_cast<size_t>(slot)].col(1);
    c.J_contact = Jc_3xk[static_cast<size_t>(slot)]; // 3×dof, or zeros if no_contact
    out->contacts.push_back(std::move(c));
  }

  return Status::Success;
}

}  // namespace sclerp::collision
