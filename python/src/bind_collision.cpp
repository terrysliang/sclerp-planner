#include "bind_common.hpp"

#include "sclerp/collision/avoidance.hpp"
#include "sclerp/collision/collision.hpp"
#include "sclerp/collision/motion_plan_collision.hpp"
#include "sclerp/collision/robot_link_meshes.hpp"
#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/math/types.hpp"

#include <filesystem>
#include <memory>
#include <vector>

namespace sclerp::python {
namespace {

using sclerp::collision::CollisionAvoidanceOptions;
using sclerp::collision::CollisionContext;
using sclerp::collision::CollisionMotionPlanOptions;
using sclerp::collision::CollisionQueryOptions;
using sclerp::collision::CollisionScene;
using sclerp::collision::FclObject;
using sclerp::collision::RobotLinkMeshBuildOptions;
using sclerp::collision::RobotLinkMeshSpec;
using sclerp::core::KinematicsSolver;
using sclerp::core::Mat3;
using sclerp::core::Mat4;
using sclerp::core::MotionPlanOptions;
using sclerp::core::MotionPlanRequest;
using sclerp::core::MotionPlanResult;
using sclerp::core::Status;
using sclerp::core::Transform;
using sclerp::core::Vec3;

}  // namespace

void bind_collision(py::module_& m) {
  py::class_<FclObject, std::shared_ptr<FclObject>>(m, "FclObject")
      .def("set_transform", &FclObject::setTransform, py::arg("position"), py::arg("orientation"))
      .def("compute_aabb", &FclObject::computeAABB);

  m.def(
      "create_box",
      [](const Vec3& dimensions, const Vec3& position, const Mat3& orientation) {
        std::shared_ptr<FclObject> out;
        const Status st = sclerp::collision::createBox(dimensions, position, orientation, &out);
        return py::make_tuple(st, out);
      },
      py::arg("dimensions"),
      py::arg("position"),
      py::arg("orientation") = Mat3::Identity());

  py::class_<RobotLinkMeshSpec>(m, "RobotLinkMeshSpec")
      .def(py::init<>())
      .def(py::init<std::vector<std::string>, std::vector<std::string>>(),
           py::arg("frame_names"),
           py::arg("frame_mesh_uris") = std::vector<std::string>{})
      .def_readwrite("frame_names", &RobotLinkMeshSpec::frame_names)
      .def_readwrite("frame_mesh_uris", &RobotLinkMeshSpec::frame_mesh_uris);

  py::class_<RobotLinkMeshBuildOptions>(m, "RobotLinkMeshBuildOptions")
      .def(py::init<>())
      .def_readwrite("tool_placeholder_radius", &RobotLinkMeshBuildOptions::tool_placeholder_radius);

  m.def(
      "build_robot_link_meshes_from_stl_directory",
      [](const KinematicsSolver& solver,
         const std::string& stl_dir,
         const RobotLinkMeshSpec& spec,
         const RobotLinkMeshBuildOptions& opt) {
        std::vector<std::shared_ptr<FclObject>> link_meshes;
        std::vector<Mat4> mesh_offset_transforms;
        const Status st = sclerp::collision::buildRobotLinkMeshesFromStlDirectory(
            solver, std::filesystem::path(stl_dir), spec, &link_meshes, &mesh_offset_transforms, opt);
        return py::make_tuple(st, link_meshes, mesh_offset_transforms);
      },
      py::arg("solver"),
      py::arg("stl_dir"),
      py::arg("spec"),
      py::arg("opt") = RobotLinkMeshBuildOptions());

  py::class_<CollisionQueryOptions>(m, "CollisionQueryOptions")
      .def(py::init<>())
      .def_readwrite("check_self_collision", &CollisionQueryOptions::check_self_collision)
      .def_readwrite("num_links_ignore", &CollisionQueryOptions::num_links_ignore)
      .def_readwrite("use_obstacle_broadphase", &CollisionQueryOptions::use_obstacle_broadphase);

  py::class_<CollisionAvoidanceOptions>(m, "CollisionAvoidanceOptions")
      .def(py::init<>())
      .def_readwrite("safe_dist", &CollisionAvoidanceOptions::safe_dist)
      .def_readwrite("comp_activate_tol", &CollisionAvoidanceOptions::comp_activate_tol)
      .def_readwrite("dt", &CollisionAvoidanceOptions::dt)
      .def_readwrite("pinv_lambda", &CollisionAvoidanceOptions::pinv_lambda);

  py::class_<CollisionMotionPlanOptions>(m, "CollisionMotionPlanOptions")
      .def(py::init<>())
      .def_readwrite("motion", &CollisionMotionPlanOptions::motion)
      .def_readwrite("query", &CollisionMotionPlanOptions::query)
      .def_readwrite("avoidance", &CollisionMotionPlanOptions::avoidance);

  m.def(
      "plan_motion_sclerp_with_collision",
      [](const KinematicsSolver& solver,
         const Eigen::VectorXd& q_init,
         const Mat4& g_f,
         const std::vector<std::shared_ptr<FclObject>>& link_meshes,
         const std::vector<std::shared_ptr<FclObject>>& obstacles,
         const std::vector<Mat4>& mesh_offset_transforms,
         const CollisionMotionPlanOptions& opt) {
        MotionPlanResult out;
        Transform g_i = Transform::Identity();
        const Status st_fk = solver.forwardKinematics(q_init, &g_i);
        if (!sclerp::core::ok(st_fk)) {
          out.status = st_fk;
          return out;
        }

        MotionPlanRequest req;
        req.q_init = q_init;
        req.g_i = g_i;
        req.g_f = sclerp::core::transformFromMatrix4(g_f);

        std::shared_ptr<FclObject> grasped_object;  // none
        const CollisionContext ctx{link_meshes, obstacles, grasped_object};
        CollisionScene scene{ctx, mesh_offset_transforms};
        return sclerp::collision::planMotionSclerpWithCollision(solver, req, scene, opt);
      },
      py::arg("solver"),
      py::arg("q_init"),
      py::arg("g_f"),
      py::arg("link_meshes"),
      py::arg("obstacles"),
      py::arg("mesh_offset_transforms"),
      py::arg("opt") = CollisionMotionPlanOptions());
}

}  // namespace sclerp::python
