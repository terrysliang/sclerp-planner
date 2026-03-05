#include "bind_common.hpp"

#include "sclerp/core/common/constants.hpp"
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/dual_quat/dual_quat.hpp"
#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/math/types.hpp"
#include "sclerp/core/model/manipulator_model.hpp"
#include "sclerp/core/path/joint_path.hpp"
#include "sclerp/core/planning/motion_plan.hpp"

#include <Eigen/Core>

namespace sclerp::python {
namespace {

using sclerp::core::JointLimit;
using sclerp::core::JointSpec;
using sclerp::core::JointType;
using sclerp::core::KinematicsSolver;
using sclerp::core::ManipulatorModel;
using sclerp::core::Mat4;
using sclerp::core::MotionPlanOptions;
using sclerp::core::MotionPlanRequest;
using sclerp::core::MotionPlanResult;
using sclerp::core::RmrcDamping;
using sclerp::core::RmrcOptions;
using sclerp::core::Status;
using sclerp::core::Thresholds;
using sclerp::core::Transform;
using sclerp::core::DualQuat;
using sclerp::core::JointPath;
using sclerp::core::FailureLogOptions;

}  // namespace

void bind_core(py::module_& m) {
  py::enum_<Status>(m, "Status")
      .value("Success", Status::Success)
      .value("Failure", Status::Failure)
      .value("JointLimit", Status::JointLimit)
      .value("InvalidParameter", Status::InvalidParameter);

  m.def("ok", &sclerp::core::ok, py::arg("status"));

  py::class_<Thresholds>(m, "Thresholds")
      .def(py::init<>())
      .def_readwrite("pure_translation_rot_angle", &Thresholds::pure_translation_rot_angle)
      .def_readwrite("pure_rotation_pitch", &Thresholds::pure_rotation_pitch)
      .def_readwrite("no_motion_magnitude", &Thresholds::no_motion_magnitude)
      .def_readwrite("axis_norm_eps", &Thresholds::axis_norm_eps);

  m.attr("k_default_thresholds") = py::cast(sclerp::core::kDefaultThresholds);

  py::enum_<JointType>(m, "JointType")
      .value("Revolute", JointType::Revolute)
      .value("Prismatic", JointType::Prismatic)
      .value("Fixed", JointType::Fixed);

  py::class_<JointLimit>(m, "JointLimit")
      .def(py::init<>())
      .def_readwrite("lower", &JointLimit::lower)
      .def_readwrite("upper", &JointLimit::upper)
      .def_readwrite("enabled", &JointLimit::enabled);

  py::class_<JointSpec>(m, "JointSpec")
      .def(py::init<>())
      .def_readwrite("name", &JointSpec::name)
      .def_readwrite("type", &JointSpec::type)
      .def_readwrite("axis", &JointSpec::axis)
      .def_readwrite("point", &JointSpec::point)
      .def_readwrite("limit", &JointSpec::limit);

  py::class_<ManipulatorModel>(m, "ManipulatorModel")
      .def(py::init<>())
      .def("dof", &ManipulatorModel::dof)
      .def("joint_names", &ManipulatorModel::joint_names)
      .def("joint", &ManipulatorModel::joint, py::arg("i"),
           py::return_value_policy::reference_internal)
      .def("within_limits", &ManipulatorModel::within_limits, py::arg("q"), py::arg("tol") = 0.0);

  py::enum_<RmrcDamping>(m, "RmrcDamping")
      .value("None", RmrcDamping::None)
      .value("Constant", RmrcDamping::Constant)
      .value("Adaptive", RmrcDamping::Adaptive);

  using NullJlimOpt = RmrcOptions::RmrcNullspaceJointLimitOptions;
  using NullPostOpt = RmrcOptions::RmrcNullspacePostureOptions;
  using NullOpt = RmrcOptions::RmrcNullspaceOptions;

  py::class_<NullJlimOpt>(m, "RmrcNullspaceJointLimitOptions")
      .def(py::init<>())
      .def_readwrite("enabled", &NullJlimOpt::enabled)
      .def_readwrite("weight", &NullJlimOpt::weight)
      .def_readwrite("margin_frac", &NullJlimOpt::margin_frac);

  py::class_<NullPostOpt>(m, "RmrcNullspacePostureOptions")
      .def(py::init<>())
      .def_readwrite("enabled", &NullPostOpt::enabled)
      .def_readwrite("weight", &NullPostOpt::weight)
      .def_property(
          "q_nominal",
          [](const NullPostOpt& self) -> py::object {
            if (self.q_nominal.has_value()) return py::cast(*self.q_nominal);
            return py::none();
          },
          [](NullPostOpt& self, const py::object& v) {
            if (v.is_none()) {
              self.q_nominal.reset();
              return;
            }
            self.q_nominal = v.cast<Eigen::VectorXd>();
          });

  py::class_<NullOpt>(m, "RmrcNullspaceOptions")
      .def(py::init<>())
      .def_readwrite("enabled", &NullOpt::enabled)
      .def_readwrite("gain", &NullOpt::gain)
      .def_readwrite("max_joint_step_frac", &NullOpt::max_joint_step_frac)
      .def_readwrite("max_joint_step_abs", &NullOpt::max_joint_step_abs)
      .def_readwrite("max_norm_ratio", &NullOpt::max_norm_ratio)
      .def_readwrite("max_norm_abs", &NullOpt::max_norm_abs)
      .def_readwrite("joint_limits", &NullOpt::joint_limits)
      .def_readwrite("posture", &NullOpt::posture);

  py::class_<RmrcOptions>(m, "RmrcOptions")
      .def(py::init<>())
      .def_readwrite("damping", &RmrcOptions::damping)
      .def_readwrite("lambda_", &RmrcOptions::lambda)
      .def_readwrite("sigma_min", &RmrcOptions::sigma_min)
      .def_readwrite("nullspace", &RmrcOptions::nullspace);

  py::class_<JointPath>(m, "JointPath")
      .def(py::init<>())
      .def_readwrite("joint_names", &JointPath::joint_names)
      .def_readwrite("positions", &JointPath::positions)
      .def("size", &JointPath::size);

  py::class_<FailureLogOptions>(m, "FailureLogOptions")
      .def(py::init<>())
      .def_readwrite("enabled", &FailureLogOptions::enabled)
      .def_readwrite("dir", &FailureLogOptions::dir);

  py::class_<MotionPlanOptions>(m, "MotionPlanOptions")
      .def(py::init<>())
      .def_readwrite("max_iters", &MotionPlanOptions::max_iters)
      .def_readwrite("q_init_tol", &MotionPlanOptions::q_init_tol)
      .def_readwrite("pos_tol", &MotionPlanOptions::pos_tol)
      .def_readwrite("rot_tol", &MotionPlanOptions::rot_tol)
      .def_readwrite("beta", &MotionPlanOptions::beta)
      .def_readwrite("tau", &MotionPlanOptions::tau)
      .def_readwrite("tau_i", &MotionPlanOptions::tau_i)
      .def_readwrite("tau_max", &MotionPlanOptions::tau_max)
      .def_readwrite("tau_break", &MotionPlanOptions::tau_break)
      .def_readwrite("joint_delta_min", &MotionPlanOptions::joint_delta_min)
      .def_readwrite("rmrc", &MotionPlanOptions::rmrc)
      .def_readwrite("failure_log", &MotionPlanOptions::failure_log)
      .def_readwrite("thr", &MotionPlanOptions::thr);

  py::class_<MotionPlanResult>(m, "MotionPlanResult")
      .def(py::init<>())
      .def_readwrite("status", &MotionPlanResult::status)
      .def_readwrite("path", &MotionPlanResult::path)
      .def_readwrite("iters", &MotionPlanResult::iters);

  py::class_<DualQuat>(m, "DualQuat")
      .def(py::init<>())
      .def(py::init<const Mat4&>(), py::arg("T"))
      .def_static("identity", &DualQuat::identity)
      .def("to_matrix4", &DualQuat::toMatrix4)
      .def("translation", &DualQuat::translation);

  py::class_<KinematicsSolver>(m, "KinematicsSolver")
      .def(py::init<ManipulatorModel>(), py::arg("model"))
      .def("model", &KinematicsSolver::model,
           py::return_value_policy::reference_internal)
      .def("forward_kinematics",
           [](const KinematicsSolver& self, const Eigen::VectorXd& q) {
             Transform g = Transform::Identity();
             const Status st = self.forwardKinematics(q, &g);
             return py::make_tuple(st, sclerp::core::matrix4FromTransform(g));
           },
           py::arg("q"))
      .def("forward_kinematics_all",
           [](const KinematicsSolver& self, const Eigen::VectorXd& q) {
             std::vector<Transform> g_all;
             const Status st = self.forwardKinematicsAll(q, &g_all);
             std::vector<Mat4> out;
             out.reserve(g_all.size());
             for (const auto& g : g_all) out.push_back(sclerp::core::matrix4FromTransform(g));
             return py::make_tuple(st, out);
           },
           py::arg("q"))
      .def("spatial_jacobian",
           [](const KinematicsSolver& self, const Eigen::VectorXd& q) {
             Eigen::MatrixXd J;
             const Status st = self.spatialJacobian(q, &J);
             return py::make_tuple(st, J);
           },
           py::arg("q"))
      .def("rmrc_increment",
           [](const KinematicsSolver& self,
              const Mat4& dq_i,
              const Mat4& dq_f,
              const Eigen::VectorXd& q_current,
              const RmrcOptions& opt) {
             Eigen::VectorXd dq;
             KinematicsSolver::RmrcWorkspace ws;
             const Status st = self.rmrcIncrement(DualQuat(dq_i), DualQuat(dq_f), q_current, &dq, opt, &ws);
             return py::make_tuple(st, dq);
           },
           py::arg("dq_i"), py::arg("dq_f"), py::arg("q_current"), py::arg("opt") = RmrcOptions());

  m.def(
      "plan_motion_sclerp",
      [](const KinematicsSolver& solver,
         const Eigen::VectorXd& q_init,
         const Mat4& g_f,
         const MotionPlanOptions& opt) {
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
        return sclerp::core::planMotionSclerp(solver, req, opt);
      },
      py::arg("solver"),
      py::arg("q_init"),
      py::arg("g_f"),
      py::arg("opt") = MotionPlanOptions());
}

}  // namespace sclerp::python
