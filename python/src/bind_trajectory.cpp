#include "bind_common.hpp"

#include "sclerp/trajectory/interpolator.hpp"

namespace sclerp::python {
namespace {

using sclerp::trajectory::CsvMode;
using sclerp::trajectory::GridTotg;
using sclerp::trajectory::Limits;
using sclerp::trajectory::PlannedSegment;
using sclerp::trajectory::PlannedTrajectory;
using sclerp::trajectory::Sample;
using sclerp::core::JointPath;
using sclerp::core::Status;

}  // namespace

void bind_trajectory(py::module_& m) {
  py::class_<Limits>(m, "Limits")
      .def(py::init<>())
      .def_readwrite("v_max", &Limits::v_max)
      .def_readwrite("a_max", &Limits::a_max)
      .def_readwrite("j_max", &Limits::j_max);

  py::class_<Sample>(m, "Sample")
      .def(py::init<>())
      .def_readwrite("t", &Sample::t)
      .def_readwrite("q", &Sample::q)
      .def_readwrite("qd", &Sample::qd)
      .def_readwrite("qdd", &Sample::qdd);

  py::class_<PlannedSegment>(m, "PlannedSegment")
      .def(py::init<>())
      .def_readwrite("T", &PlannedSegment::T)
      .def_readwrite("q0", &PlannedSegment::q0)
      .def_readwrite("dQ", &PlannedSegment::dQ)
      .def_readwrite("c", &PlannedSegment::c);

  py::class_<PlannedTrajectory>(m, "PlannedTrajectory")
      .def(py::init<>())
      .def_readwrite("joint_names", &PlannedTrajectory::joint_names)
      .def_readwrite("segs", &PlannedTrajectory::segs)
      .def_readwrite("table", &PlannedTrajectory::table)
      .def_readwrite("total_time", &PlannedTrajectory::total_time)
      .def_readwrite("sample_dt", &PlannedTrajectory::sample_dt);

  py::enum_<CsvMode>(m, "CsvMode")
      .value("PositionOnly", CsvMode::PositionOnly)
      .value("PositionVelocityAcceleration", CsvMode::PositionVelocityAcceleration);

  py::class_<GridTotg> cls_grid(m, "GridTotg");

  py::class_<GridTotg::Config>(cls_grid, "Config")
      .def(py::init<>())
      .def_readwrite("unwrap_angles", &GridTotg::Config::unwrap_angles)
      .def_readwrite("dedup_eps", &GridTotg::Config::dedup_eps)
      .def_readwrite("drop_eps", &GridTotg::Config::drop_eps)
      .def_readwrite("resample_uniform", &GridTotg::Config::resample_uniform)
      .def_readwrite("resample_ds", &GridTotg::Config::resample_ds)
      .def_readwrite("max_resample_points", &GridTotg::Config::max_resample_points)
      .def_readwrite("grid_ds", &GridTotg::Config::grid_ds)
      .def_readwrite("max_grid_points", &GridTotg::Config::max_grid_points)
      .def_readwrite("sdot_start", &GridTotg::Config::sdot_start)
      .def_readwrite("sdot_goal", &GridTotg::Config::sdot_goal)
      .def_readwrite("compute_qd_qdd", &GridTotg::Config::compute_qd_qdd)
      .def_readwrite("verbose", &GridTotg::Config::verbose);

  cls_grid.def(py::init<Limits>(), py::arg("limits"));
  cls_grid.def(py::init<Limits, GridTotg::Config>(), py::arg("limits"), py::arg("cfg") = GridTotg::Config());
  cls_grid.def(
      "plan",
      [](const GridTotg& self,
         const JointPath& path,
         double sample_dt,
         const std::vector<bool>& unwrap_revolute_mask) {
        PlannedTrajectory out;
        const Status st = self.plan(path, sample_dt, &out, unwrap_revolute_mask);
        return py::make_tuple(st, out);
      },
      py::arg("path"),
      py::arg("sample_dt"),
      py::arg("unwrap_revolute_mask") = std::vector<bool>{});

  m.def("continuous_revolute_mask",
        &sclerp::trajectory::continuousRevoluteMask,
        py::arg("model"));

  m.def("write_trajectory_csv",
        &sclerp::trajectory::writeTrajectoryCsv,
        py::arg("traj"),
        py::arg("csv_path"),
        py::arg("mode") = CsvMode::PositionOnly);
}

}  // namespace sclerp::python
