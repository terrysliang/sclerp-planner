#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace sclerp::python {

void bind_core(py::module_& m);
void bind_urdf(py::module_& m);
void bind_trajectory(py::module_& m);
void bind_collision(py::module_& m);
void bind_gazebo(py::module_& m);

}  // namespace sclerp::python

PYBIND11_MODULE(sclerp, m) {
  m.doc() = "sclerp_planner Python bindings (pybind11)";

  auto m_core = m.def_submodule("core");
  auto m_urdf = m.def_submodule("urdf");
  auto m_traj = m.def_submodule("trajectory");
  auto m_col = m.def_submodule("collision");
  auto m_gz = m.def_submodule("gazebo");

  // Bind order matters: other modules depend on core types.
  sclerp::python::bind_core(m_core);
  sclerp::python::bind_urdf(m_urdf);
  sclerp::python::bind_trajectory(m_traj);
  sclerp::python::bind_collision(m_col);
  sclerp::python::bind_gazebo(m_gz);
}

