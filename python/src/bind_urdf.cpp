#include "bind_common.hpp"

#include "sclerp/core/math/types.hpp"
#include "sclerp/urdf/load_manipulator.hpp"

namespace sclerp::python {
namespace {

using sclerp::core::Mat4;
using sclerp::urdf::LoadOptions;
using sclerp::urdf::LoadResult;

}  // namespace

void bind_urdf(py::module_& m) {
  py::class_<LoadOptions>(m, "LoadOptions")
      .def(py::init<>())
      .def_readwrite("base_link", &LoadOptions::base_link)
      .def_readwrite("tip_link", &LoadOptions::tip_link)
      .def_property(
          "tool_offset",
          [](const LoadOptions& self) { return sclerp::core::matrix4FromTransform(self.tool_offset); },
          [](LoadOptions& self, const Mat4& T) { self.tool_offset = sclerp::core::transformFromMatrix4(T); })
      .def_readwrite("collapse_fixed_joints", &LoadOptions::collapse_fixed_joints)
      .def_readwrite("strict", &LoadOptions::strict);

  py::class_<LoadResult>(m, "LoadResult")
      .def(py::init<>())
      .def_readwrite("status", &LoadResult::status)
      .def_readwrite("model", &LoadResult::model)
      .def_readwrite("message", &LoadResult::message)
      .def_readwrite("fk_frame_names", &LoadResult::fk_frame_names)
      .def_readwrite("fk_frame_mesh_uris", &LoadResult::fk_frame_mesh_uris);

  m.def("load_manipulator_model_from_file",
        &sclerp::urdf::loadManipulatorModelFromFile,
        py::arg("urdf_path"),
        py::arg("opt"));

  m.def("load_manipulator_model_from_string",
        &sclerp::urdf::loadManipulatorModelFromString,
        py::arg("urdf_xml"),
        py::arg("opt"));
}

}  // namespace sclerp::python

