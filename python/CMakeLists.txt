find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(sclerp MODULE
  src/module.cpp
  src/bind_core.cpp
  src/bind_urdf.cpp
  src/bind_trajectory.cpp
  src/bind_collision.cpp
  src/bind_gazebo.cpp
)

target_link_libraries(sclerp PRIVATE
  sclerp_core
  sclerp_urdf
  sclerp_trajectory
  sclerp_collision
  sclerp_gazebo
)

target_compile_features(sclerp PRIVATE cxx_std_17)
sclerp_target_warnings(sclerp)

# Put the extension module in a predictable location for local use:
#   PYTHONPATH=build/python python3 -c "import sclerp"
set_target_properties(sclerp PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
)
