include(CMakePackageConfigHelpers)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(sclerp_core
  src/types.cpp
  src/so3.cpp
  src/se3.cpp
  src/adjoint.cpp
  src/distance.cpp
  src/svd.cpp
  src/dual_quat.cpp
  src/screw.cpp
  src/manipulator_model.cpp
  src/kinematics_solver.cpp
  src/motion_plan.cpp
)
add_library(sclerp::core ALIAS sclerp_core)

target_include_directories(sclerp_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(sclerp_core PUBLIC Eigen3::Eigen)
target_compile_features(sclerp_core PUBLIC cxx_std_17)
target_compile_definitions(sclerp_core PRIVATE SCLERP_CORE_BUILDING)

sclerp_target_warnings(sclerp_core)

# ---- install ----
install(TARGETS sclerp_core
  EXPORT sclerp_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT sclerp_coreTargets
  FILE sclerp_coreTargets.cmake
  NAMESPACE sclerp::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sclerp_core
)

# Config + Version files
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sclerp_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sclerp_core
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sclerp_core
)

# ---- tests ----
include(CTest)

if(BUILD_TESTING)
  enable_testing()

  add_executable(sclerp_core_smoke_test
    tests/smoke_test.cpp
  )
  target_link_libraries(sclerp_core_smoke_test PRIVATE sclerp_core)
  target_compile_features(sclerp_core_smoke_test PRIVATE cxx_std_17)

  add_test(NAME sclerp_core_smoke_test COMMAND sclerp_core_smoke_test)
endif()
