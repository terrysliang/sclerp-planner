include(CMakePackageConfigHelpers)

add_library(sclerp_trajectory
  src/common.cpp
  src/grid_totg.cpp
  src/csv.cpp
  src/toppra.cpp
)
add_library(sclerp::trajectory ALIAS sclerp_trajectory)

target_include_directories(sclerp_trajectory
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(sclerp_trajectory
  PUBLIC
    sclerp_core
)

target_compile_features(sclerp_trajectory PUBLIC cxx_std_17)
sclerp_target_warnings(sclerp_trajectory)

# ---- optional TOPPRA backend ----
option(SCLERP_TRAJECTORY_WITH_TOPPRA "Enable TOPPRA trajectory backend when available" ON)

if(SCLERP_TRAJECTORY_WITH_TOPPRA)
  find_package(toppra QUIET)
  if(toppra_FOUND)
    target_compile_definitions(sclerp_trajectory PUBLIC SCLERP_TRAJECTORY_HAS_TOPPRA)
    # `toppra::toppra` is needed by consumers when sclerp_trajectory is a static library.
    target_link_libraries(sclerp_trajectory PUBLIC toppra::toppra)
  else()
    message(STATUS "toppra not found; building sclerp_trajectory without TOPPRA support")
  endif()
endif()

# ---- install ----
install(TARGETS sclerp_trajectory
  EXPORT sclerp_trajectoryTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT sclerp_trajectoryTargets
  FILE sclerp_trajectoryTargets.cmake
  NAMESPACE sclerp::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sclerp_trajectory
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sclerp_trajectoryConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_trajectoryConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sclerp_trajectory
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_trajectoryConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_trajectoryConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_trajectoryConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sclerp_trajectory
)

# ---- tests ----
include(CTest)

if(SCLERP_BUILD_TESTS AND BUILD_TESTING)
  enable_testing()

  add_executable(sclerp_trajectory_unit_test
    tests/unit_test.cpp
  )
  target_link_libraries(sclerp_trajectory_unit_test PRIVATE sclerp_trajectory)
  target_compile_features(sclerp_trajectory_unit_test PRIVATE cxx_std_17)

  add_test(NAME sclerp_trajectory_unit_test COMMAND sclerp_trajectory_unit_test)
endif()
