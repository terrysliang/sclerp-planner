include(CMakePackageConfigHelpers)

add_library(sclerp_gazebo
  src/obstacle_registry.cpp
  src/obstacle_registry_io.cpp
  src/joint_trajectory_csv.cpp
)
add_library(sclerp::gazebo ALIAS sclerp_gazebo)

target_include_directories(sclerp_gazebo
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(sclerp_gazebo
  PUBLIC
    sclerp_collision
)

# URDF parsing (for inline robot export)
find_package(PkgConfig REQUIRED)
pkg_check_modules(URDFDOM REQUIRED IMPORTED_TARGET urdfdom)
pkg_check_modules(URDFDOM_HEADERS REQUIRED IMPORTED_TARGET urdfdom_headers)
pkg_check_modules(SDFORMAT12 REQUIRED IMPORTED_TARGET sdformat12)

target_link_libraries(sclerp_gazebo
  PRIVATE
    PkgConfig::URDFDOM
    PkgConfig::URDFDOM_HEADERS
    PkgConfig::SDFORMAT12
)

target_compile_features(sclerp_gazebo PUBLIC cxx_std_17)
sclerp_target_warnings(sclerp_gazebo)

# ---- CLI world generator ----
add_executable(sclerp_gazebo_generate_world
  src/generate_world_cli.cpp
)
target_link_libraries(sclerp_gazebo_generate_world PRIVATE sclerp_gazebo)
target_compile_features(sclerp_gazebo_generate_world PRIVATE cxx_std_17)
sclerp_target_warnings(sclerp_gazebo_generate_world)

install(TARGETS sclerp_gazebo_generate_world
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# ---- ignition gazebo sim plugin (optional) ----
option(SCLERP_GAZEBO_BUILD_IGNITION_PLUGIN
  "Build Gazebo Sim 6 system plugins (JointTrajectoryPlayer, WorldSdfSaver, PrimitiveResizer)"
  ON
)

if(SCLERP_GAZEBO_BUILD_IGNITION_PLUGIN)
  find_package(ignition-gazebo6 QUIET)
  find_package(ignition-plugin1 QUIET)

  if(ignition-gazebo6_FOUND AND ignition-plugin1_FOUND)
    add_library(sclerp_joint_trajectory_player SHARED
      src/joint_trajectory_player_ignition.cpp
    )
    target_link_libraries(sclerp_joint_trajectory_player
      PRIVATE
        ignition-gazebo6::ignition-gazebo6
        ignition-plugin1::ignition-plugin1
    )
    target_compile_features(sclerp_joint_trajectory_player PRIVATE cxx_std_17)
    sclerp_target_warnings(sclerp_joint_trajectory_player)

    set_target_properties(sclerp_joint_trajectory_player PROPERTIES
      OUTPUT_NAME "sclerp_joint_trajectory_player"
    )

    install(TARGETS sclerp_joint_trajectory_player
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    add_library(sclerp_world_sdf_saver SHARED
      src/world_sdf_saver_ignition.cpp
    )
    target_link_libraries(sclerp_world_sdf_saver
      PRIVATE
        ignition-gazebo6::ignition-gazebo6
        ignition-plugin1::ignition-plugin1
    )
    target_compile_features(sclerp_world_sdf_saver PRIVATE cxx_std_17)
    sclerp_target_warnings(sclerp_world_sdf_saver)

    set_target_properties(sclerp_world_sdf_saver PROPERTIES
      OUTPUT_NAME "sclerp_world_sdf_saver"
    )

    install(TARGETS sclerp_world_sdf_saver
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    add_library(sclerp_primitive_resizer SHARED
      src/primitive_resizer_ignition.cpp
    )
    target_link_libraries(sclerp_primitive_resizer
      PRIVATE
        ignition-gazebo6::ignition-gazebo6
        ignition-plugin1::ignition-plugin1
    )
    target_compile_features(sclerp_primitive_resizer PRIVATE cxx_std_17)
    sclerp_target_warnings(sclerp_primitive_resizer)

    set_target_properties(sclerp_primitive_resizer PROPERTIES
      OUTPUT_NAME "sclerp_primitive_resizer"
    )

    install(TARGETS sclerp_primitive_resizer
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
  else()
    message(STATUS "Ignition Gazebo Sim 6 not found; skipping sclerp Gazebo system plugins")
  endif()
endif()

# ---- install ----
install(TARGETS sclerp_gazebo
  EXPORT sclerp_gazeboTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT sclerp_gazeboTargets
  FILE sclerp_gazeboTargets.cmake
  NAMESPACE sclerp::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sclerp_gazebo
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sclerp_gazeboConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_gazeboConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sclerp_gazebo
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_gazeboConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_gazeboConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_gazeboConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sclerp_gazebo
)

# ---- tests ----
include(CTest)

if(SCLERP_BUILD_TESTS AND BUILD_TESTING)
  enable_testing()

  add_executable(sclerp_gazebo_unit_test
    tests/unit_test.cpp
  )
  target_link_libraries(sclerp_gazebo_unit_test PRIVATE sclerp_gazebo)
  target_compile_features(sclerp_gazebo_unit_test PRIVATE cxx_std_17)

  add_test(NAME sclerp_gazebo_unit_test COMMAND sclerp_gazebo_unit_test)
endif()
