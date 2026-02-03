include(CMakePackageConfigHelpers)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(fcl REQUIRED)
find_package(assimp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem) 

# Normalize FCL target name across distros
if(NOT TARGET fcl::fcl)
  if(TARGET fcl)
    add_library(fcl::fcl ALIAS fcl)
  elseif(TARGET FCL::fcl)
    add_library(fcl::fcl ALIAS FCL::fcl)
  elseif(TARGET fcl::fcl_shared)
    add_library(fcl::fcl ALIAS fcl::fcl_shared)
  endif()
endif()

add_library(sclerp_collision
  src/avoidance.cpp
  src/collision_utils.cpp
  src/motion_plan_collision.cpp
  src/lemke.cpp
)
add_library(sclerp::collision ALIAS sclerp_collision)

target_include_directories(sclerp_collision
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(sclerp_collision
  PUBLIC
    sclerp_core
    Eigen3::Eigen
    fcl::fcl
    assimp::assimp
)

target_compile_features(sclerp_collision PUBLIC cxx_std_17)
sclerp_target_warnings(sclerp_collision)

# ---- install ----
install(TARGETS sclerp_collision
  EXPORT sclerp_collisionTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT sclerp_collisionTargets
  FILE sclerp_collisionTargets.cmake
  NAMESPACE sclerp::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sclerp_collision
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sclerp_collisionConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_collisionConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sclerp_collision
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_collisionConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_collisionConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/sclerp_collisionConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sclerp_collision
)

# ---- tests ----
include(CTest)

if(SCLERP_BUILD_TESTS AND BUILD_TESTING)
  enable_testing()

  add_executable(sclerp_collision_compat_test
    tests/compat_test.cpp
  )
  target_link_libraries(sclerp_collision_compat_test PRIVATE sclerp_collision)
  target_compile_features(sclerp_collision_compat_test PRIVATE cxx_std_17)

  add_test(NAME sclerp_collision_compat_test COMMAND sclerp_collision_compat_test)
endif()

option(SCLERP_BUILD_KINLIB_COMPAT_TEST "Build kinlib compatibility test" OFF)
set(SCLERP_KINLIB_PREFIX "" CACHE PATH "Prefix where kinlib is installed")

if(SCLERP_BUILD_KINLIB_COMPAT_TEST AND SCLERP_BUILD_TESTS AND BUILD_TESTING)
  set(_kinlib_hints)
  if(SCLERP_KINLIB_PREFIX)
    list(APPEND _kinlib_hints "${SCLERP_KINLIB_PREFIX}")
  elseif(EXISTS "/home/terry/gd/install/kinlib")
    list(APPEND _kinlib_hints "/home/terry/gd/install/kinlib")
  endif()

  find_package(kinlib CONFIG QUIET HINTS ${_kinlib_hints})
  if(NOT kinlib_FOUND)
    message(FATAL_ERROR "kinlib not found. Set SCLERP_KINLIB_PREFIX to your kinlib install prefix.")
  endif()

  add_executable(sclerp_collision_kinlib_compat_test
    tests/kinlib_compat_test.cpp
  )
  target_link_libraries(sclerp_collision_kinlib_compat_test
    PRIVATE
      sclerp_collision
      kinlib::kinlib
  )
  target_compile_features(sclerp_collision_kinlib_compat_test PRIVATE cxx_std_17)

  add_test(NAME sclerp_collision_kinlib_compat_test COMMAND sclerp_collision_kinlib_compat_test)
endif()
