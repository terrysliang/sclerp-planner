cmake_minimum_required(VERSION 3.20)

project(sclerp_planner VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SCLERP_BUILD_CORE "Build sclerp_core library" ON)
option(SCLERP_BUILD_URDF "Build URDF loader module" ON)
option(SCLERP_BUILD_TESTS "Build tests" OFF)
option(SCLERP_BUILD_CORE_TESTS "Build core tests" ${SCLERP_BUILD_TESTS})
option(SCLERP_BUILD_URDF_TESTS "Build URDF tests" ${SCLERP_BUILD_TESTS})

if(SCLERP_BUILD_URDF AND NOT SCLERP_BUILD_CORE)
  message(FATAL_ERROR "SCLERP_BUILD_URDF requires SCLERP_BUILD_CORE=ON")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(SclerpCommon)

# Align CTest's BUILD_TESTING default with project/module toggles unless user set it.
if(NOT DEFINED BUILD_TESTING)
  set(_sclerp_any_tests ${SCLERP_BUILD_TESTS})
  if(SCLERP_BUILD_CORE_TESTS OR SCLERP_BUILD_URDF_TESTS)
    set(_sclerp_any_tests ON)
  endif()
  set(BUILD_TESTING ${_sclerp_any_tests} CACHE BOOL "Build tests")
endif()
include(CTest)

if(SCLERP_BUILD_CORE)
  add_subdirectory(core)
endif()

if(SCLERP_BUILD_URDF)
  add_subdirectory(urdf)
endif()
