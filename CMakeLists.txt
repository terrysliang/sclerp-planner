cmake_minimum_required(VERSION 3.20)

project(sclerp_planner VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SCLERP_BUILD_CORE "Build sclerp_core library" ON)
option(SCLERP_BUILD_TESTS "Build tests" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(SclerpCommon)

if(SCLERP_BUILD_CORE)
  add_subdirectory(core)
endif()

if(SCLERP_BUILD_TESTS)
  enable_testing()
  # add_subdirectory(tests)
endif()
