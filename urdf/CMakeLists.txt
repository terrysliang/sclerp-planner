cmake_minimum_required(VERSION 3.16)

add_library(sclerp_urdf
  src/load_manipulator.cpp
)

target_compile_features(sclerp_urdf PUBLIC cxx_std_17)

target_include_directories(sclerp_urdf
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(sclerp_urdf
  PUBLIC
    sclerp_core
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(URDFDOM REQUIRED IMPORTED_TARGET urdfdom)
pkg_check_modules(URDFDOM_HEADERS REQUIRED IMPORTED_TARGET urdfdom_headers)

target_link_libraries(sclerp_urdf
  PRIVATE
    PkgConfig::URDFDOM
    PkgConfig::URDFDOM_HEADERS
)

# ---- install ----
install(TARGETS sclerp_urdf
  EXPORT sclerpTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

include(CTest)

if(SCLERP_BUILD_URDF_TESTS AND BUILD_TESTING)
  add_executable(sclerp_urdf_kinematics_test
    tests/urdf_kinematics_test.cpp
  )
  target_link_libraries(sclerp_urdf_kinematics_test PRIVATE sclerp_urdf sclerp_core)
  target_compile_features(sclerp_urdf_kinematics_test PRIVATE cxx_std_17)

  add_test(NAME sclerp_urdf_kinematics_test COMMAND sclerp_urdf_kinematics_test)
endif()
