cmake_minimum_required(VERSION 3.15)
project(riptide_control_core CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Conan-only package (CMakeDeps provides all three targets).
find_package(Eigen3 REQUIRED)
find_package(riptide_dynamics REQUIRED)
find_package(riptide_geometry REQUIRED)

add_library(riptide_control_core SHARED
  src/task_space_impedance.cpp
  src/task_space_lqr.cpp
  src/task_space_mpc.cpp
  src/template_control_law.cpp)
target_include_directories(riptide_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(riptide_control_core
  Eigen3::Eigen
  riptide_dynamics::riptide_dynamics
  riptide_geometry::riptide_geometry)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS riptide_control_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
