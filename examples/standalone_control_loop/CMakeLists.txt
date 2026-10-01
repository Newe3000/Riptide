cmake_minimum_required(VERSION 3.15)
project(standalone_control_loop CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# All dependencies come from Conan (CMakeDeps). No ROS, no find_package(ament_*),
# no *_ROOT, no hand-set rpath.
find_package(riptide_geometry REQUIRED)       # eigen-only leaf
find_package(riptide_dynamics REQUIRED)       # Pinocchio-backed dynamics
find_package(riptide_control_core REQUIRED)   # ROS-free control laws (IControlLaw)

# (1) Frictionless proof: eigen-only geometry, zero Pinocchio.
add_executable(geometry_demo geometry_demo.cpp)
target_link_libraries(geometry_demo riptide_geometry::riptide_geometry)

# (2) Full closed loop: Pinocchio dynamics + a task-space control law.
add_executable(control_loop control_loop.cpp)
target_link_libraries(control_loop
  riptide_control_core::riptide_control_core
  riptide_dynamics::riptide_dynamics
  riptide_geometry::riptide_geometry)

enable_testing()
add_test(NAME geometry_demo COMMAND geometry_demo)
# The URDF sits next to this CMakeLists; pass its absolute path so the test is
# runnable from any build dir.
add_test(NAME control_loop COMMAND control_loop "${CMAKE_CURRENT_SOURCE_DIR}/fer_arm.urdf")
