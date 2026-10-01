cmake_minimum_required(VERSION 3.15)
project(riptide_dynamics CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default to an optimized build (Eigen/Pinocchio are ~10-50x slower without it,
# and -DNDEBUG disables Eigen's runtime asserts). Override with -DCMAKE_BUILD_TYPE.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Conan-only Layer-1 core (COLCON_IGNORE). Eigen + Pinocchio come from CMakeDeps
# (conanfile.py); Pinocchio sets its own rpath. No ament, no PINOCCHIO_ROOT.
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(riptide_dynamics SHARED src/pinocchio_model.cpp)
target_include_directories(riptide_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(riptide_dynamics Eigen3::Eigen pinocchio::pinocchio)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS riptide_dynamics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
