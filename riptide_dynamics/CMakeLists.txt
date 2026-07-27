cmake_minimum_required(VERSION 3.8)
project(riptide_dynamics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Pinocchio (built from source; see README). Override with -DPINOCCHIO_ROOT=...
# or the PINOCCHIO_ROOT env var; defaults to ~/.local/pinocchio.
if(NOT DEFINED PINOCCHIO_ROOT)
  if(DEFINED ENV{PINOCCHIO_ROOT})
    set(PINOCCHIO_ROOT $ENV{PINOCCHIO_ROOT})
  else()
    set(PINOCCHIO_ROOT "$ENV{HOME}/.local/pinocchio")
  endif()
endif()
list(APPEND CMAKE_PREFIX_PATH "${PINOCCHIO_ROOT}")
find_package(pinocchio REQUIRED)

add_library(riptide_dynamics SHARED src/pinocchio_model.cpp)
target_include_directories(riptide_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(riptide_dynamics Eigen3::Eigen pinocchio::pinocchio)
set_target_properties(riptide_dynamics PROPERTIES
  INSTALL_RPATH "${PINOCCHIO_ROOT}/lib")

install(
  DIRECTORY include/
  DESTINATION include)
install(TARGETS riptide_dynamics
  EXPORT export_riptide_dynamics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_riptide_dynamics HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 pinocchio)
ament_package()
