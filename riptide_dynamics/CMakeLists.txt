cmake_minimum_required(VERSION 3.8)
project(riptide_dynamics)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Header-only interface package (Phase 1). Concrete backends land in Phase 2.
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
