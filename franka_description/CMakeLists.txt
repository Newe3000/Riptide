cmake_minimum_required(VERSION 3.8)
project(franka_description)

find_package(ament_cmake REQUIRED)

# Install the xacro macros, configs and meshes so package:// paths resolve.
install(
  DIRECTORY robots end_effectors meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
