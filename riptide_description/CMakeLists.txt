cmake_minimum_required(VERSION 3.8)
project(riptide_description)

find_package(ament_cmake REQUIRED)

# urdf/ : xacro sources (description + ros2_control seam)
# config/ : controller_manager + controller parameters
# mujoco/ : MJCF scene + meshes for the MuJoCo SystemInterface (Phase 2)
install(
  DIRECTORY urdf config mujoco rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
