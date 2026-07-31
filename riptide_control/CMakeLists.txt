cmake_minimum_required(VERSION 3.8)
project(riptide_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Pinocchio prefix (needed transitively via riptide_dynamics).
if(NOT DEFINED PINOCCHIO_ROOT)
  if(DEFINED ENV{PINOCCHIO_ROOT})
    set(PINOCCHIO_ROOT $ENV{PINOCCHIO_ROOT})
  else()
    set(PINOCCHIO_ROOT "$ENV{HOME}/.local/pinocchio")
  endif()
endif()
list(APPEND CMAKE_PREFIX_PATH "${PINOCCHIO_ROOT}")
find_package(riptide_dynamics REQUIRED)

add_library(riptide_control SHARED
  src/joint_pd_controller.cpp
  src/ee_stabilization_controller.cpp
  src/base_thruster_controller.cpp
  src/task_space_impedance.cpp
  src/task_space_lqr.cpp)
target_include_directories(riptide_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(riptide_control Eigen3::Eigen)
ament_target_dependencies(riptide_control
  ament_index_cpp controller_interface hardware_interface pluginlib
  rclcpp rclcpp_lifecycle riptide_dynamics)
set_target_properties(riptide_control PROPERTIES
  INSTALL_RPATH "${PINOCCHIO_ROOT}/lib")

# Register the ros2_control controllers and the IControlLaw plugins.
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)
pluginlib_export_plugin_description_file(riptide_control control_law_plugins.xml)

install(TARGETS riptide_control
  EXPORT export_riptide_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES controller_plugins.xml control_law_plugins.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(riptide_control)
ament_export_dependencies(
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  Eigen3 riptide_dynamics)
ament_package()
