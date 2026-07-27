cmake_minimum_required(VERSION 3.8)
project(riptide_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(riptide_dynamics REQUIRED)

# Controller library (currently: JointPdController). The IControlLaw plugins
# (PID/impedance/LQR/MPC) and the EeStabilizationController are added here later.
add_library(riptide_control SHARED
  src/joint_pd_controller.cpp)
target_include_directories(riptide_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(riptide_control Eigen3::Eigen)
ament_target_dependencies(riptide_control
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  riptide_dynamics)

# Register the controller with controller_interface via pluginlib.
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(TARGETS riptide_control
  EXPORT export_riptide_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES controller_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(riptide_control)
ament_export_dependencies(
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  Eigen3 riptide_dynamics)
ament_package()
