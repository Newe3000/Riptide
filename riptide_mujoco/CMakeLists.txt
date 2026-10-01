cmake_minimum_required(VERSION 3.8)
project(riptide_mujoco)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default to an optimized build (Eigen/MuJoCo stepping). Override with -DCMAKE_BUILD_TYPE.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(riptide_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ---- MuJoCo (Conan) ---------------------------------------------------------
# Resolved from the Conan-generated mujoco-config.cmake, discovered through the
# CMakeDeps output dir appended to CMAKE_PREFIX_PATH by the colcon build (see
# docs/adr/0002-colcon-conan-seam.md). No MUJOCO_ROOT / ~/.mujoco staging.
find_package(mujoco REQUIRED)

# ---- SystemInterface plugin -------------------------------------------------
add_library(riptide_mujoco SHARED src/mujoco_system.cpp)
target_include_directories(riptide_mujoco PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(riptide_mujoco mujoco::mujoco)
ament_target_dependencies(riptide_mujoco
  hardware_interface pluginlib rclcpp rclcpp_lifecycle
  riptide_msgs geometry_msgs nav_msgs tf2_ros)

# libmujoco resolves at runtime from the Conan cache: the install RPATH is
# captured from mujoco::mujoco's imported location via
# CMAKE_INSTALL_RPATH_USE_LINK_PATH=ON (set by the colcon --cmake-args).

pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

install(TARGETS riptide_mujoco
  EXPORT export_riptide_mujoco
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(riptide_mujoco)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
