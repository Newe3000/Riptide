cmake_minimum_required(VERSION 3.8)
project(riptide_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default to an optimized build (Eigen/Pinocchio are ~10-50x slower without it,
# and -DNDEBUG disables Eigen's runtime asserts). Override with -DCMAKE_BUILD_TYPE.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---- ROS (ament) dependencies ----------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(riptide_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---- urdfdom: single (system/ROS) provider ----------------------------------
# urdf::urdf (pulled in transitively above) needs urdfdom's component targets,
# while conan Pinocchio's config references the aggregate `urdfdom::urdfdom`.
# build.sh removes Conan's urdfdom-config so both resolve to the ONE system
# urdfdom (ABI-compatible soname .so.4.0) -> no two-urdfdom ODR. Provide the
# aggregate name here for Pinocchio. See docs/adr/0002-colcon-conan-seam.md.
find_package(urdfdom REQUIRED)
if(NOT TARGET urdfdom::urdfdom)
  add_library(urdfdom::urdfdom INTERFACE IMPORTED)
  set_target_properties(urdfdom::urdfdom PROPERTIES
    INTERFACE_LINK_LIBRARIES
      "urdfdom::urdfdom_model;urdfdom::urdfdom_world;urdfdom::urdfdom_sensor")
endif()

# ---- Conan-provided cores ---------------------------------------------------
# Discovered through the CMakeDeps output dir appended to CMAKE_PREFIX_PATH by the
# colcon build (see docs/adr/0002-colcon-conan-seam.md) -- NO conan_toolchain, so
# the ament find_package()s above still resolve. riptide_control_core transitively
# carries riptide_dynamics, riptide_geometry, Pinocchio and Eigen.
find_package(riptide_control_core REQUIRED)
find_package(riptide_geometry REQUIRED)
# riptide_control instantiates the concrete riptide::PinocchioModel directly
# (ee_stabilization_controller), so depend on riptide_dynamics explicitly rather
# than relying on a transitive interface -- this makes libriptide_dynamics a hard
# NEEDED, so dlopen pulls it (and, via its RUNPATH, the single Conan Pinocchio).
find_package(riptide_dynamics REQUIRED)

add_library(riptide_control SHARED
  src/joint_pd_controller.cpp
  src/ee_stabilization_controller.cpp
  src/base_thruster_controller.cpp
  src/control_law_plugins_register.cpp)
target_include_directories(riptide_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(riptide_control
  riptide_control_core::riptide_control_core
  riptide_dynamics::riptide_dynamics
  riptide_geometry::riptide_geometry)
ament_target_dependencies(riptide_control
  ament_index_cpp controller_interface hardware_interface pluginlib
  rclcpp rclcpp_lifecycle riptide_msgs geometry_msgs)

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
# Only the ament half is exported; the Conan cores are resolved by consumers the
# same way (CMakeDeps on the prefix path), never through ament.
ament_export_dependencies(
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  riptide_msgs geometry_msgs)

ament_package()
