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

# ---- MuJoCo -----------------------------------------------------------------
# Point at a MuJoCo SDK (headers + libmujoco). Override with -DMUJOCO_ROOT=... or
# the MUJOCO_ROOT env var; defaults to the staged SDK under ~/.mujoco.
if(NOT DEFINED MUJOCO_ROOT)
  if(DEFINED ENV{MUJOCO_ROOT})
    set(MUJOCO_ROOT $ENV{MUJOCO_ROOT})
  else()
    set(MUJOCO_ROOT "$ENV{HOME}/.mujoco/mujoco-3.10.0")
  endif()
endif()

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS "${MUJOCO_ROOT}/include")
find_library(MUJOCO_LIB mujoco HINTS "${MUJOCO_ROOT}/lib")
if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIB)
  message(FATAL_ERROR
    "MuJoCo not found under MUJOCO_ROOT='${MUJOCO_ROOT}'. "
    "Install the SDK there or pass -DMUJOCO_ROOT=/path/to/mujoco.")
endif()
get_filename_component(MUJOCO_LIB_DIR "${MUJOCO_LIB}" DIRECTORY)
message(STATUS "MuJoCo: ${MUJOCO_LIB} (include: ${MUJOCO_INCLUDE_DIR})")

# ---- SystemInterface plugin -------------------------------------------------
add_library(riptide_mujoco SHARED src/mujoco_system.cpp)
target_include_directories(riptide_mujoco PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(riptide_mujoco "${MUJOCO_LIB}")
ament_target_dependencies(riptide_mujoco
  hardware_interface pluginlib rclcpp rclcpp_lifecycle
  riptide_msgs geometry_msgs nav_msgs tf2_ros)

# Embed the MuJoCo lib dir so libmujoco resolves at runtime.
set_target_properties(riptide_mujoco PROPERTIES
  INSTALL_RPATH "${MUJOCO_LIB_DIR}"
  BUILD_WITH_INSTALL_RPATH FALSE)

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
