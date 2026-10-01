cmake_minimum_required(VERSION 3.15)
project(abi_canary CXX)
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/jazzy")   # apt ROS for rclcpp (+ transitive)
find_package(riptide_dynamics REQUIRED)           # Conan (CMakeDeps)
find_package(rclcpp REQUIRED)                      # apt ament
add_executable(abi_canary main.cpp)
target_link_libraries(abi_canary riptide_dynamics::riptide_dynamics rclcpp::rclcpp)
