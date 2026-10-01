cmake_minimum_required(VERSION 3.15)
project(test_mujoco CXX)
find_package(mujoco REQUIRED)                 # resolves the Conan-generated config
add_executable(test_mujoco test_package.cpp)
target_link_libraries(test_mujoco mujoco::mujoco)
