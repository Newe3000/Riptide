cmake_minimum_required(VERSION 3.15)
project(test_riptide_dynamics CXX)
find_package(riptide_dynamics REQUIRED)       # Conan-generated config
add_executable(test_riptide_dynamics test_package.cpp)
target_link_libraries(test_riptide_dynamics riptide_dynamics::riptide_dynamics)
