cmake_minimum_required(VERSION 3.15)
project(pinocchio_smoke CXX)
find_package(pinocchio REQUIRED)              # Conan-generated config
add_executable(pinocchio_smoke main.cpp)
target_link_libraries(pinocchio_smoke pinocchio::pinocchio)
