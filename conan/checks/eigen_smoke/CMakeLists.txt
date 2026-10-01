cmake_minimum_required(VERSION 3.15)
project(eigen_smoke CXX)
find_package(Eigen3 REQUIRED)          # same package name the repo already uses
add_executable(eigen_smoke main.cpp)
target_link_libraries(eigen_smoke Eigen3::Eigen)   # same target name
