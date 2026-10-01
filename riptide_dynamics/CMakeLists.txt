cmake_minimum_required(VERSION 3.15)
project(riptide_dynamics CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default to an optimized build (Eigen/Pinocchio are ~10-50x slower without it,
# and -DNDEBUG disables Eigen's runtime asserts). Override with -DCMAKE_BUILD_TYPE.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Dual-mode build. RIPTIDE_CONAN_BUILD is set by the Conan CMakeToolchain
# (conanfile.py); otherwise this is the colcon/ament build. The dependency
# target names (Eigen3::Eigen, pinocchio::pinocchio) are identical on both paths,
# so the library target definition below is shared.
find_package(Eigen3 REQUIRED)

if(RIPTIDE_CONAN_BUILD)
  # Conan path: pinocchio comes from CMakeDeps; it sets the rpath, and there is
  # no ament. (Build with `conan create` / `conan build`.)
  find_package(pinocchio REQUIRED)
else()
  # colcon/ament path: pinocchio is the from-source build under PINOCCHIO_ROOT.
  find_package(ament_cmake REQUIRED)
  find_package(eigen3_cmake_module REQUIRED)
  if(NOT DEFINED PINOCCHIO_ROOT)
    if(DEFINED ENV{PINOCCHIO_ROOT})
      set(PINOCCHIO_ROOT $ENV{PINOCCHIO_ROOT})
    else()
      set(PINOCCHIO_ROOT "$ENV{HOME}/.local/pinocchio")
    endif()
  endif()
  list(APPEND CMAKE_PREFIX_PATH "${PINOCCHIO_ROOT}")
  find_package(pinocchio REQUIRED)
endif()

add_library(riptide_dynamics SHARED src/pinocchio_model.cpp)
target_include_directories(riptide_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(riptide_dynamics Eigen3::Eigen pinocchio::pinocchio)

install(DIRECTORY include/ DESTINATION include)

if(RIPTIDE_CONAN_BUILD)
  install(TARGETS riptide_dynamics
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
else()
  set_target_properties(riptide_dynamics PROPERTIES
    INSTALL_RPATH "${PINOCCHIO_ROOT}/lib")
  install(TARGETS riptide_dynamics
    EXPORT export_riptide_dynamics
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  ament_export_targets(export_riptide_dynamics HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_export_dependencies(Eigen3 pinocchio)
  ament_package()
endif()
