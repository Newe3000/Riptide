"""Standalone (zero-ROS) consumer of the Riptide first-party Conan cores.

This is the migration payoff: a plain-CMake project OUTSIDE colcon that builds the
control stack with NO ROS, NO ~/.mujoco, and NO ~/.local/pinocchio -- everything comes
from Conan. See examples/standalone_control_loop/README.md and docs/CONSUMING.md.

    conan install . -pr:h ../../conan/profiles/riptide-linux-release \
                    -pr:b ../../conan/profiles/riptide-linux-build \
                    -o "pinocchio/*:with_collision_support=False" --build=missing
    cmake --preset conan-release && cmake --build --preset conan-release
    ctest --preset conan-release
"""
from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


class StandaloneControlLoop(ConanFile):
    settings = "os", "arch", "compiler", "build_type"
    generators = "CMakeToolchain", "CMakeDeps"

    def requirements(self):
        # Pin exact versions, matching the first-party policy (see conan/README.md).
        self.requires("riptide_control_core/0.0.1")
        self.requires("riptide_dynamics/0.0.1")
        self.requires("riptide_geometry/0.0.1")

    def configure(self):
        # The cores are built with collision support off (no coal); a consumer must
        # resolve the same Pinocchio variant.
        self.options["pinocchio/*"].with_collision_support = False

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
