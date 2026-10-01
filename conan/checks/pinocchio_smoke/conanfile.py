# Standalone check: proves pinocchio/3.8.0 resolves via Conan under the Riptide
# ABI profile and that the exact API PinocchioModel uses (buildModel, crba,
# getFrameJacobian, forwardKinematics/updateFramePlacements) compiles and runs.
from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout


class PinocchioSmoke(ConanFile):
    settings = "os", "arch", "compiler", "build_type"
    generators = "CMakeToolchain", "CMakeDeps", "VirtualRunEnv"

    def requirements(self):
        self.requires("pinocchio/3.8.0")
        self.requires("eigen/3.4.0")  # force the exact system version

    def configure(self):
        self.options["pinocchio/*"].with_collision_support = False

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
