# ABI canary: links the Conan-built riptide_dynamics (Conan pinocchio 3.8.0 +
# boost 1.89) together with apt rclcpp in ONE process, to validate the Jazzy ABI
# profile before the Step-8 seam cutover.
from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout


class AbiCanary(ConanFile):
    settings = "os", "arch", "compiler", "build_type"
    generators = "CMakeToolchain", "CMakeDeps", "VirtualRunEnv"

    def requirements(self):
        self.requires("riptide_dynamics/0.0.1")

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
