# Throwaway consumer that proves eigen resolves via Conan under the Riptide ABI
# profile AND that CMakeDeps emits the same Eigen3::Eigen target the repo links.
from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout


class EigenSmoke(ConanFile):
    settings = "os", "arch", "compiler", "build_type"
    generators = "CMakeToolchain", "CMakeDeps", "VirtualRunEnv"

    def requirements(self):
        self.requires("eigen/3.4.0")  # pinned to system libeigen3-dev 3.4.0

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
