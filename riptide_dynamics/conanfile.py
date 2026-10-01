"""Conan recipe for riptide_dynamics -- the ROS-agnostic whole-body dynamics core
(IDynamicsModel + PinocchioModel + RobotState). The same CMakeLists also builds
under colcon/ament; the Conan path is selected by the RIPTIDE_CONAN_BUILD cache
variable set here."""
import subprocess

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


class RiptideDynamicsConan(ConanFile):
    name = "riptide_dynamics"
    license = "Proprietary"
    description = "Framework-agnostic whole-body dynamics (IDynamicsModel, PinocchioModel)."
    package_type = "shared-library"
    settings = "os", "arch", "compiler", "build_type"
    exports_sources = "CMakeLists.txt", "src/*", "include/*"

    def set_version(self):
        # Prefer the nearest git tag (vX.Y.Z); fall back to 0.0.1 (no tags yet).
        self.version = "0.0.1"
        try:
            out = subprocess.run(
                ["git", "-C", self.recipe_folder, "describe", "--tags", "--abbrev=0"],
                capture_output=True, text=True, timeout=5)
            tag = out.stdout.strip().lstrip("v")
            if tag and tag[0].isdigit():
                self.version = tag
        except Exception:
            pass

    def requirements(self):
        # transitive_headers: consumers include Eigen + pinocchio headers through us.
        self.requires("eigen/3.4.0", transitive_headers=True)
        self.requires("pinocchio/3.8.0", transitive_headers=True)

    def configure(self):
        self.options["pinocchio/*"].with_collision_support = False

    def layout(self):
        cmake_layout(self)

    def generate(self):
        CMakeDeps(self).generate()
        tc = CMakeToolchain(self)
        tc.cache_variables["RIPTIDE_CONAN_BUILD"] = True
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        CMake(self).install()

    def package_info(self):
        self.cpp_info.libs = ["riptide_dynamics"]
        self.cpp_info.set_property("cmake_file_name", "riptide_dynamics")
        self.cpp_info.set_property("cmake_target_name", "riptide_dynamics::riptide_dynamics")
        self.cpp_info.requires = ["eigen::eigen", "pinocchio::pinocchio"]
