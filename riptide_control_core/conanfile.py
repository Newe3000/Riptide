"""Conan recipe for riptide_control_core -- the de-ROS'd control-law math
(operational-space mapping + impedance/LQR/MPC/template laws). No rclcpp,
pluginlib, rosidl, or riptide_msgs; the pluginlib registration + ROS wrappers
stay in the riptide_control (L3) package."""
import subprocess

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


class RiptideControlCoreConan(ConanFile):
    name = "riptide_control_core"
    license = "Proprietary"
    description = "Framework-neutral task-space control laws (impedance, LQR, MPC, template)."
    package_type = "shared-library"
    settings = "os", "arch", "compiler", "build_type"
    exports_sources = "CMakeLists.txt", "src/*", "include/*"

    def set_version(self):
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
        self.requires("riptide_dynamics/0.0.1", transitive_headers=True)
        self.requires("riptide_geometry/0.0.1", transitive_headers=True)
        self.requires("eigen/3.4.0", transitive_headers=True)

    def configure(self):
        self.options["pinocchio/*"].with_collision_support = False

    def layout(self):
        cmake_layout(self)

    def package_id(self):
        # ABI-sensitive core: key the binary on the full versions of its deps
        # (riptide_dynamics, riptide_geometry, Eigen -- and Boost transitively through
        # dynamics->Pinocchio) plus the compiler/ABI settings. See conan/README.md.
        self.info.requires.full_version_mode()

    def generate(self):
        CMakeDeps(self).generate()
        CMakeToolchain(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        CMake(self).install()

    def package_info(self):
        self.cpp_info.libs = ["riptide_control_core"]
        self.cpp_info.set_property("cmake_file_name", "riptide_control_core")
        self.cpp_info.set_property("cmake_target_name", "riptide_control_core::riptide_control_core")
        # NOTE: no explicit cpp_info.requires -- see riptide_dynamics/conanfile.py.
        # Pinning "pkg::pkg" component refs here zeroes out the transitive
        # riptide_dynamics LIBS in CMakeDeps, breaking the consumer link.
