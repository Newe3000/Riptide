"""Conan recipe for riptide_geometry -- an Eigen-only, header-only leaf: thruster
allocation (matrix + regularized pseudo-inverse) and SO(3)/SE(3) task-error math.
The lightest, most reusable package in the graph."""
import subprocess

from conan import ConanFile
from conan.tools.files import copy
import os


class RiptideGeometryConan(ConanFile):
    name = "riptide_geometry"
    license = "Proprietary"
    description = "Eigen-only geometry: thruster allocation + SO(3)/SE(3) task error."
    package_type = "header-library"
    settings = "os", "arch", "compiler", "build_type"
    exports_sources = "include/*"

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
        self.requires("eigen/3.4.0", transitive_headers=True)

    def package(self):
        copy(self, "*.hpp",
             src=os.path.join(self.source_folder, "include"),
             dst=os.path.join(self.package_folder, "include"))

    def package_id(self):
        self.info.clear()  # header-only: one package id for all configs

    def package_info(self):
        self.cpp_info.bindirs = []
        self.cpp_info.libdirs = []
        self.cpp_info.set_property("cmake_file_name", "riptide_geometry")
        self.cpp_info.set_property("cmake_target_name", "riptide_geometry::riptide_geometry")
        self.cpp_info.requires = ["eigen::eigen"]
