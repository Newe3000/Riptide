"""Conan recipe for MuJoCo: repackages the upstream prebuilt Linux SDK (no
compiler build) and exports the mujoco::mujoco CMake target. New versions are
added purely by extending conandata.yml (url + sha256)."""
import os

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.files import copy, get


class MujocoConan(ConanFile):
    name = "mujoco"
    description = "MuJoCo physics engine (upstream prebuilt SDK, repackaged)"
    license = "Apache-2.0"
    homepage = "https://github.com/google-deepmind/mujoco"
    url = "https://github.com/google-deepmind/mujoco"
    topics = ("physics", "simulation", "robotics", "prebuilt")
    settings = "os", "arch", "compiler", "build_type"
    package_type = "shared-library"
    no_copy_source = True

    def validate(self):
        if self.settings.os != "Linux" or self.settings.arch != "x86_64":
            raise ConanInvalidConfiguration(
                "This recipe repackages the linux-x86_64 prebuilt SDK only.")

    def source(self):
        get(self, **self.conan_data["sources"][self.version], strip_root=True)

    def package(self):
        copy(self, "*",
             src=os.path.join(self.source_folder, "include"),
             dst=os.path.join(self.package_folder, "include"))
        # Conan 2 copy() preserves the libmujoco.so -> libmujoco.so.3.10.0 symlink;
        # flattening it would break soname resolution at load time (verified below).
        copy(self, "libmujoco.so*",
             src=os.path.join(self.source_folder, "lib"),
             dst=os.path.join(self.package_folder, "lib"))
        copy(self, "LICENSE",
             src=self.source_folder,
             dst=os.path.join(self.package_folder, "licenses"))

    def package_id(self):
        # Prebuilt C-API blob: identical for any consumer toolchain, so the
        # binary is keyed on os + arch + version only.
        del self.info.settings.compiler
        del self.info.settings.build_type

    def package_info(self):
        self.cpp_info.libs = ["mujoco"]
        self.cpp_info.set_property("cmake_file_name", "mujoco")
        self.cpp_info.set_property("cmake_target_name", "mujoco::mujoco")
