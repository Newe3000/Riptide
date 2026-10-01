# Consuming the Riptide Conan packages

The reusable Riptide cores ship as **Conan 2** packages, so you can build them with no
ROS, no `~/.mujoco`, and no `~/.local/pinocchio`. This page is the honest consumer guide;
the runnable proof is `examples/standalone_control_loop/` (CI-guarded in a ROS-free
container).

## Start here — the eigen-only leaf (frictionless)

The lowest-barrier package is `riptide_geometry`: header-only, Eigen-only, no Pinocchio,
no Boost. If you just want the thruster allocation / SE(3) task-error math:

```python
# conanfile.py
def requirements(self):
    self.requires("riptide_geometry/0.0.1")
```
```cmake
find_package(riptide_geometry REQUIRED)
target_link_libraries(my_app riptide_geometry::riptide_geometry)
```
```cpp
#include "riptide_geometry/allocation.hpp"   // allocation_matrix(), regularized_pinv()
#include "riptide_geometry/spatial.hpp"      // task_pose_error()
```

That's the whole dependency. See `geometry_demo.cpp` in the example.

## The full stack

| Package | Role | Pulls |
|---|---|---|
| `riptide_geometry/0.0.1` | eigen-only geometry (header-only) | `eigen` |
| `riptide_dynamics/0.0.1` | whole-body dynamics (`PinocchioModel`, `IDynamicsModel`, `RobotState`) | `eigen`, `pinocchio` |
| `riptide_control_core/0.0.1` | ROS-free control laws (`IControlLaw`: impedance / LQR / MPC / template) | `riptide_dynamics`, `riptide_geometry`, `eigen` |

```python
def requirements(self):
    self.requires("riptide_control_core/0.0.1")   # brings dynamics + geometry + pinocchio + eigen
    self.requires("riptide_dynamics/0.0.1")       # list directly if you link PinocchioModel yourself*
    self.requires("riptide_geometry/0.0.1")

def configure(self):
    self.options["pinocchio/*"].with_collision_support = False
```

\* `control_core` is a shared library that does **not** itself NEED `libriptide_dynamics`
(it uses only the abstract `IDynamicsModel`). If your code instantiates the concrete
`riptide::PinocchioModel`, list `riptide_dynamics` as a **direct** require so CMakeDeps
emits its libs onto your link line — see `docs/adr/0002-colcon-conan-seam.md`.

## Remotes & profile

```sh
# conancenter (default) provides eigen/pinocchio/boost/urdfdom/...; the private remote
# provides the first-party cores + the custom mujoco recipe.
conan remote add riptide-private <your-remote-url>
conan remote login riptide-private <user>

# Build with the frozen ABI profile (gcc13 / libstdc++11 / gnu17 / Release) so the
# binaries are ABI-coherent with apt ROS 2 Jazzy (only needed if you mix them in-process).
conan install . -pr:h conan/profiles/riptide-linux-release \
                -pr:b conan/profiles/riptide-linux-build \
                -o "pinocchio/*:with_collision_support=False" --build=missing
```

Conan builds **relocatable** libraries (no baked rpath), so before running a consumer
binary put the Conan cache lib dirs on the loader path:
`source <build>/generators/conanrun.sh`.

## Active core development (editable)

To hack on a core without re-`conan create` each change:

```sh
conan editable add riptide_dynamics
conan editable add riptide_geometry
conan editable add riptide_control_core
# ... build the editable layouts, then consumers resolve from your working tree.
conan editable remove riptide_dynamics   # back to cache/remote
```

## The third-party situation (honest)

- **Pinocchio** — `pinocchio/3.8.0` straight from **conancenter** (not forked), built
  **with collision support off** (`with_collision_support=False`, so no `coal`/`hpp-fcl`).
  That's the version the cores pin; a consumer must resolve the same option.
- **Boost** — pulled by Pinocchio. Pinocchio links Boost **statically**, so no
  `libboost_*.so` crosses the Pinocchio `.so` boundary; a process mixing Conan Pinocchio
  with apt ROS (Boost 1.83) shows no two-Boost ODR symptom (verified in Step 08). The
  cores keep Boost out of their public API entirely.
- **MuJoCo** — `mujoco/3.10.0` is a **repackaged prebuilt SDK** (Linux x86_64), not a
  from-source build; keyed on `os`+`arch`+`version` only. It is only needed by the ROS
  sim's hardware plugin, not by the control cores, so a pure-Conan consumer of the cores
  never pulls it.
- **Eigen** — `eigen/3.4.0`, pinned to the exact apt `libeigen3-dev` version to avoid
  header ODR/alignment skew.

## Two consumption modes

1. **Pure Conan (external, this guide)** — `conan install` + plain CMake. No ROS.
2. **colcon + the Step-8 seam (the Riptide ROS sim)** — the colcon packages append the
   CMakeDeps output dir to `CMAKE_PREFIX_PATH` (never a global toolchain). See
   `docs/adr/0002-colcon-conan-seam.md` and `docs/ARCHITECTURE.md` §7.
