# ADR 0002 — The colcon↔Conan discovery seam

- **Status:** Accepted (migration Step 08 — `docs/plan/08_seam-cutover-atomic.txt`)
- **Scope:** how the two L3 colcon/ament packages (`riptide_control`,
  `riptide_mujoco`) discover their Conan-provided dependencies (the first-party
  L1 cores and the L0 third-party libs) **without** breaking ament's own
  `find_package(rclcpp)` / `rosidl` resolution.

## Context

L3 packages are built by colcon/ament and must keep resolving `rclcpp`,
`controller_interface`, `rosidl_*`, `riptide_msgs`, … from the ament prefix
path (`/opt/ros/jazzy` + the workspace `install/`). They must *additionally*
resolve `pinocchio`, `Eigen3`, `mujoco`, and the first-party cores
(`riptide_control_core`, `riptide_dynamics`, `riptide_geometry`) from Conan.

Three mechanisms were considered:

| # | Mechanism | Verdict |
|---|---|---|
| (a) | Conan **dependency-provider** (`conan_provider.cmake` injected via `CMAKE_PROJECT_TOP_LEVEL_INCLUDES`) | Works, but wires a provider into every colcon package's cmake invocation and re-runs `conan install` at configure time — heavier and more opaque for a dev workspace. |
| (b) | **Append the CMakeDeps output dir to `CMAKE_PREFIX_PATH`** | **CHOSEN.** |
| (c) | ament **vendor-wrapper** packages (one ament pkg per Conan lib) | Rejected: a new package per dependency, duplicating what CMakeDeps already emits. |

**Rejected outright:** applying the global `conan_toolchain.cmake` to the colcon
build. It sets `CMAKE_PREFIX_PATH`/`CMAKE_FIND_ROOT_PATH` wholesale and
**clobbers the ament prefix path**, breaking `find_package(rclcpp)` / `rosidl`
project-wide. The toolchain is never used on the L3 colcon side.

## Decision — mechanism (b), append-only

1. Generate the Conan dependency graph as **CMakeDeps config files only** (no
   toolchain) into a fixed workspace folder:

   ```sh
   conan install --requires=riptide_control_core/0.0.1 --requires=mujoco/3.10.0 \
     -g CMakeDeps -o "pinocchio/*:with_collision_support=False" \
     -pr:h conan/profiles/riptide-linux-release \
     -pr:b conan/profiles/riptide-linux-build \
     --build=missing --output-folder=conan/ros_deps
   ```

   This emits `pinocchio-config.cmake`, `Eigen3Config.cmake`, `mujoco-config.cmake`,
   and `riptide_{control_core,dynamics,geometry}-config.cmake` (plus their
   transitive Boost/urdfdom/tinyxml2) into `conan/ros_deps/`.

2. Build the L3 packages with that folder **appended** to the prefix path, so
   ament's entries still win for ROS packages and Conan's are found for the rest:

   ```sh
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash                       # workspace overlay (riptide_msgs)
   colcon build --packages-select riptide_mujoco riptide_control \
     --cmake-args \
       -DCMAKE_PREFIX_PATH="$PWD/conan/ros_deps" \
       -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON
   ```

   `-DCMAKE_PREFIX_PATH` here is **additive** to ament's `AMENT_PREFIX_PATH`
   (ament_cmake prepends its own paths), not a replacement — `find_package(rclcpp)`
   still resolves. `tools/build.sh` wires this up; `conan/ros_deps/` is the
   committed-ignored generators dir.

## Decision — install RPATH policy

Runtime resolution of the Conan `.so` files (which live in the Conan cache, not
on any default loader path) is handled by
`-DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON`: CMake captures every linked external
library directory into the **installed** target's `RUNPATH`. The old per-target
hardcoded `INSTALL_RPATH` pointing at `~/.mujoco` / `~/.local/pinocchio` is
dropped. `ldd` on the installed `.so` must show `libpinocchio*`/`libmujoco*`
resolving from `~/.conan2/...` — no `~/.mujoco`, no `~/.local`.

## Consequences

- The two L3 `CMakeLists.txt` lose their `MUJOCO_ROOT` / `PINOCCHIO_ROOT` blocks
  and their bespoke `INSTALL_RPATH`; they `find_package(mujoco)` /
  `find_package(riptide_control_core)` like any other dependency.
- `riptide_dynamics` becomes **Conan-only** (`COLCON_IGNORE`): colcon no longer
  builds it, and `riptide_control` consumes it transitively through
  `riptide_control_core`. This guarantees a **single** `libpinocchio` (Conan
  3.8.0) in the live `controller_manager` process — the two-libpinocchio / Boost
  ODR hazard the plan warns about cannot arise.
- Re-running `conan install` after a core changes (`conan create`) refreshes
  `conan/ros_deps/` in place; no change to the colcon invocation.
