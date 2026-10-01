# Standalone control loop — the zero-ROS consumer

This is the migration payoff: a plain-CMake project **outside colcon** that builds and
runs the Riptide control stack with **no ROS, no `~/.mujoco`, no `~/.local/pinocchio`** —
every dependency comes from Conan. Two executables, both wired into CTest:

| Target | What it proves | Deps |
|---|---|---|
| `geometry_demo` | The frictionless entry point: the **eigen-only** `riptide_geometry` leaf (thruster allocation + SE(3) task error). No Pinocchio. | `riptide_geometry` |
| `control_loop` | The full payoff: Conan **Pinocchio** loads the vendored `fer` URDF, builds a `RobotState`, and drives `TaskSpaceImpedance` through the ROS-free `riptide::IControlLaw` in a closed loop that converges the end-effector to a scripted Cartesian target. | `riptide_control_core` + `riptide_dynamics` + `riptide_geometry` |

## Build & run

Needs only Conan 2 + a C++17 toolchain. Point at the frozen ABI profiles and resolve
the first-party cores from your remote (or a local cache / editables):

```sh
cd examples/standalone_control_loop

conan install . \
  -pr:h ../../conan/profiles/riptide-linux-release \
  -pr:b ../../conan/profiles/riptide-linux-build \
  -o "pinocchio/*:with_collision_support=False" --build=missing

cmake --preset conan-release
cmake --build --preset conan-release

# Conan builds relocatable libs (no baked rpath), so put the Conan cache lib dirs
# (Pinocchio, the cores) on the loader path before running:
source build/Release/generators/conanrun.sh

ctest --preset conan-release --output-on-failure
# or run directly:
./build/Release/geometry_demo
./build/Release/control_loop fer_arm.urdf
```

Expected:

```
geometry_demo: A=6x7  wrench_reproduce_err=~2e-05  pose_err=0.100
geometry_demo: PASS
control_loop: EE error  start=0.0361 m  end=0.0000 m  (target offset=0.036 m)
control_loop: PASS
```

## Notes

- **No ROS.** The control laws are consumed through `riptide::IControlLaw` with an
  in-memory `ParamSource` (defaults) and a no-op logger — the same seam the ROS layer
  binds to `rclcpp` (see `docs/ARCHITECTURE.md`).
- **No physics engine.** `control_loop` integrates the rigid-body forward dynamics
  (`ddq = M⁻¹(τ − nle)`) itself, so it needs neither MuJoCo nor `ros2_control`.
- The CI job `example-clean-container` builds and runs this in a **ROS-free** container
  so it cannot rot. See `.github/workflows/ci.yml` and `docs/CONSUMING.md`.
