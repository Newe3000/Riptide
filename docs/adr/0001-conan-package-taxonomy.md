# ADR 0001 — Conan package taxonomy, layer rules, and audit baseline

- **Status:** Accepted (migration Step 01 — `docs/plan/01_freeze-taxonomy-audit-golden-baseline.txt`)
- **Scope:** freeze the target package graph and the dependency invariant; record
  the verified boundary-violation list and a golden control-law baseline. **No
  product code is moved or changed in this step.**

## Context

Riptide is a colcon/ament monorepo. Its physics (`riptide_dynamics`) and
control-law math are, in substance, framework-neutral C++, but they are welded to
the ROS build and API. The goal is a project of **multiple Conan packages** whose
reusable cores are consumable outside ROS, while the ros2_control / MuJoCo layer
keeps working by consuming them. Every later migration step needs a fixed target
and a machine-checkable invariant to migrate against.

## Decision — the frozen layer model

Dependencies point **downward only**. A higher layer may depend on a lower one;
never the reverse, and L1 must not depend on ROS at all.

| Layer | What | Packaging |
|---|---|---|
| **L0** third-party | Eigen, Boost, urdfdom, Pinocchio, MuJoCo | Conan (conan-center; MuJoCo custom recipe) |
| **L1** pure cores | `riptide_dynamics`, `riptide_control_core`, `riptide_geometry` | **Conan (first-party)** |
| **L2** interfaces | `riptide_msgs` | rosidl / colcon (deliberately **not** Conan) |
| **L3** ROS glue | `riptide_control`, `riptide_mujoco` | colcon; **consume** L1 via Conan |
| **L4** apps | `riptide_bringup`, `riptide_description`, `riptide_disturbance`, `riptide_eval` | colcon |

### Invariant (machine-enforced)

> No source under an L1 target may reference `rclcpp`, `rclpy`, `pluginlib`,
> `controller_interface`, `hardware_interface`, `rosidl`, or `riptide_msgs`.

Enforced by `tools/check_layer_boundaries.sh` (wired into CI in Step 09). The
core/ROS seam is crossed **only** by explicit POD↔msg / param / logger adapters
living in L3, never by leaking a ROS type into a core.

## Decision — the frozen Conan package taxonomy

| Package | Contents (public API) | Requires | Consumers |
|---|---|---|---|
| `eigen` (CCI, pinned) | Header-only linear algebra, pinned to the system version so Conan Eigen == apt Eigen | — | all cores + pinocchio |
| `boost` (CCI, ABI-managed) | Pinocchio's transitive dep; version locked in `package_id`, contained inside `libpinocchio.so` | — | pinocchio only |
| `mujoco` (**custom recipe**) | Repackaged upstream prebuilt SDK; `mujoco::mujoco` target | — | `riptide_mujoco` |
| `pinocchio/3.8.0` (CCI, pin+delta) | Rigid-body dynamics, `with_collision_support=False`, Python off | eigen, urdfdom, boost | `riptide_dynamics` |
| **`riptide_dynamics`** (L1) | `IDynamicsModel`, `PinocchioModel`, `RobotState`, `EndEffectorTarget` | eigen, pinocchio | `riptide_control_core`, `riptide_control`, non-ROS example |
| **`riptide_control_core`** (L1) | de-ROS'd `IControlLaw` (core `ParamSource` + `std::function` logger), `operational_space.hpp`, `TaskSpace{Impedance,Lqr,Mpc,Template}::compute()` (no `PLUGINLIB_EXPORT_CLASS`) | riptide_dynamics, riptide_geometry, eigen | `riptide_control`, non-ROS example |
| **`riptide_geometry`** (L1, eigen-only leaf) | thruster allocation-matrix build + regularized pseudo-inverse; SO(3)/SE(3) error helpers | eigen | riptide_control_core, riptide_control, example |
| `riptide_msgs` (L2) | `EndEffectorTarget`, `DisturbanceCommand`, `ControlDebug` | rosidl (ament) | L3/L4 via ament, never Conan |
| `riptide_control` / `riptide_mujoco` (L3) | pluginlib controllers + `SystemInterface`; rclcpp→`ParamSource` adapter; POD↔msg marshalling | L1 cores + pinocchio/mujoco (Conan); rclcpp/pluginlib/hardware_interface/riptide_msgs (ament) | L4 apps |

Each L1 package has **≥3 real consumers** (listed above) before promotion, to
avoid over-designed one-consumer packages.

## Verified boundary violations to remove (audited baseline)

Confirmed against the current tree. These are the couplings later steps remove;
`tools/check_layer_boundaries.sh` reproduces the source-token subset exactly.

**Source-token leaks (grep audit output — the frozen baseline):**

```
[L1: riptide_dynamics]                              CLEAN
[L1: control-law core -> riptide_control_core/geometry]
  control_law_interface.hpp : rclcpp, pluginlib
  task_space_impedance.hpp  : rclcpp
  task_space_lqr.hpp        : rclcpp
  task_space_mpc.hpp        : rclcpp
  template_control_law.hpp  : rclcpp
  task_space_impedance.cpp  : pluginlib, rclcpp, PLUGINLIB_EXPORT_CLASS
  task_space_lqr.cpp        : pluginlib, rclcpp, PLUGINLIB_EXPORT_CLASS
  task_space_mpc.cpp        : pluginlib, rclcpp, PLUGINLIB_EXPORT_CLASS
  template_control_law.cpp  : pluginlib, rclcpp, PLUGINLIB_EXPORT_CLASS
  operational_space.hpp     : CLEAN (pure Eigen math)
```

(`control_law_interface.hpp`'s `pluginlib` hit is a comment reference; every
other hit is code. `operational_space.hpp` is already the clean geometry core.)

**Specific couplings, by category:**

- **(a) `IControlLaw` API is ROS-typed.** `control_law_interface.hpp:6-7` include
  `rclcpp/node_interfaces/node_{logging,parameters}_interface.hpp`; `on_configure`
  (l.27-30) takes `NodeParametersInterface::SharedPtr` + `NodeLoggingInterface::SharedPtr`.
  → Step 05 replaces these with a core `ParamSource` + `std::function` logger.
- **(b) Laws declare ROS parameters.** `task_space_impedance.cpp:19`,
  `task_space_lqr.cpp:22`, `task_space_mpc.cpp:22`, `template_control_law.cpp:18`
  call `params->declare_parameter(...)`; each law header restates the ROS-typed
  `on_configure`. → Step 05.
- **(c) EE controller carries messages.** `ee_stabilization_controller.hpp:19`
  includes `riptide_msgs/msg/control_debug.hpp`; publishes `ControlDebug`
  (`.cpp:160`). This stays in **L3** — the controller wrapper is ROS glue; only
  the `compute()` core moves to L1.
- **(d) Pinocchio via env root.** `riptide_dynamics/CMakeLists.txt:20-36` and
  `riptide_control/CMakeLists.txt:27-81` set `PINOCCHIO_ROOT` + `INSTALL_RPATH`.
  → Step 04/06 replace with `find_package(pinocchio)` from Conan.
- **(e) MuJoCo via env root.** `riptide_mujoco/CMakeLists.txt:26-58` use
  `MUJOCO_ROOT` `find_path`/`find_library` + hand-set `INSTALL_RPATH`.
  → Step 03/08 replace with `find_package(mujoco)` from Conan.

Categories (d)/(e) are CMake/build-system leaks handled by the build steps, not
the source-token gate.

## Decision — golden control-law baseline

The Step 05 de-ROS refactor changes the `IControlLaw` API but must not change any
`compute()` output. `riptide_control/test/test_control_laws.cpp` gains a golden
test that, for a fixed `RobotState` + `EndEffectorTarget` and default gains, dumps
the torque vector of all four laws to a checked-in fixture
(`riptide_control/test/golden/control_law_torques.csv`). Later steps must
reproduce it within tolerance; the ported unit tests are not the only safety net.

## Consequences

- A fixed target and a runnable invariant now exist; every later step is measured
  against them.
- `riptide_dynamics` is confirmed already-clean and is the first package to
  Conanize (Step 06). `operational_space.hpp` is confirmed clean and seeds
  `riptide_geometry`/`riptide_control_core`.
- `riptide_msgs` staying rosidl-only fixes the L1/L2 boundary: cores never see a
  generated message type.

See `docs/ARCHITECTURE.md` for the current runtime structure and
`docs/plan/` for the full step sequence.
