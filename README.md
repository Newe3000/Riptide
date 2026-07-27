# Riptide

Control-approach test rig for **end-effector stabilization of a floating-base
AUV + robotic arm** under heavy disturbances. A ROS 2 (Jazzy, C++) control stack
drives a MuJoCo simulation; the controller reads sensors and publishes joint
torques over the `ros2_control` seam.

See [`PROJECT_PLAN.md`](PROJECT_PLAN.md) for the full architecture and roadmap.

## Packages

| Package | Role | Status |
|---------|------|--------|
| `franka_description` | Vendored Franka "fer" (Panda) arm — full inertials + meshes | provided |
| `reach_bravo_description_mk2` | Original Reach Bravo 7 arm (unused; kept for reference) | legacy |
| `riptide_description` | AUV cube base + Franka arm assembly + `ros2_control` seam + MJCF | **Phase 2** |
| `riptide_msgs` | `EndEffectorTarget`, `DisturbanceCommand`, `ControlDebug` | **Phase 1** |
| `riptide_bringup` | Launch files | **Phase 1** |
| `riptide_dynamics` | `IDynamicsModel` / `RobotState` interfaces | interface only |
| `riptide_control` | `IControlLaw` + EE-stabilization controller | interface only |
| `riptide_mujoco` | MuJoCo `SystemInterface` (`MujocoSystem`) | **Phase 2** |

## Prerequisites

- ROS 2 **Jazzy**
- A **MuJoCo SDK** (headers + `libmujoco`) for building `riptide_mujoco`.
  `CMakeLists` looks under `MUJOCO_ROOT` (env or `-D`), defaulting to
  `~/.mujoco/mujoco-3.10.0`. Override for a different location/version.

## Build & run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Mock hardware (no physics) — Phase 1 seam check:
ros2 launch riptide_bringup sim.launch.py

# MuJoCo physics — Phase 2:
ros2 launch riptide_bringup sim.launch.py use_mock_hardware:=false
```

Verify the hardware seam (the `ros2 control` CLI isn't installed here, so query
the services directly):

```bash
# 7 effort command interfaces + state (pos/vel/effort) per joint
ros2 service call /controller_manager/list_hardware_interfaces \
  controller_manager_msgs/srv/ListHardwareInterfaces

# joint_state_broadcaster streaming /joint_states at ~500 Hz.
# Under MuJoCo with no controller, the arm sags from its home pose under gravity.
ros2 topic hz /joint_states
ros2 topic echo --once /joint_states --field position
```

## Current status: Phase 2 complete

- Cube AUV base (fixed to `world`) + **Franka fer (Panda)** arm with hand;
  full per-link inertials. Description parses (`check_urdf`) and expands.
- `ros2_control` seam works with **both** hardware plugins, selected by one arg:
  - `mock_components/GenericSystem` (Phase 1), and
  - **`riptide_mujoco/MujocoSystem`** (Phase 2) — embeds MuJoCo 3.10, maps the 7
    `fer_joint*` by name, reads position/velocity/effort, writes joint torques,
    and steps `mj_step` in the controller_manager loop at ~500 Hz.
- MuJoCo scene `riptide_description/mujoco/riptide.xml` — Menagerie Panda with
  torque actuators on the cube base (see that dir's README for provenance).
- Verified live: MuJoCo loads/activates, `/joint_states` at ~500 Hz, arm evolves
  under gravity (stepping proven); commanded torque drives the joints.

**Phase 3 next:** replace the fixed base with a `<freejoint/>` (floating base) +
expose base pose/twist as state interfaces, then add hydrodynamics + the
disturbance applier. **Phase 4:** the four `IControlLaw` plugins.
