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
| `riptide_dynamics` | `IDynamicsModel` + `PinocchioModel` (whole-body dynamics) | **Phase 4** |
| `riptide_control` | `JointPdController`, `EeStabilizationController` + `TaskSpaceImpedance` | **Phase 4** |
| `riptide_mujoco` | MuJoCo `SystemInterface` + floating base + disturbances | **Phase 3** |
| `riptide_disturbance` | Disturbance scenario generator (current/sinusoid/impulse) | **Phase 3** |

## Prerequisites

- ROS 2 **Jazzy**
- A **MuJoCo SDK** (headers + `libmujoco`) for building `riptide_mujoco`.
  `CMakeLists` looks under `MUJOCO_ROOT` (env or `-D`), defaulting to
  `~/.mujoco/mujoco-3.10.0`.
- **Pinocchio** (C++) for `riptide_dynamics`. `CMakeLists` looks under
  `PINOCCHIO_ROOT` (env or `-D`), defaulting to `~/.local/pinocchio`. Either
  `sudo apt install ros-jazzy-pinocchio` (then set `PINOCCHIO_ROOT` to
  `/opt/ros/jazzy`), or build from source (python/collision OFF, urdf ON) into
  `~/.local/pinocchio`.

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

## Running it

```bash
# Task-space EE stabilization (Pinocchio impedance) under a current, in RViz:
ros2 launch riptide_bringup sim.launch.py \
    use_mock_hardware:=false controller:=ee rviz:=true disturbance:=sinusoid
```

Launch args: `use_mock_hardware` (true/false), `controller` (`pd` joint-hold /
`ee` task-space / `none`), `rviz` (true/false), `disturbance`
(none/steady_current/sinusoid/impulse). Tune the disturbance live, e.g.
`ros2 param set /disturbance_generator amplitude 20.0`.

## Current status: Phase 4 (task-space control with Pinocchio)

- **`PinocchioModel`** (`riptide_dynamics`): builds a reduced 7-DoF arm model
  from `urdf/fer_arm.urdf`, and each cycle composes fixed-base FK/Jacobian with
  the measured floating-base pose to expose the EE pose + Jacobian in the world
  frame (plus M and Coriolis/gravity).
- **`EeStabilizationController`** (`riptide_control`): reads the 7 arm joints +
  the 13-interface base sensor, assembles a `RobotState`, and delegates to a
  swappable `IControlLaw` plugin. Captures the EE target at activation.
- **`TaskSpaceImpedance`** control law: world-frame EE pose error → task wrench →
  `J^T` joint torques, with a nullspace posture task. Because the EE pose uses
  the measured base pose, base motion appears as task error and is fought.

**What works / the honest result.** The stack runs end-to-end (Pinocchio → task
space → torque → MuJoCo). Under a sinusoidal current the EE is **~2.3× steadier
than the base** (e.g. base ±20 cm, EE ±8 cm on the disturbance axis).

**The limiting factor** is that the AUV base is **free-floating with no
station-keeping** — with no restoring force it behaves like an integrator and
drifts far under a sustained current, and the arm's own reaction forces push the
light base around. Fully holding the EE therefore needs **base dynamic
positioning** (hull thrusters + an allocator/base controller) — the natural next
step (the plan's "thruster allocation" item). Also still open in the control
zoo: the LQR and MPC `IControlLaw` plugins (the architecture already supports
dropping them in alongside `TaskSpaceImpedance`).
