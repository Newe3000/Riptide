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
| `riptide_control` | `JointPdController` + `IControlLaw` interface | **Phase 2** |
| `riptide_mujoco` | MuJoCo `SystemInterface` + floating base + disturbances | **Phase 3** |
| `riptide_disturbance` | Disturbance scenario generator (current/sinusoid/impulse) | **Phase 3** |

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

## Running it

```bash
# MuJoCo physics, PD hold controller, RViz, steady-current disturbance:
ros2 launch riptide_bringup sim.launch.py \
    use_mock_hardware:=false controller:=pd rviz:=true disturbance:=sinusoid
```

Launch args: `use_mock_hardware` (true/false), `controller` (pd/none),
`rviz` (true/false), `disturbance` (none/steady_current/sinusoid/impulse).
Change the disturbance live: `ros2 param set /disturbance_generator scenario impulse`.

## Current status: Phase 3 (floating base + hydro + disturbances) complete

- **Floating base**: `auv_base` is a MuJoCo `<freejoint/>` in a fluid medium
  (drag + added mass). Neutral buoyancy is modelled as zero gravity; validated
  that the vehicle hovers at rest and a current drives it to a drag-limited
  speed. (Fossen-style explicit buoyancy is a documented later upgrade.)
- **Base sensing**: pose + twist exposed as a 13-interface `ros2_control` sensor
  (`auv_base/position.*`, `orientation.*`, `linear_velocity.*`,
  `angular_velocity.*`); `MujocoSystem` also broadcasts `world→auv_base_link` TF
  and `/riptide/odom`.
- **Disturbances**: `riptide_disturbance` publishes `DisturbanceCommand`;
  `MujocoSystem` applies it to `xfrc_applied` and echoes `/riptide/disturbance/
  ground_truth`. Verified the base drifts/oscillates under the applied wrench.
- Earlier phases still hold: mock + MuJoCo hardware, 7 `fer_joint*` effort seam,
  `JointPdController` holding the arm.

**Note on the exit criterion.** The Phase 3 *infrastructure* (floating base,
hydro, disturbances, base sensing) is done. Making the **end-effector** hold
steady while the base is disturbed needs the task-space controller — that is
**Phase 4** (the `IControlLaw` plugins reading the new base-state interfaces).
Today the joint-space PD holds joint angles, so the EE still moves with the base.
