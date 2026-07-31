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
| `riptide_control` | `JointPdController`, `EeStabilizationController` + `TaskSpaceImpedance`, `BaseThrusterController` | **Phase 4b** |
| `riptide_mujoco` | MuJoCo `SystemInterface` + floating base + hull thrusters + disturbances | **Phase 4b** |
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
`ee` task-space / `none`), `control_law` (`impedance` / `lqr` / `mpc` — the EE
law when `controller:=ee`), `base_control` (true/false — run the hull-thruster base
dynamic-positioning controller), `rviz` (true/false), `disturbance`
(none/steady_current/sinusoid/impulse). Tune the disturbance live, e.g.
`ros2 param set /disturbance_generator amplitude 20.0`.

The EE control law is a hot-swappable `IControlLaw` plugin. Compare the Cartesian
impedance law against the operational-space LQR (optimal task gains from the cost
weights — see [`docs/control_theory.md`](docs/control_theory.md) §3):

```bash
ros2 launch riptide_bringup sim.launch.py use_mock_hardware:=false controller:=ee control_law:=impedance disturbance:=sinusoid
ros2 launch riptide_bringup sim.launch.py use_mock_hardware:=false controller:=ee control_law:=lqr       disturbance:=sinusoid
ros2 launch riptide_bringup sim.launch.py use_mock_hardware:=false controller:=ee control_law:=mpc       disturbance:=sinusoid
```

`mpc` (§4 of the notes) adds input constraints + a receding horizon on top of the
LQR cost, solved as per-axis QPs each cycle by a self-contained fast-gradient
method (no external solver); its DARE terminal cost makes it provably stabilizing.

To see what the thrusters buy you, compare the base drift with and without them:

```bash
# Base holds station (surge/heave + roll/pitch leveling):
ros2 launch riptide_bringup sim.launch.py \
    use_mock_hardware:=false controller:=ee base_control:=true  disturbance:=sinusoid
# Free-floating base (drifts away under the current):
ros2 launch riptide_bringup sim.launch.py \
    use_mock_hardware:=false controller:=ee base_control:=false disturbance:=sinusoid
```

## Troubleshooting

- **The robot/base "teleports" or jumps around in RViz.** Almost always a
  leftover sim from a previous run: Ctrl-C doesn't always reap `ros2_control_node`
  (and closing the terminal orphans it). Each `ros2_control_node` embeds its own
  MuJoCo world, so two of them publish two diverging `/joint_states` + `/tf`
  streams and RViz jumps between them. `sim.launch.py` now reaps stale
  `ros2_control_node` / `robot_state_publisher` / `rviz2` / `disturbance_generator`
  before starting, so a fresh launch is always a single clean sim. To check by
  hand: `ps -C ros2_control_node` should list exactly one process while running.
- **The arm slams into its joint limits after a minute or two** (with
  `base_control:=false`). Without base station-keeping the free-floating base
  slowly drifts out of the arm's reach, so the arm saturates trying to hold the
  fixed world target. Run with `base_control:=true` (the default) so the hull
  thrusters hold the base, or lower the disturbance
  (`ros2 param set /disturbance_generator amplitude 20.0`).
- **"Overrun detected! ... missed its desired rate."** A benign warning: the
  control loop occasionally exceeds its period without real-time (FIFO) priority,
  which isn't available here. It self-corrects and doesn't affect the result.

## Current status: Phase 4b (base dynamic positioning)

- **`PinocchioModel`** (`riptide_dynamics`): builds a reduced 7-DoF arm model
  from `urdf/fer_arm.urdf`, and each cycle composes fixed-base FK/Jacobian with
  the measured floating-base pose to expose the EE pose + Jacobian in the world
  frame (plus M and Coriolis/gravity).
- **`EeStabilizationController`** (`riptide_control`): reads the 7 arm joints +
  the 13-interface base sensor, assembles a `RobotState`, and delegates to a
  swappable `IControlLaw` plugin. Captures the EE target on the first finite
  cycle.
- **`TaskSpaceImpedance`** control law: world-frame EE pose error → task wrench →
  `J^T` joint torques, with a nullspace posture task.
- **Hull thrusters** (`generate_scene.py`): 5 force actuators on the base — 1
  surge (±x) + 4 vertical corner thrusters (±z). Exposed through the
  `ros2_control` seam as a `<gpio>` and mapped by name to MuJoCo `<motor>`s, so
  adding a thruster is a declarative change (scene + xacro + config).
- **`BaseThrusterController`** (`riptide_control`): reads the base sensor and
  allocates a PD station-keeping wrench `[Fx, Fz, Mx, My]` to the thrusters via
  the pseudo-inverse of the geometry-derived allocation matrix — surge/heave
  position hold + roll/pitch leveling.

**What works / the honest result.** With `base_control:=true` under a sinusoidal
surge current the base **holds station**: surge x stays within a few cm of
target, heave z is held exactly, and roll/pitch stay level (`qx, qy ≈ 0`) — vs.
the free-floating base drifting past 1 m and away. The EE is correspondingly
steady in the controlled axes.

**Remaining limits.** **Sway (y) and yaw are unactuated** (no thrusters were
requested for them), so the base still slowly drifts in y / yaw under the arm's
reaction — adding a sway/yaw thruster is now a config + scene change. Still open
in the control zoo: the LQR and MPC `IControlLaw` plugins (the architecture
already supports dropping them in alongside `TaskSpaceImpedance`).
