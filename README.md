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
| `riptide_disturbance` | Disturbance scenario generator (current/sinusoid/impulse/stochastic) | **Phase 3** |
| `riptide_eval` | Benchmark runner: same disturbance vs each law → CSV + plots | **Phase 5** |

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

Shortcut: **`source build.sh`** does the three lines below in one step (sources
ROS, builds with `--symlink-install`, sources the overlay). Source it — don't run
`./build.sh` — so the environment stays in your shell. Extra args pass to colcon,
e.g. `source build.sh --packages-select riptide_control`.

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
(none/steady_current/sinusoid/impulse/**stochastic**), `current_speed` (m/s — flow
speed for the current scenarios), `disturbance_amplitude` (N — the `impulse` hit),
`water` (true/false — the fluid medium), `fixed_base` (true/false — non-floating base).
Tune the current live, e.g. `ros2 param set /disturbance_generator current_speed 0.5`.

**Reusability toggles** (MuJoCo path only): `water:=false` zeroes the fluid
medium so no water effects act at all — drag, added mass, and the ocean current
all scale with the medium density/viscosity (gravity stays 0 / neutral buoyancy;
that is a separate axis). `fixed_base:=true` activates a weld constraint that pins
the AUV base to the world, turning it into a fixed-base manipulator (pair it with
`base_control:=false`, since station-keeping is moot). Both are hardware
parameters plumbed launch arg → xacro → `MujocoSystem`.

### Viewing the sim

Two independent views — pass either, or both:

* `rviz:=true` — RViz, driven by `/joint_states` + TF (robot model + frames).
* `mujoco_viewer:=true` — MuJoCo's **native viewer window**, opened in parallel. It
  is a read-only mirror of the live sim (arm from `/joint_states`, floating base
  from `/riptide/odom`): it loads its own copy of the MJCF and only syncs poses,
  never stepping physics, so it cannot affect the control loop. Requires the
  `mujoco` Python package in the ROS Python — `pip install --user mujoco==3.10.0`
  (match the SDK version) — and the MuJoCo path (`use_mock_hardware:=false`).

```bash
# Same run, seen in both RViz and the MuJoCo viewer:
ros2 launch riptide_bringup sim.launch.py \
    use_mock_hardware:=false controller:=ee disturbance:=sinusoid \
    rviz:=true mujoco_viewer:=true
```

### Teleoperating the end-effector

Drive the desired EE Cartesian pose live with a small GUI (`ee_gui:=true`,
requires `controller:=ee`):

```bash
ros2 launch riptide_bringup sim.launch.py \
    use_mock_hardware:=false controller:=ee ee_gui:=true rviz:=true mujoco_viewer:=true
```

Press-and-**hold** the arrows to move the target at a constant speed — `±X/±Y/±Z`
for position and `±Roll/±Pitch/±Yaw` for orientation — and the EE tracks it in
real time. The GUI publishes `geometry_msgs/PoseStamped` on **`/riptide/ee_target`**
(world frame); the controller tracks any such message, so you can also script it:

```bash
ros2 topic pub -r 20 /riptide/ee_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.35, y: 0.0, z: 1.35}, orientation: {w: 1.0}}}"
```

The controller echoes its current target (latched) on `/riptide/ee_target/current`,
which the GUI reads once at startup to seed itself — so taking control doesn't snap
the arm. The GUI needs Tkinter (`sudo apt install python3-tk`).

### Example scenarios

Three representative configurations, a progression from a plain manipulator to
the full AUV under load (all on the MuJoCo physics path, with EE stabilization and
RViz on):

**1 — Fixed base (dry 7-DoF manipulator).** The base is welded to the world and
the fluid medium is off, so this is an ordinary fixed-base arm holding its EE
target — no floating dynamics, no water. `base_control:=false` since
station-keeping is moot when the base is pinned.

```bash
ros2 launch riptide_bringup sim.launch.py \
    use_mock_hardware:=false controller:=ee rviz:=true \
    fixed_base:=true water:=false base_control:=false
```

**2 — Floating base + water (passive).** The AUV floats (6-DOF free joint) in
neutrally-buoyant water with no external current. The fully-actuated hull thrusters
hold full pose (position + level attitude + heading) against the arm's reaction,
while the water's drag + added mass damp the motion. To instead let the hull tilt
freely with the arm (a compliant floating base), zero the attitude gains:
`ros2 param set /base_thruster_controller kp_roll 0.0` (and `kp_pitch`).

```bash
ros2 launch riptide_bringup sim.launch.py \
    use_mock_hardware:=false controller:=ee rviz:=true \
    fixed_base:=false water:=true base_control:=true disturbance:=none
```

**3 — Floating base + water + strong current.** Adds a strong turbulent ocean
current (2.5 m/s mean flow) that pushes the whole structure — hull and every arm
link. The EE law and base thrusters fight to hold station under the load.

```bash
ros2 launch riptide_bringup sim.launch.py \
    use_mock_hardware:=false controller:=ee rviz:=true \
    fixed_base:=false water:=true base_control:=true \
    disturbance:=stochastic current_speed:=2.5
```

**The current acts on the whole structure.** The flow scenarios
(`steady_current` / `sinusoid` / `stochastic`) publish a current *velocity*
(`/riptide/current`, m/s, world frame) which the MuJoCo `SystemInterface` feeds
into the fluid model as `wind`. Drag is then computed on **every** submerged geom
relative to `(v_geom − v_current)`, so the current pushes the hull **and every arm
link** — distributed and velocity-dependent — rather than as one lumped force on
the base. `impulse` is instead a localized point wrench on the base (a bump),
published on `/riptide/disturbance`.

`stochastic` is a **realistic turbulent current**: a mean flow (`current_speed`)
plus first-order Gauss–Markov (Ornstein–Uhlenbeck) turbulence on the current
*velocity* — temporally correlated, mean-reverting colored noise (Fossen's
standard environmental-load model), with lateral components a clean sinusoid
lacks. The distributed drag turns the fluctuating flow into fluctuating force
*and* torque on the whole body automatically. Knobs: `turbulence_std` (m/s,
per-axis velocity std), `correlation_time` (s), and `seed` (≥0 = reproducible for
fair controller comparisons, <0 = nondeterministic).

```bash
ros2 launch riptide_bringup sim.launch.py \
    use_mock_hardware:=false controller:=ee control_law:=mpc disturbance:=stochastic
```

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

### Benchmark (quantitative comparison)

`riptide_eval` replays the **same** disturbance (fixed seed) against each control
law and writes comparison metrics + plots:

```bash
ros2 run riptide_eval benchmark                       # all laws x {sinusoid, stochastic}
ros2 run riptide_eval benchmark --laws impedance,lqr,mpc \
    --scenarios stochastic --duration 20 --output-dir ~/riptide_bench
```

It launches the full stack per combination, records `/riptide/control_debug`
(EE error, joint torque, solve time) + `/riptide/odom`, then emits into the
output dir: `summary.csv` (EE position/orientation RMS + max, control effort,
base station-keeping RMS, solver time), `raw_<law>_<scenario>.csv` time series,
an `ee_error_<scenario>.png` overlay, and grouped-bar charts for accuracy,
effort, and solver cost. The EE law also publishes `riptide_msgs/ControlDebug`
at 50 Hz whenever it runs, so you can log/plot any run live.

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
  thrusters hold the base, or lower the current
  (`ros2 param set /disturbance_generator current_speed 0.4`).
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
- **Hull thrusters** (`generate_scene.py`): 7 force actuators making the base
  **fully actuated** (all 6 DOF) — 1 surge (±x), 4 vertical corner thrusters (±z:
  heave + roll + pitch), and 2 lateral thrusters (±y at ±x: sway + yaw). Exposed
  through the `ros2_control` seam as a `<gpio>` and mapped by name to MuJoCo
  `<motor>`s, so adding a thruster is a declarative change (scene + xacro + config).
- **`BaseThrusterController`** (`riptide_control`): reads the base sensor and
  allocates a full 6-DOF PD station-keeping wrench `[Fx, Fy, Fz, Mx, My, Mz]` to
  the thrusters via the pseudo-inverse of the 6×N geometry-derived allocation
  matrix — position hold (x/y/z), attitude leveling (roll/pitch), and heading
  hold (yaw). Every gain is independent in the yaml, so any DOF can be left
  compliant (set its gain pair to 0).

**What works / the honest result.** With `base_control:=true` under a current the
fully-actuated base **holds station in all 6 DOF**: position (x/y/z) within a few
cm of target, roll/pitch level (`qx, qy ≈ 0`), and heading (yaw) held — vs. the
free-floating base drifting away. The EE is correspondingly steady. The allocation
is decoupled (a pure sway command fires only the lateral thrusters, pure yaw only
their differential), verified by reproduction error ~1e-6 per DOF.

**Remaining limits.** Still open in the control zoo: the LQR and MPC `IControlLaw`
plugins (the architecture already supports dropping them in alongside
`TaskSpaceImpedance`).
