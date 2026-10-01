# Riptide — Architecture

Riptide is a control-approach test rig for **end-effector stabilization of a
floating-base AUV + robotic arm** under heavy disturbances. A ROS 2 (Jazzy, C++)
control stack drives a MuJoCo simulation; controllers read sensors and publish
joint torques + thruster forces across the `ros2_control` seam.

This document maps the code structure and the interfaces between its parts. All
diagrams are Mermaid and render inline on GitHub and in most IDEs.

---

## 1. Layered view

The stack separates into four layers. The single boundary between the simulator
and every controller is the **`ros2_control` seam**; the single boundary between
a control law and the physics it reasons about is **`IDynamicsModel`**.

```mermaid
flowchart TB
    subgraph teleop["Teleop / evaluation"]
        GUI["ee_target_gui.py<br/>(Tkinter teleop)"]
        VIEW["mujoco_viewer.py<br/>(MuJoCo mirror)"]
        RVIZ["rviz2"]
        BENCH["riptide_eval<br/>benchmark"]
    end

    subgraph control["Control layer (ros2_control_node)"]
        EE["EeStabilizationController"]
        BASE["BaseThrusterController"]
        PD["JointPdController"]
        LAW["IControlLaw plugin<br/>impedance / lqr / mpc"]
        DYN["IDynamicsModel<br/>(PinocchioModel)"]
    end

    subgraph plant["Plant layer (ros2_control_node)"]
        HW["MujocoSystem<br/>(hardware SystemInterface)"]
        MJ["MuJoCo physics"]
    end

    subgraph env["Environment"]
        DIST["disturbance_generator"]
    end

    GUI -->|"/riptide/ee_target"| EE
    EE -->|"/riptide/ee_target/current"| GUI
    EE --> LAW --> DYN
    EE -.ros2_control seam.-> HW
    BASE -.ros2_control seam.-> HW
    PD -.ros2_control seam.-> HW
    HW <--> MJ
    DIST -->|"/riptide/current + /riptide/disturbance"| HW
    HW -->|"/riptide/odom, /tf"| RVIZ
    HW -->|"/riptide/odom"| VIEW
    EE -->|"/riptide/control_debug"| BENCH
    HW -->|"/riptide/odom"| BENCH

    classDef sim fill:#0b3d5c,color:#fff,stroke:#093247;
    classDef ctl fill:#1f6f43,color:#fff,stroke:#124a2c;
    class HW,MJ sim;
    class EE,BASE,PD,LAW,DYN ctl;
```

---

## 2. Package map & build/link dependencies

Eight `riptide_*` packages (the vendored `franka_description` and legacy
`reach_bravo_description_mk2` are asset-only and omitted here).

```mermaid
flowchart LR
    subgraph ext["External (non-ROS) libraries"]
        EIGEN["Eigen3<br/>(rosdep: eigen)"]
        PIN["Pinocchio<br/>(source, PINOCCHIO_ROOT)"]
        MJC["MuJoCo SDK<br/>(MUJOCO_ROOT)"]
        PY["numpy / matplotlib<br/>mujoco / glfw / tkinter (pip)"]
    end

    MSGS["riptide_msgs<br/>(interfaces)"]
    DYNP["riptide_dynamics<br/>(C++ lib · ROS-agnostic)"]
    CTRL["riptide_control<br/>(controllers + IControlLaw)"]
    MUJ["riptide_mujoco<br/>(hardware plugin)"]
    DISTP["riptide_disturbance<br/>(py node)"]
    EVAL["riptide_eval<br/>(py benchmark)"]
    DESC["riptide_description<br/>(URDF/MJCF/config assets)"]
    BRING["riptide_bringup<br/>(launch + helper nodes)"]

    DYNP --> EIGEN
    DYNP --> PIN
    CTRL --> DYNP
    CTRL --> MSGS
    CTRL --> EIGEN
    CTRL --> PIN
    MUJ --> MSGS
    MUJ --> MJC
    DISTP --> MSGS
    EVAL --> MSGS
    EVAL --> PY
    BRING --> DESC
    BRING --> DISTP
    BRING --> PY

    %% runtime-only (name reference, not a build dep)
    DESC -.names plugin.-> MUJ
    DESC -.names plugin.-> CTRL
    EVAL -.launches.-> BRING

    classDef pure fill:#1f6f43,color:#fff,stroke:#124a2c;
    class DYNP pure;
```

Solid arrows are build/link dependencies; dashed arrows are runtime references
(a package names another's plugin/launch by string, with no compile-time link).
**`riptide_dynamics` (green) is fully ROS-agnostic** — the single most important
fact for the Conan-packaging roadmap (see §7).

| Package | Language | Role | Key deps |
|---|---|---|---|
| `riptide_msgs` | IDL | `EndEffectorTarget`, `DisturbanceCommand`, `ControlDebug` | std_msgs, geometry_msgs |
| `riptide_dynamics` | C++ lib | `IDynamicsModel`, `PinocchioModel`, `RobotState` | **Eigen, Pinocchio only** |
| `riptide_control` | C++ | 3 controllers + 4 `IControlLaw` plugins | riptide_dynamics, riptide_msgs, pluginlib, controller_interface |
| `riptide_mujoco` | C++ | `MujocoSystem` hardware plugin | riptide_msgs, MuJoCo, hardware_interface, tf2_ros, nav_msgs |
| `riptide_disturbance` | Python | `disturbance_generator` node | rclpy, riptide_msgs, geometry_msgs |
| `riptide_eval` | Python | `benchmark` runner | rclpy, riptide_msgs, numpy, matplotlib |
| `riptide_description` | assets | URDF/MJCF, the ros2_control seam, controller config | xacro (build) |
| `riptide_bringup` | launch + py | `sim.launch.py` + `mujoco_viewer.py` + `ee_target_gui.py` | riptide_description, riptide_disturbance |

---

## 3. Runtime graph — nodes & topics

Every edge is a ROS topic labelled with its message type. The controllers and
the MuJoCo hardware all live inside the single `ros2_control_node` process
(dashed box); they are drawn separately because each owns distinct topics and
`ros2_control` interfaces.

```mermaid
flowchart LR
    GEN["disturbance_generator"]
    subgraph cm["ros2_control_node (controller_manager)"]
        HW["MujocoSystem<br/>(hardware)"]
        JSB["joint_state_broadcaster"]
        EE["ee_stabilization_controller"]
        BASE["base_thruster_controller"]
    end
    RSP["robot_state_publisher"]
    GUI["ee_target_gui"]
    VIEW["mujoco_viewer"]
    RVIZ["rviz2"]
    BENCH["benchmark (riptide_eval)"]

    GEN -->|"/riptide/current<br/>geometry_msgs/Vector3Stamped"| HW
    GEN -->|"/riptide/disturbance<br/>riptide_msgs/DisturbanceCommand"| HW
    HW -->|"/riptide/disturbance/ground_truth"| BENCH
    HW -->|"/riptide/odom<br/>nav_msgs/Odometry"| BENCH
    HW -->|"/riptide/odom"| VIEW
    HW -->|"/tf (world→auv_base_link)"| RVIZ
    JSB -->|"/joint_states<br/>sensor_msgs/JointState"| RSP
    JSB -->|"/joint_states"| VIEW
    RSP -->|"/tf, /robot_description"| RVIZ
    EE -->|"/riptide/control_debug<br/>riptide_msgs/ControlDebug"| BENCH
    EE -->|"/riptide/ee_target/current<br/>geometry_msgs/PoseStamped (latched)"| GUI
    GUI -->|"/riptide/ee_target<br/>geometry_msgs/PoseStamped"| EE
```

**Topic reference:**

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/riptide/ee_target` | `geometry_msgs/PoseStamped` | ee_target_gui → EeStabilizationController |
| `/riptide/ee_target/current` | `geometry_msgs/PoseStamped` (latched) | EeStabilizationController → teleop (seed) |
| `/riptide/control_debug` | `riptide_msgs/ControlDebug` | EeStabilizationController → riptide_eval |
| `/riptide/odom` | `nav_msgs/Odometry` | MujocoSystem → eval, viewer |
| `/riptide/current` | `geometry_msgs/Vector3Stamped` | disturbance_generator → MujocoSystem (fluid `wind`) |
| `/riptide/disturbance` | `riptide_msgs/DisturbanceCommand` | disturbance_generator → MujocoSystem (point wrench) |
| `/riptide/disturbance/ground_truth` | `riptide_msgs/DisturbanceCommand` | MujocoSystem → eval |
| `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster → RSP, viewer |
| `/tf`, `/robot_description` | tf2_msgs / std_msgs | MujocoSystem + robot_state_publisher |

---

## 4. The `ros2_control` seam

`riptide_description/urdf/riptide.ros2_control.xacro` declares one system,
`RiptideSystem`, whose `<hardware>` plugin is swappable (`mock_components/
GenericSystem` for the no-physics path, `riptide_mujoco/MujocoSystem` for real
physics). Controllers address the exposed interfaces **by name**, never by index,
so adding an actuator or sensor is a declarative change to the xacro.

```mermaid
flowchart TB
    subgraph hwif["MujocoSystem — exported interfaces"]
        direction LR
        AJ["arm joints fer_joint1..7<br/>cmd: effort · state: pos/vel/effort"]
        SEN["sensor 'auv_base'<br/>state: 13 (pose 7 + twist 6)"]
        THR["gpio 'thrusters'<br/>cmd+state: thr_surge, thr_vfl/vfr/vbl/vbr,<br/>thr_lat_fwd, thr_lat_aft"]
    end

    EE["EeStabilizationController"]
    BASE["BaseThrusterController"]
    PD["JointPdController"]

    AJ -->|reads pos/vel| EE
    SEN -->|reads base pose+twist| EE
    EE -->|writes effort| AJ
    SEN -->|reads base pose+twist| BASE
    BASE -->|writes force| THR
    AJ -->|reads pos/vel| PD
    PD -->|writes effort| AJ

    classDef hw fill:#0b3d5c,color:#fff,stroke:#093247;
    class AJ,SEN,THR hw;
```

- **Arm joints** (`fer_joint1..7`): effort command, position/velocity/effort state.
- **`auv_base` sensor** (13 state interfaces): `position.{x,y,z}`,
  `orientation.{x,y,z,w}`, `linear_velocity.{x,y,z}`, `angular_velocity.{x,y,z}`.
- **`thrusters` gpio** (7 command + 7 state): one force interface per thruster;
  the geometry (positions/axes) is mirrored between `generate_scene.py` and
  `riptide_controllers.yaml`.
- **Hardware params** (MuJoCo path): `mjcf_model`, `floating_base_joint`,
  `base_link_frame`, `water`, `fixed_base`.

`EeStabilizationController` and `BaseThrusterController` claim **disjoint** command
interfaces (arm effort vs thruster force), so they run simultaneously.

---

## 5. Plugin & class interfaces

Three plugin families, each behind a stable base class, make the stack
hot-swappable at runtime.

```mermaid
classDiagram
    class SystemInterface {
        <<hardware_interface>>
        on_init() / read() / write()
        export_state_interfaces()
        export_command_interfaces()
    }
    class ControllerInterface {
        <<controller_interface>>
        command_interface_configuration()
        state_interface_configuration()
        update()
    }
    class IControlLaw {
        <<riptide_control>>
        on_configure(params, logging, model)
        compute(state, target, dt) VectorXd
        reset()
    }
    class IDynamicsModel {
        <<riptide_dynamics>>
        update(RobotState)
        massMatrix() / nonlinear()
        jacobian(frame) / framePose(frame)
        hydroForces(state)
    }

    SystemInterface <|-- MujocoSystem
    ControllerInterface <|-- JointPdController
    ControllerInterface <|-- EeStabilizationController
    ControllerInterface <|-- BaseThrusterController
    IControlLaw <|-- TaskSpaceImpedance
    IControlLaw <|-- TaskSpaceLqr
    IControlLaw <|-- TaskSpaceMpc
    IControlLaw <|-- TemplateControlLaw
    IDynamicsModel <|-- PinocchioModel

    EeStabilizationController o-- IControlLaw : loads via pluginlib
    EeStabilizationController ..> PinocchioModel : builds from arm URDF
    IControlLaw ..> IDynamicsModel : queries M, C+g, J, pose
```

- `SystemInterface` / `ControllerInterface` are **pluginlib** exports registered
  in `plugins.xml` / `controller_plugins.xml` — discovered by the
  controller_manager at runtime.
- `IControlLaw` is a pluginlib base **defined by `riptide_control`** itself
  (`control_law_plugins.xml`); `EeStabilizationController` picks the concrete law
  from the `control_law` parameter.
- `IDynamicsModel` is a **plain C++ virtual interface** (not pluginlib) — a
  control law owns one instance and refreshes it each cycle, keeping physics off
  the real-time I/O path. `PinocchioModel` is the only backend today.
- `TaskSpace*` laws share `operational_space.hpp` (task-space error → wrench →
  `Jᵀ` torque + dynamically-consistent nullspace); they differ only in how the
  desired task acceleration is produced.

---

## 6. One control cycle

At ~250 Hz the controller_manager drives read → controllers.update → write.

```mermaid
sequenceDiagram
    participant CM as controller_manager
    participant HW as MujocoSystem
    participant EE as EeStabilizationController
    participant LAW as IControlLaw
    participant DYN as PinocchioModel
    participant BASE as BaseThrusterController

    CM->>HW: read()
    HW-->>CM: joint pos/vel/effort, base 13, thruster force
    CM->>EE: update()
    EE->>EE: read_state() → RobotState
    EE->>DYN: update(state)  (FK, J, M, C+g in world)
    EE->>LAW: compute(state, target, dt)
    LAW->>DYN: massMatrix / jacobian / nonlinear
    LAW-->>EE: joint torque vector
    EE-->>CM: write arm effort commands
    CM->>BASE: update()
    BASE->>BASE: allocate wrench → per-thruster force
    BASE-->>CM: write thruster commands
    CM->>HW: write()
    HW->>HW: apply torques + forces + disturbance + current, mj_step
    HW-->>HW: publish /riptide/odom + /tf (~50 Hz)
```

The desired target enters asynchronously: `ee_target_gui` publishes
`/riptide/ee_target`, whose latest value the EE controller latches into `target`
(overriding its startup capture); `disturbance_generator` publishes the flow
`/riptide/current` and point `/riptide/disturbance` the hardware applies in
`write()`.

---

## 7. External dependencies & the packaging seam

How each non-ROS dependency is located today — the starting point for the
Conan-packaging roadmap (`docs/plan/`).

| Library | Used by | Found via |
|---|---|---|
| Eigen3 | riptide_dynamics, riptide_control | rosdep `eigen` + `eigen3_cmake_module` |
| Pinocchio | riptide_dynamics, riptide_control | **source build**, CMake `PINOCCHIO_ROOT` (not rosdep) |
| MuJoCo SDK | riptide_mujoco | CMake `MUJOCO_ROOT` (not rosdep) |
| numpy, matplotlib | riptide_eval | rosdep `python3-numpy`, `python3-matplotlib` |
| mujoco, glfw (pip) | mujoco_viewer.py | pip in the ROS Python |
| tkinter | ee_target_gui.py | system `python3-tk` |

**ROS-agnostic core vs ROS glue** — the seam a Conan split runs along:

```mermaid
flowchart LR
    subgraph agnostic["ROS-agnostic C++ (Conan-ready)"]
        DYN["riptide_dynamics<br/>IDynamicsModel · PinocchioModel · RobotState"]
        OPS["riptide_control law core<br/>operational_space.hpp + TaskSpace* math"]
    end
    subgraph glue["ROS / ros2_control glue"]
        CTRLW["controller wrappers<br/>Ee/Base/JointPd + pluginlib exports"]
        HWW["MujocoSystem SystemInterface"]
        NODES["disturbance_generator · viewer · gui · launch"]
    end
    OPS --> DYN
    CTRLW --> OPS
    CTRLW --> DYN
    HWW -.-> MJC2["MuJoCo"]
    DYN --> PIN2["Pinocchio + Eigen"]

    classDef pure fill:#1f6f43,color:#fff,stroke:#124a2c;
    class DYN,OPS pure;
```

- `riptide_dynamics` is **already** a standalone C++ library — no
  rclcpp/ros2_control/pluginlib anywhere; only Eigen + Pinocchio. It is the
  cleanest first Conan package.
- The `TaskSpace*` control-law **math** (`operational_space.hpp` + the compute
  cores) is ROS-agnostic in substance but currently compiled inside the
  `riptide_control` ament library alongside the pluginlib controller wrappers;
  extracting it is the second Conan candidate.
- Everything that includes `rclcpp`, `controller_interface`, `hardware_interface`,
  or `rclpy` is ROS glue and stays in ROS packages that *consume* the Conan
  libraries.

The target Conan package graph, the downward-only layer invariant, and the
verified boundary-violation baseline are frozen in
[`docs/adr/0001-conan-package-taxonomy.md`](adr/0001-conan-package-taxonomy.md)
and enforced by `tools/check_layer_boundaries.sh`. See `docs/plan/` for the
step-by-step roadmap to a multi-Conan-package project.
