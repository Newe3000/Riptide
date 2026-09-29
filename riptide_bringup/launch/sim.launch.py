"""Bring up the Riptide sim: robot_state_publisher + ros2_control + broadcasters.

Defaults to mock hardware (mock_components/GenericSystem); switch to MuJoCo with
`use_mock_hardware:=false` — no other launch change needed.
"""

import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _reap_stale_sim_processes():
    """Kill leftover sim processes from an earlier run before starting a new one.

    Each ``ros2_control_node`` embeds its OWN MuJoCo world. An orphaned one keeps
    stepping physics and publishing a second, diverging ``/joint_states`` + ``/tf``
    stream, so RViz jumps between the two worlds every message and the robot and
    base appear to "teleport". Reaping guarantees each launch starts from a single
    clean sim.
    """
    for name in (
        "ros2_control_node",        # embeds the MuJoCo world
        "robot_state_publisher",    # holds a latched /robot_description
        "mock_base_tf",             # static world->auv_base_link TF
        "rviz2",
        "disturbance_generator",
        # Match the script filename, not bare "mujoco_viewer": that string is also
        # the launch arg (mujoco_viewer:=true), so `pkill -f mujoco_viewer` would
        # match and terminate this launch process (and the calling shell).
        "mujoco_viewer.py",
        "ee_target_gui.py",
    ):
        subprocess.run(
            ["pkill", "-9", "-f", name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
        )


def generate_launch_description():
    # Pre-flight: never let a previous run's sim linger under this one.
    _reap_stale_sim_processes()

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    water = LaunchConfiguration("water")
    fixed_base = LaunchConfiguration("fixed_base")
    rviz = LaunchConfiguration("rviz")
    disturbance = LaunchConfiguration("disturbance")

    description_pkg = FindPackageShare("riptide_description")

    # Expand the top-level xacro into a robot_description string.
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([description_pkg, "urdf", "riptide.urdf.xacro"]),
            " use_mock_hardware:=", use_mock_hardware,
            " water:=", water,
            " fixed_base:=", fixed_base,
        ]),
        value_type=str,
    )

    controllers_yaml = PathJoinSubstitution(
        [description_pkg, "config", "riptide_controllers.yaml"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # controller_manager reads the description from the /robot_description topic
    # published (latched) by robot_state_publisher.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_yaml],
    )

    # The spawner blocks until the controller_manager services are available,
    # so no explicit ordering/event handler is needed.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Which controller to spawn (controller:=pd|ee|none).
    pd_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_pd_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("controller"), "' == 'pd'"])),
    )
    # Task-space EE stabilization (Pinocchio). The control law (impedance | lqr)
    # is selected by a small per-law param file passed to the spawner, which
    # overrides `control_law` on the controller — hot-swappable per the design.
    law_param_file = PathJoinSubstitution([
        description_pkg, "config",
        PythonExpression(["'law_' + '", LaunchConfiguration("control_law"), "' + '.yaml'"]),
    ])
    # Hydrodynamic-drag compensation toggle (hydro:=true|false).
    hydro_param_file = PathJoinSubstitution([
        description_pkg, "config",
        PythonExpression(["'hydro_' + '", LaunchConfiguration("hydro"), "' + '.yaml'"]),
    ])
    ee_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ee_stabilization_controller", "--controller-manager", "/controller_manager",
            "--param-file", law_param_file,
            "--param-file", hydro_param_file,
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("controller"), "' == 'ee'"])),
    )
    # Base dynamic positioning (hull thrusters). Runs alongside any arm
    # controller; toggle with base_control:=false to compare with/without it.
    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["base_thruster_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(LaunchConfiguration("base_control")),
    )

    # Optional RViz view of the live robot state (driven by /joint_states + TF).
    rviz_config = PathJoinSubstitution([description_pkg, "rviz", "riptide.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz),
        output="screen",
    )

    # Optional native MuJoCo viewer window that mirrors the sim (read-only),
    # alongside RViz. Needs the `mujoco` pip package in the ROS Python.
    mujoco_viewer_node = Node(
        package="riptide_bringup",
        executable="mujoco_viewer.py",
        name="mujoco_viewer",
        condition=IfCondition(LaunchConfiguration("mujoco_viewer")),
        output="screen",
    )

    # Optional EE-pose teleop GUI (Tkinter): press-and-hold arrows publish the
    # desired Cartesian pose on /riptide/ee_target for the EE controller to track.
    ee_gui_node = Node(
        package="riptide_bringup",
        executable="ee_target_gui.py",
        name="ee_target_gui",
        condition=IfCondition(LaunchConfiguration("ee_gui")),
        output="screen",
    )

    # Floating base: MuJoCo broadcasts world->auv_base_link. On the mock path
    # (no physics) publish a static transform so RViz still has the base frame.
    mock_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mock_base_tf",
        arguments=["0", "0", "0.3", "0", "0", "0", "world", "auv_base_link"],
        condition=IfCondition(use_mock_hardware),
    )

    # Disturbance generator (steady_current / sinusoid / impulse / stochastic).
    # 'none' => off.
    disturbance_node = Node(
        package="riptide_disturbance",
        executable="disturbance_generator",
        name="disturbance_generator",
        parameters=[{
            "scenario": disturbance,
            "current_speed": ParameterValue(
                LaunchConfiguration("current_speed"), value_type=float),
            "amplitude": ParameterValue(
                LaunchConfiguration("disturbance_amplitude"), value_type=float),
        }],
        condition=UnlessCondition(
            PythonExpression(["'", disturbance, "' == 'none'"])),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Use mock_components hardware (Phase 1) instead of MuJoCo.",
        ),
        DeclareLaunchArgument(
            "water",
            default_value="true",
            description="Enable the fluid medium (drag, added mass, ocean current). "
                        "Set false for a dry sim (MuJoCo path only).",
        ),
        DeclareLaunchArgument(
            "fixed_base",
            default_value="false",
            description="Weld the AUV base to the world (non-floating, fixed-base "
                        "manipulator). MuJoCo path only; pair with base_control:=false.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Also open RViz with the Riptide view.",
        ),
        DeclareLaunchArgument(
            "mujoco_viewer",
            default_value="false",
            description="Also open MuJoCo's native viewer (mirrors the sim, "
                        "read-only). Needs `pip install mujoco` in the ROS Python.",
        ),
        DeclareLaunchArgument(
            "ee_gui",
            default_value="false",
            description="Open the EE-pose teleop GUI (press-and-hold arrows drive "
                        "the desired Cartesian pose). Needs controller:=ee.",
        ),
        DeclareLaunchArgument(
            "controller",
            default_value="pd",
            description="Controller to spawn: 'pd' (joint hold), 'ee' (task-space), 'none'.",
        ),
        DeclareLaunchArgument(
            "base_control",
            default_value="true",
            description="Run the hull-thruster base dynamic-positioning controller.",
        ),
        DeclareLaunchArgument(
            "control_law",
            default_value="impedance",
            description="EE control law when controller:=ee "
                        "('impedance' | 'lqr' | 'mpc' | 'template' — blank starting point).",
        ),
        DeclareLaunchArgument(
            "hydro",
            default_value="false",
            description="Compensate arm hydrodynamic drag ('true' | 'false'). Off by "
                        "default: cancelling drag is anti-damping and destabilizes.",
        ),
        DeclareLaunchArgument(
            "disturbance",
            default_value="none",
            description=(
                "Disturbance scenario: none | steady_current | sinusoid | impulse | "
                "stochastic (turbulent current: mean flow + Gauss-Markov turbulence)."
            ),
        ),
        DeclareLaunchArgument(
            "current_speed",
            default_value="1.0",
            description="Ocean-current flow speed [m/s] for the flow scenarios "
                        "(steady_current / sinusoid / stochastic). Applied via the "
                        "MuJoCo fluid model as 'wind', so it pushes the whole "
                        "structure (hull + every arm link), not just the base.",
        ),
        DeclareLaunchArgument(
            "disturbance_amplitude",
            default_value="80.0",
            description="Point-wrench force magnitude [N] for the 'impulse' scenario "
                        "(a localized hit on the base).",
        ),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        pd_controller_spawner,
        ee_controller_spawner,
        base_controller_spawner,
        mock_base_tf,
        disturbance_node,
        rviz_node,
        mujoco_viewer_node,
        ee_gui_node,
    ])
