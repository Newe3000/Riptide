"""Bring up the Riptide sim: robot_state_publisher + ros2_control + broadcasters.

Phase 1 uses mock hardware (mock_components/GenericSystem). Swap to MuJoCo in
Phase 2 with `use_mock_hardware:=false` — no other launch change needed.
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
    """Kill leftover sim processes from a previous run before starting a new one.

    Each ``ros2_control_node`` embeds its OWN MuJoCo world. If a previous launch
    didn't shut down cleanly (Ctrl-C doesn't always reap it, closing the
    terminal orphans it), a second one keeps stepping physics and publishing a
    diverging ``/joint_states`` + ``/tf`` stream. RViz then jumps between the two
    worlds every message — the robot and base appear to "teleport". Reaping the
    stale processes here guarantees each launch starts from a single clean sim.
    """
    for name in (
        "ros2_control_node",        # the embedded-MuJoCo culprit
        "robot_state_publisher",    # stale latched /robot_description
        "mock_base_tf",             # stale mock world->auv_base_link static TF
        "rviz2",
        "disturbance_generator",
    ):
        subprocess.run(
            ["pkill", "-9", "-f", name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
        )


def generate_launch_description():
    # Pre-flight: never let a previous run's sim linger under this one.
    _reap_stale_sim_processes()

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    rviz = LaunchConfiguration("rviz")
    disturbance = LaunchConfiguration("disturbance")

    description_pkg = FindPackageShare("riptide_description")

    # Expand the top-level xacro into a robot_description string.
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([description_pkg, "urdf", "riptide.urdf.xacro"]),
            " use_mock_hardware:=", use_mock_hardware,
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
    # Task-space EE stabilization (Pinocchio + impedance) — rejects base motion.
    ee_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ee_stabilization_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("controller"), "' == 'ee'"])),
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

    # Floating base: MuJoCo broadcasts world->auv_base_link. On the mock path
    # (no physics) publish a static transform so RViz still has the base frame.
    mock_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mock_base_tf",
        arguments=["0", "0", "0.3", "0", "0", "0", "world", "auv_base_link"],
        condition=IfCondition(use_mock_hardware),
    )

    # Disturbance generator (steady_current / sinusoid / impulse). 'none' => off.
    disturbance_node = Node(
        package="riptide_disturbance",
        executable="disturbance_generator",
        name="disturbance_generator",
        parameters=[{"scenario": disturbance}],
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
            "rviz",
            default_value="false",
            description="Also open RViz with the Riptide view.",
        ),
        DeclareLaunchArgument(
            "controller",
            default_value="pd",
            description="Controller to spawn: 'pd' (joint hold), 'ee' (task-space), 'none'.",
        ),
        DeclareLaunchArgument(
            "disturbance",
            default_value="none",
            description="Disturbance scenario: none | steady_current | sinusoid | impulse.",
        ),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        pd_controller_spawner,
        ee_controller_spawner,
        mock_base_tf,
        disturbance_node,
        rviz_node,
    ])
