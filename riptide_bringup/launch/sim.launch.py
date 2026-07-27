"""Bring up the Riptide sim: robot_state_publisher + ros2_control + broadcasters.

Phase 1 uses mock hardware (mock_components/GenericSystem). Swap to MuJoCo in
Phase 2 with `use_mock_hardware:=false` — no other launch change needed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    rviz = LaunchConfiguration("rviz")

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

    # Baseline torque controller (holds the arm against gravity). Disable with
    # controller:=none to watch the arm free-fall instead.
    pd_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_pd_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("controller"), "' == 'pd'"])),
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
            description="Which controller to spawn: 'pd' (JointPdController) or 'none'.",
        ),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        pd_controller_spawner,
        rviz_node,
    ])
