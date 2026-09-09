from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Robot model argument to select between dual and pro
    robot_model_arg = DeclareLaunchArgument(
        "robot_model", default_value="pro", description="Robot model to use (dual or pro)"
    )

    # State conditions (CLEANED)
    robot_model = LaunchConfiguration("robot_model")
    is_pro = LaunchConfigurationEquals("robot_model", "pro")
    is_dual = LaunchConfigurationNotEquals("robot_model", "pro")

    # ROS namespace for the app nodes (both solvers + the cartesian interface).
    # Defaults to 'tiago_pro' / 'tiago' from robot_model so two stacks can share a
    # DDS graph without colliding. The robot_state_publishers, RViz, the static TF
    # and the hardware-boundary topics (/opensot/joint_states,
    # /opensot/base_velocity_command) stay GLOBAL -- they are the contract with the
    # robot bridge and keep /tf a single tree.
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=PythonExpression(
            ["'tiago_pro' if '", robot_model, "' == 'pro' else 'tiago'"]
        ),
        description="ROS namespace for the WBC app nodes (solver + cartesian interface)",
    )
    namespace = LaunchConfiguration("namespace")

    # Setup Paths
    control_pkg_share = FindPackageShare("tiago_control_node")
    urdf_pkg_share = FindPackageShare("tiago_dual_cartesio_config")

    rviz_config_file = PathJoinSubstitution([control_pkg_share, "rviz", "tiago_dual.rviz"])
    config_path = PathJoinSubstitution([control_pkg_share, "config", "params.yaml"])

    # Dynamic URDF Selection
    urdf_file_name = PythonExpression(
        [
            "'tiago_pro_capsules.urdf' if '",
            robot_model,
            "' == 'pro' else 'tiago_dual_capsules.urdf'",
        ]
    )

    urdf_file_path = PathJoinSubstitution([urdf_pkg_share, "capsules", "urdf", urdf_file_name])

    # Load URDF content once as a command (CLEANED: Forced string typing for Humble safety)
    doc = ParameterValue(Command(["cat ", urdf_file_path]), value_type=str)

    # This dictionary can be passed to any node's 'parameters' list
    robot_description = {"robot_description": doc}

    # Nodes

    # Real Robot State Publisher
    node_real_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="real_robot_state_publisher",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[robot_description, {"publish_frequency": 50.0}],
        remappings=[("/joint_states", "/joint_states")],
    )

    # OpenSoT Robot State Publisher
    node_opensot_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="opensot_robot_state_publisher",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[robot_description, {"publish_frequency": 50.0, "frame_prefix": "opensot/"}],
        remappings=[
            ("/joint_states", "/opensot/joint_states"),
            ("robot_description", "opensot_robot_description"),
        ],
    )

    # Static TFs (CLEANED: Removed deprecated old-style arguments)
    node_tf_bridge_opensot = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="opensot_world_connector",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "base_footprint",
            "--child-frame-id",
            "opensot/world",
        ],
    )

    # RViz
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # OpenSoT Solver Node - Dual
    node_solver_dual = Node(
        package="tiago_control_node",
        executable="tiago_opensot_node",
        name="tiago_opensot_control",
        output="screen",
        parameters=[config_path],
        condition=is_dual,
    )

    # OpenSoT Solver Node - Pro
    node_solver_pro = Node(
        package="tiago_control_node",
        executable="tiago_pro_opensot_node",
        name="tiago_pro_opensot_control",
        output="screen",
        parameters=[config_path],
        condition=is_pro,
    )

    # Cartesian Interface Node
    node_cartesian_interface = Node(
        package="tiago_control_node",
        executable="cartesian_interface_node",
        name="cartesian_interface_node",
        output="screen",
        parameters=[config_path, robot_description, {"robot_model": robot_model}],
    )

    # App nodes go under `namespace`; the RSPs / RViz / static TF above stay global.
    delayed_nodes = TimerAction(
        period=2.0,
        actions=[
            GroupAction(
                [
                    PushRosNamespace(namespace),
                    node_solver_dual,
                    node_solver_pro,
                    node_cartesian_interface,
                ]
            )
        ],
    )

    # Final Launch Description
    return LaunchDescription(
        [
            robot_model_arg,
            namespace_arg,
            node_tf_bridge_opensot,
            node_real_rsp,
            node_opensot_rsp,
            node_rviz,
            delayed_nodes,
        ]
    )
