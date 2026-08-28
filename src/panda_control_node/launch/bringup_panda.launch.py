from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Unlike bringup.launch.py's tiago_dual_cartesio_config lookup, this URDF is vendored
    # directly in this repo (robots/panda/urdf/panda.urdf) - no external ROS package needed.
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='/home/forest_ws/robots/panda/urdf/panda.urdf',
        description="Path to the Panda URDF (kinematics/collision only - see that file's "
                    "header comment on why visual meshes aren't resolved). Shared between "
                    "the OpenSoT ghost's robot_state_publisher and panda_opensot_node "
                    "itself so they can't drift out of sync.")

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2 showing the OpenSoT ghost.')

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('panda_control_node'), 'rviz', 'panda_opensot.rviz'])

    urdf_path = LaunchConfiguration('urdf_path')
    robot_description = {'robot_description': ParameterValue(Command(['cat ', urdf_path]), value_type=str)}

    # Publishes the OpenSoT ghost's TF tree (opensot/link0 -> ... -> opensot/ee_panda) from
    # /opensot/joint_states, frame_prefix='opensot/' - same pattern as bringup.launch.py's
    # opensot_robot_state_publisher. No separate opensot/world static-transform bridge is
    # needed here the way TIAGo's bringup has one: TIAGo has a mobile base that needs
    # aligning to a real localization frame; Panda is fixed-base, and the URDF's own
    # world -> link0 fixed joint already roots the whole ghost tree by itself.
    node_opensot_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opensot_robot_state_publisher',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[robot_description, {'publish_frequency': 50.0, 'frame_prefix': 'opensot/'}],
        remappings=[('/joint_states', '/opensot/joint_states')],
    )

    node_panda_opensot = Node(
        package='panda_control_node',
        executable='panda_opensot_node',
        name='panda_opensot_control',
        output='screen',
        parameters=[{'urdf_path': urdf_path}],
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # Small delay so opensot_robot_state_publisher's TF is already up before the OpenSoT
    # node starts publishing joint states that depend on it existing - same pattern
    # bringup.launch.py uses for its own solver/cartesian_interface nodes.
    delayed_nodes = TimerAction(period=2.0, actions=[node_panda_opensot])

    return LaunchDescription([
        urdf_path_arg,
        use_rviz_arg,
        node_opensot_rsp,
        node_rviz,
        delayed_nodes,
    ])
