from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'mujoco_xml_path',
            default_value='/home/forest_ws/robots/panda/xmls/scene_panda.xml',
            description='Path to the Panda scene XML.'),
        DeclareLaunchArgument(
            'viewer',
            default_value='true',
            description='Whether to open the MuJoCo passive viewer window.'),
        DeclareLaunchArgument(
            'fps',
            default_value='90.0',
            description='Physics/render loop rate (Hz).'),
        DeclareLaunchArgument(
            'episode_log_fps',
            default_value=LaunchConfiguration('fps'),
            description='How often a step is appended to the episode log (Hz) - defaults '
                         'to tracking fps, same as mujoco_bridge.launch.py.'),
        DeclareLaunchArgument(
            'command_topic',
            default_value='/opensot/joint_states',
            description='Topic the sim reads commanded joint positions from.'),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/joint_states',
            description='Topic the sim publishes its own joint state on.'),
        DeclareLaunchArgument(
            'gripper_speed',
            default_value='400.0',
            description='Gripper open/close ramp speed - ctrl units/sec (0-255 range, not '
                         'radians, since the gripper is one tendon-coupled actuator here).'),
        DeclareLaunchArgument(
            'target_object_joint',
            default_value='cube_freejoint',
            description='MuJoCo freejoint name of the object tracked/randomized for episodes.'),
        DeclareLaunchArgument(
            'episode_log_dir',
            default_value='/tmp/panda_episodes',
            description='Directory dataset files are saved into.'),
        DeclareLaunchArgument(
            'dataset_name',
            default_value='dataset',
            description='Base filename (without .h5) for this collection run\'s HDF5 file.'),
        DeclareLaunchArgument(
            'save_failed_episodes',
            default_value='false',
            description='Keep failed episodes in the log too, instead of discarding them.'),
        DeclareLaunchArgument(
            'object_x_range',
            default_value='[0.50, 0.65]',
            description='Table-frame x range (meters) the object is respawned into - same '
                         'as the TIAGo scene, physically identical table.'),
        DeclareLaunchArgument(
            'object_y_range',
            default_value='[-0.20, -0.10]',
            description='Table-frame y range (meters) the object is respawned into.'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='opensot/link0',
            description='Frame the published target-object pose is expressed in.'),
    ]

    node_panda_sim = Node(
        package='panda_mujoco_bridge',
        executable='panda_sim_node',
        name='panda_sim_node',
        output='screen',
        parameters=[{
            'mujoco_xml_path': LaunchConfiguration('mujoco_xml_path'),
            'viewer': LaunchConfiguration('viewer'),
            'fps': ParameterValue(LaunchConfiguration('fps'), value_type=float),
            'episode_log_fps': ParameterValue(LaunchConfiguration('episode_log_fps'), value_type=float),
            'command_topic': LaunchConfiguration('command_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'gripper_speed': ParameterValue(LaunchConfiguration('gripper_speed'), value_type=float),
            'target_object_joint': LaunchConfiguration('target_object_joint'),
            'episode_log_path': [LaunchConfiguration('episode_log_dir'), '/', LaunchConfiguration('dataset_name'), '.h5'],
            'save_failed_episodes': ParameterValue(LaunchConfiguration('save_failed_episodes'), value_type=bool),
            'object_x_range': ParameterValue(LaunchConfiguration('object_x_range'), value_type=List[float]),
            'object_y_range': ParameterValue(LaunchConfiguration('object_y_range'), value_type=List[float]),
            'base_frame': LaunchConfiguration('base_frame'),
        }]
    )

    # Reused directly from tiago_pro_mujoco_bridge, unmodified - it's already robot-agnostic
    # (only coupled to topic/service names like /mujoco_bridge/sim/*, /opensot/reset_complete,
    # not to joint counts or arm count - see project memory panda_wbc_ablation_goal), and
    # panda_sim_node.py above implements the exact same service/topic contract it expects.
    node_episode_orchestrator = Node(
        package='tiago_pro_mujoco_bridge',
        executable='episode_orchestrator_node',
        name='episode_orchestrator_node',
        output='screen',
    )

    return LaunchDescription(args + [
        node_panda_sim,
        node_episode_orchestrator,
    ])
