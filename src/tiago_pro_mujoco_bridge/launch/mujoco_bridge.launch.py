from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mujoco_xml_path_arg = DeclareLaunchArgument(
        'mujoco_xml_path',
        default_value='/home/forest_ws/robots/pal_tiago_pro/xmls/scene_tiago_pro.xml',
        description='Path to the tiago-pro-mujoco scene XML.'
    )
    viewer_arg = DeclareLaunchArgument(
        'viewer',
        default_value='true',
        description='Whether to open the MuJoCo passive viewer window.'
    )

    node_mujoco_bridge = Node(
        package='tiago_pro_mujoco_bridge',
        executable='mujoco_bridge_node',
        name='tiago_pro_mujoco_bridge',
        output='screen',
        parameters=[{
            'mujoco_xml_path': LaunchConfiguration('mujoco_xml_path'),
            'viewer': LaunchConfiguration('viewer'),
        }]
    )

    return LaunchDescription([
        mujoco_xml_path_arg,
        viewer_arg,
        node_mujoco_bridge,
    ])
