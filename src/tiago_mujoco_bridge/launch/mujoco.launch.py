from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="pro",
        description="Robot model to simulate: pro or dual",
    )

    robot_model = LaunchConfiguration(
        "robot_model"
    )

    mujoco_node = Node(
        package="tiago_mujoco_bridge",
        executable="mujoco_sim_node",
        name="mujoco_sim_node",
        output="screen",
        parameters=[
            {
                "robot_model": robot_model,
            }
        ],
    )

    return LaunchDescription(
        [
            robot_model_arg,
            mujoco_node,
        ]
    )