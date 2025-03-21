import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description.

    Returns:
        LaunchDescription -- The launch description.
    """
    debug = LaunchConfiguration("debug")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "debug", default_value="False", description="Enable debug mode"
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    get_package_share_directory("rosbag2_utils"),
                    "config",
                    "ros_params.yaml",
                ),
                description="Path to the ROS parameters file",
            ),
            Node(
                package="rosbag2_utils",
                namespace="",  # Is also the namespace for loading the params
                executable="rosbag_to_images_node",
                name="rosbag_to_images_node",
                parameters=[
                    {"debug": debug},
                    params_file,
                ],
            ),
        ]
    )
