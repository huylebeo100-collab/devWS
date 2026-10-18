import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("esp32_base_driver"), "config", "base_params.yaml"
    )
    return LaunchDescription([
        Node(
            package="esp32_base_driver",
            executable="base_node",
            name="esp32_base_driver",
            output="screen",
            parameters=[params],
        ),
    ])
