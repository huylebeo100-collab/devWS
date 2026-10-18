"""Launch kiem tra: base_node + cmd_vel_test. Khong co SLAM/Nav2."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("esp32_base_driver"), "config", "base_params.yaml"
    )
    phase = LaunchConfiguration("phase")
    lifted = LaunchConfiguration("lifted")

    return LaunchDescription([
        DeclareLaunchArgument("phase", default_value="all",
                              description="all | forward | backward | spin_left | "
                                          "spin_right | arc_left | arc_right"),
        DeclareLaunchArgument("lifted", default_value="true",
                              description="true khi dang ke banh len cao"),
        Node(
            package="esp32_base_driver",
            executable="base_node",
            name="esp32_base_driver",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="esp32_base_driver",
            executable="cmd_vel_test",
            name="cmd_vel_test",
            output="screen",
            parameters=[{
                "phase": phase,
                "lifted": lifted,
                "wheel_separation": 0.390,
                "linear": 0.15,
                "angular": 0.8,
                "duration": 3.0,
            }],
        ),
    ])
