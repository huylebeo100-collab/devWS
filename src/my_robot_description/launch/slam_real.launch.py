"""
SLAM tren xe that - ROS 2 Humble.

Chay SAU real_robot.launch.py:
    ros2 launch my_robot_description slam_real.launch.py

Khac ban cu (slam.launch.py):
  - use_sim_time: "true" -> "false"   <-- day la loi hay quen nhat
  - doc slam_real.yaml thay vi slam.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_name = "my_robot_description"
    package_share = get_package_share_directory(package_name)

    slam_params_file = os.path.join(package_share, "config", "slam_real.yaml")

    slam_launch_file = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py",
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": slam_params_file,
        }.items(),
    )

    return LaunchDescription([slam_toolbox])