import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("amr_mission"), "config", "mission_params.yaml")
    params_file = LaunchConfiguration("params_file")
    dry_run = LaunchConfiguration("dry_run")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="true = khong gui goal Nav2, xe khong chay"),
        Node(
            package="amr_mission",
            executable="mission_manager",
            name="mission_manager",
            output="screen",
            parameters=[params_file,
                        {"dry_run": ParameterValue(dry_run, value_type=bool)}],
        ),
    ])
