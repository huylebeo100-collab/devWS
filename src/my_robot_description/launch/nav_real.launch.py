"""
Nav2 tren xe that - ROS 2 Humble.

Chay SAU real_robot.launch.py, va sau khi da co map:
    ros2 launch my_robot_description nav_real.launch.py \
        map:=/home/pi/maps/map_real.yaml

Chuoi lenh van toc (khong co collision_monitor o lan chay dau):
    controller_server --> /cmd_vel_nav
      --> velocity_smoother --> /cmd_vel --> base_node --> ESP32
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "my_robot_description"
    package_share = get_package_share_directory(package_name)

    default_params = os.path.join(package_share, "config", "nav2_params_real.yaml")
    default_map = os.path.join(package_share, "maps", "map_real.yaml")

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Nav2 params cho xe that",
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Duong dan file .yaml cua map da luu",
    )

    # use_sim_time duoc ep False o moi node ben duoi. Vi launch truyen
    # tham so SAU khi nap params_file nen no ghi de gia tri trong YAML,
    # khong con cach nao sot lai use_sim_time: true.
    sim_time = {"use_sim_time": False}

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, sim_time, {"yaml_filename": map_yaml}],
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file, sim_time],
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, sim_time],
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, sim_time],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, sim_time],
    )

    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, sim_time],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    smoother_server_node = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[params_file, sim_time],
    )

    waypoint_follower_node = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[params_file, sim_time],
    )

    # Vao: cmd_vel_nav.  Ra: cmd_vel (thang toi base_node).
    # Khi bat lai collision_monitor thi bo remap thu hai di, de
    # velocity_smoother phat ra cmd_vel_smoothed nhu mac dinh.
    velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[params_file, sim_time],
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),
            ("cmd_vel_smoothed", "cmd_vel"),
        ],
    )

    lifecycle_manager_localization_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    lifecycle_manager_navigation_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": [
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
            ],
        }],
    )

    return LaunchDescription([
        declare_params_file,
        declare_map,

        map_server_node,
        amcl_node,

        planner_server_node,
        controller_server_node,
        bt_navigator_node,
        behavior_server_node,
        smoother_server_node,
        waypoint_follower_node,
        velocity_smoother_node,

        lifecycle_manager_localization_node,
        lifecycle_manager_navigation_node,
    ])