"""
Bringup xe that - ROS 2 Humble tren Raspberry Pi 4.
Thay the hoan toan backup.launch.py (ban Gazebo).

Chay dau tien, truoc SLAM va truoc Nav2:
    ros2 launch my_robot_description real_robot.launch.py
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "my_robot_description"
    package_share = get_package_share_directory(package_name)

    xacro_path = os.path.join(package_share, "urdf", "robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_path).toxml()

    esp32_port = LaunchConfiguration("esp32_port")
    lidar_port = LaunchConfiguration("lidar_port")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_esp32_port = DeclareLaunchArgument(
        "esp32_port",
        default_value="/dev/ttyS0",
        description="Cong UART noi voi ESP32-S3",
    )

    declare_lidar_port = DeclareLaunchArgument(
        "lidar_port",
        default_value="/dev/ttyUSB0",
        description="Cong USB cua RPLIDAR A2M8",
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Bat RViz. Tren Pi nen de false, chay RViz o may khac.",
    )

    # ------------------------------------------------------------------
    #  robot_state_publisher: nguon duy nhat cua cac TF tinh trong URDF
    #  (base_link -> base_footprint, -> lidar_link, -> imu_link, banh xe)
    # ------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": False},
        ],
    )

    # ------------------------------------------------------------------
    #  base_node: cau noi UART <-> ROS.
    #    subscribe  /cmd_vel
    #    publish    /odom, /joint_states, TF odom -> base_link
    #
    #  wheel_diameter o day la duong kinh HIEU DUNG da hieu chuan (0.10896),
    #  KHONG phai duong kinh vat ly 0.095 trong URDF. Hai so nay phuc vu
    #  hai muc dich khac nhau, de lech nhau la dung.
    # ------------------------------------------------------------------
    base_node = Node(
        package="esp32_base_driver",
        executable="base_node",
        name="base_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "port": esp32_port,
            "baudrate": 115200,
            "wheel_diameter": 0.095,
            "wheel_separation": 0.389,
            "ticks_per_rev": 998.4,
            "publish_tf": True,
        }],
    )

    # ------------------------------------------------------------------
    #  RPLIDAR A2M8.
    #  frame_id PHAI trung ten link trong sensor.xacro = lidar_link.
    #  A2M8 dung baudrate 115200 (A3/S1 moi dung 256000).
    # ------------------------------------------------------------------
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "serial_port": lidar_port,
            "serial_baudrate": 115200,
            "frame_id": "lidar_link",
            "inverted": False,
            "angle_compensate": True,
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=None,
        parameters=[{"use_sim_time": False}],
    )

    nodes = [
        declare_esp32_port,
        declare_lidar_port,
        declare_use_rviz,
        robot_state_publisher_node,
        base_node,
        lidar_node,
    ]

    return LaunchDescription(nodes)