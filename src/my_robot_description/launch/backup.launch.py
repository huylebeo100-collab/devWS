from pathlib import Path

import xacro

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    package_name = "my_robot_description"

    package_share_path = Path(get_package_share_directory(package_name))

    xacro_path = package_share_path / "urdf" / "robot.urdf.xacro"
    world_dir = package_share_path / "worlds"
    world_file_name = "warehouse.sdf"
    ##Them 2050
    # controller_config_path = package_share_path / "config" / "diff_drive_controller.yaml"


    urdf_path = Path("/tmp/my_robot_description_robot.urdf")

    robot_description_config = xacro.process_file(str(xacro_path))
    robot_description = robot_description_config.toxml()

    urdf_path.write_text(robot_description)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_pub_gui",
    #     output="screen",
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )
    
    gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value="/opt/ros/jazzy/lib:" + os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
    )

    gazebo_process = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            world_file_name,
        ],
        cwd=str(world_dir),
        output="screen",
    )

    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_four_wheel_sensor_bot",
        output="screen",
        arguments=[
            "-world", "obstacle_world",
            "-file", str(urdf_path),
            "-name", "four_wheel_sensor_bot",
            "-x", "0.0",
            "-y", "0",
            "-z", "0.03",   # base_link can o 0.022 tren mat dat
        ],
    )

    delayed_spawn_robot = TimerAction(
        period=3.0,
        actions=[
            spawn_robot_node,
        ],
    )

    # clock_bridge_node = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     name="clock_bridge",
    #     output="screen",
    #     arguments=[
    #         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    #     ],
    # )

    scan_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="scan_bridge",
        output="screen",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        parameters=[
            {"override_frame_id": "lidar_link"},
            {"use_sim_time": True},
        ]
    )
    bridge_node = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="ros_gz_bridge",
    output="screen",
    arguments=[
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
    ],
    parameters=[
        {"use_sim_time": True},
    ],
    )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name="joint_state_broadcaster_spawner",
    #     output="screen",
    #     arguments=[
    #         "joint_state_broadcaster",
    #     ],
    # )

    

    return LaunchDescription([
        robot_state_publisher_node,
        #joint_state_publisher_gui_node,
        
        gz_plugin_path,
        gazebo_process,
        delayed_spawn_robot,
        scan_bridge_node,
        bridge_node,
        rviz_node,
        
        
    ])
