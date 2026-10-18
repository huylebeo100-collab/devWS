import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ----------------------------------------------------------------
    #  Package paths
    # ----------------------------------------------------------------
    pkg_name     = 'my_robot_description'       # <-- tên package của bạn
    pkg_share    = get_package_share_directory(pkg_name)
    nav2_bringup = get_package_share_directory('nav2_bringup')

    default_map    = os.path.join(pkg_share, 'maps', 'map_save.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params_sim.yaml')

    # ----------------------------------------------------------------
    #  Launch Arguments
    # ----------------------------------------------------------------
    declare_map_arg = DeclareLaunchArgument(
        name='map',
        default_value=default_map,
        description='Đường dẫn tới file .yaml của map (SLAM output)'
    )
    declare_params_arg = DeclareLaunchArgument(
        name='params_file',
        default_value=default_params,
        description='Đường dẫn tới nav2_params_sim.yaml'
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Dùng đồng hồ Gazebo'
    )
    declare_autostart_arg = DeclareLaunchArgument(
        name='autostart',
        default_value='true',
        description='Tự động start lifecycle nodes'
    )

    map_yaml     = LaunchConfiguration('map')
    params_file  = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')

    # ----------------------------------------------------------------
    #  Map Server node
    # ----------------------------------------------------------------
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_yaml},  # override đường dẫn map
            {'use_sim_time': use_sim_time},
        ]
    )

    # ----------------------------------------------------------------
    #  AMCL node
    # ----------------------------------------------------------------
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ]
    )

    # ----------------------------------------------------------------
    #  Lifecycle Manager 1 — chỉ quản lý localization
    #  (tách riêng để không xung đột với nav2_bringup)
    # ----------------------------------------------------------------
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'map_server',
                'amcl',
            ]
        }]
    )

    # ----------------------------------------------------------------
    #  Nav2 Bringup — navigation stack
    #  (tự có lifecycle_manager_navigation bên trong,
    #   quản lý: controller, planner, bt, behaviors, smoother, waypoint)
    # ----------------------------------------------------------------
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart':    autostart,
            'params_file':  params_file,
            'use_composition': 'False',
        }.items()
    )

    # ----------------------------------------------------------------
    #  Assemble
    # ----------------------------------------------------------------
    return LaunchDescription([
        declare_map_arg,
        declare_params_arg,
        declare_use_sim_time_arg,
        declare_autostart_arg,

        map_server_node,
        amcl_node,
        lifecycle_manager_localization,  # lifecycle cho map + amcl
        nav2_bringup_launch,             # nav2 tự lo phần còn lại
    ])