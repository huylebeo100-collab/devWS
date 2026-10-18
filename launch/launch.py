from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mypkg',
            namespace='vd1',
            executable='talker',
            name='sim1',
            output = 'screen'
        ),
        Node(
            package='mypkg',
            namespace='vd1',
            executable='listener',
            name='sim2',
            output = 'screen'
        ),
        
    ])
