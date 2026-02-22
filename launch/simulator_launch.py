from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robomaze',
            executable='simulator',
            name='robomaze',
            output='screen',
        ),
    ])
