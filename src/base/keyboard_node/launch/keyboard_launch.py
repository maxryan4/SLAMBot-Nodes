from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='keyboard_node',
            executable='keyboard_node',
            name='keyboard_node',
            output='screen'
        )
    ])