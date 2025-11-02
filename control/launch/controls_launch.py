from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            executable='manual_control_node',
            name='manual_control_node',
            output='screen'
        ),
    ])
