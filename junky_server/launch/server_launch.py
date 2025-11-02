from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='junky_server',
            executable='web_server',
            name='web_server',
            output='screen'
        ),
        Node(
            package='junky_server',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
    ])
