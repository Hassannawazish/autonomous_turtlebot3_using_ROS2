from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'autonomous_bot',
            executable = 'autonomous_bot_executable',
            output = 'screen'),
    ])
