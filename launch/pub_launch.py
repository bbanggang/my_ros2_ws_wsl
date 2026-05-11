from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='rclpy_pkg1',
            executable='twist_pub',
            name='twist_pub',
            output='screen',
        ),
    ])
