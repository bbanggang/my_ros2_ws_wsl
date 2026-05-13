from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_ros_rclpy_pkg',
            executable='hello_pub',
            name='helloworld_publisher',
            output='screen',
        ),
        Node(
            package='my_first_ros_rclpy_pkg',
            executable='hello_sub',
            name='helloworld_subscriber',
            output='screen',
        ),
    ])
