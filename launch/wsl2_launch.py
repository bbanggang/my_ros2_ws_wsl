from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros2',
            executable='sub',
            name='camera_sub',
            output='screen',
        ),
        Node(
            package='dxl_wsl',
            executable='pub',
            name='dxl_pub',
            output='screen',
            prefix='xterm -e',
        ),
    ])
