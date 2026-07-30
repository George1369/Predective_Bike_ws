from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_node',
            executable='gps_driver_node',
            name='gps_driver_node',
            output='screen',
        ),
        Node(
            package='radar_node',
            executable='radar_driver_node',
            name='radar_driver_node',
            output='screen',
        ),
        Node(
            package='display_node',
            executable='gui_display_node',
            name='gui_display_node',
            output='screen',
        ),
    ])
