from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_node',
            executable='gps_driver_node',
            name='gps_driver_node',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyAMA0', 'baud_rate': 9600}],
        ),
        Node(
            package='radar_node',
            executable='radar_driver_node',
            name='radar_driver_node',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'baud_rate': 921600}],
        ),
    ])
