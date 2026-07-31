from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_node',
            executable='gps_driver_node',
            name='gps_driver_node',
            output='screen',
            parameters=[{'serial_port': '/dev/serial0', 'baud_rate': 9600}],
        ),
        Node(
            package='radar_node',
            executable='radar_driver_node',
            name='radar_driver_node',
            output='screen',
            parameters=[{'i2c_bus': 1, 'i2c_address': 0x52}],
        ),
    ])
