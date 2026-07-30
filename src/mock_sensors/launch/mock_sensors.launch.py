"""
Launch both mock sensor publishers together, so you can start testing
sensor_fusion_node / risk_assessment_node against two simultaneous
synthetic sensor streams without any hardware attached.

Run with:
    ros2 launch mock_sensors mock_sensors.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mock_sensors',
            executable='mock_imu_publisher',
            name='mock_imu_publisher',
            output='screen',
        ),
        Node(
            package='mock_sensors',
            executable='mock_gps_publisher',
            name='mock_gps_publisher',
            output='screen',
        ),
    ])
