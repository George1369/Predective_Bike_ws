from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gps_port = LaunchConfiguration('gps_port')
    radar_bus = LaunchConfiguration('radar_i2c_bus')
    radar_address = LaunchConfiguration('radar_i2c_address')
    radar_busy_gpio = LaunchConfiguration('radar_busy_gpio')
    return LaunchDescription([
        DeclareLaunchArgument('gps_port', default_value='/dev/serial0'),
        DeclareLaunchArgument('radar_i2c_bus', default_value='1'),
        DeclareLaunchArgument('radar_i2c_address', default_value='82'),
        DeclareLaunchArgument('radar_busy_gpio', default_value='-1'),
        Node(
            package='gps_node',
            executable='gps_driver_node',
            name='gps_driver_node',
            output='screen',
            parameters=[{'serial_port': gps_port, 'baud_rate': 9600}],
        ),
        Node(
            package='radar_node',
            executable='radar_driver_node',
            name='radar_driver_node',
            output='screen',
            parameters=[{
                'i2c_bus': ParameterValue(radar_bus, value_type=int),
                'i2c_address': ParameterValue(radar_address, value_type=int),
                'busy_gpio': ParameterValue(radar_busy_gpio, value_type=int),
            }],
        ),
        Node(
            package='display_node',
            executable='gui_display_node',
            name='gui_display_node',
            output='screen',
        ),
    ])
