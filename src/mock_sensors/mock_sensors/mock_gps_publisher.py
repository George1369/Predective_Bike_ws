#!/usr/bin/env python3
"""
mock_gps_publisher.py

Publishes synthetic sensor_msgs/NavSatFix + speed data on /gps/fix and
/gps/speed, simulating a bike riding in a slow loop, so downstream nodes
can be developed before real GPS hardware is wired up.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32

PUBLISH_RATE_HZ = 1.0

# Simulated loop parameters (arbitrary starting point -- adjust to your area)
CENTER_LAT = 37.7749
CENTER_LON = -122.4194
RADIUS_DEG = 0.001       # roughly ~100m loop
ANGULAR_SPEED_RAD_S = 0.05
SIMULATED_SPEED_MPS = 5.5  # ~20 km/h


class MockGpsPublisher(Node):
    def __init__(self):
        super().__init__('mock_gps_publisher')

        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.speed_pub = self.create_publisher(Float32, '/gps/speed', 10)

        self._t = 0.0
        self._dt = 1.0 / PUBLISH_RATE_HZ

        self.timer = self.create_timer(self._dt, self._on_timer)
        self.get_logger().info('mock_gps_publisher started (synthetic loop route)')

    def _on_timer(self):
        self._t += self._dt
        angle = ANGULAR_SPEED_RAD_S * self._t

        lat = CENTER_LAT + RADIUS_DEG * math.sin(angle)
        lon = CENTER_LON + RADIUS_DEG * math.cos(angle)

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 15.0

        self.fix_pub.publish(msg)
        self.speed_pub.publish(Float32(data=SIMULATED_SPEED_MPS))


def main(args=None):
    rclpy.init(args=args)
    node = MockGpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
