#!/usr/bin/env python3
"""
mock_imu_publisher.py

Publishes synthetic sensor_msgs/Imu data on /imu/data at the same rate
and topic the real imu_driver_node will use, so downstream nodes
(sensor_fusion_node, risk_assessment_node) can be developed and tested
on a PC before real IMU hardware is wired up.

Simulates gentle riding motion (small oscillating tilt + noise) and
occasionally fires a synthetic fall event so /imu/fall_event handling
can be tested too.
"""

import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

PUBLISH_RATE_HZ = 50.0
FALL_EVENT_PERIOD_S = 25.0  # inject a fake fall roughly this often, for testing


class MockImuPublisher(Node):
    def __init__(self):
        super().__init__('mock_imu_publisher')

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.fall_pub = self.create_publisher(Bool, '/imu/fall_event', 10)

        self._t = 0.0
        self._dt = 1.0 / PUBLISH_RATE_HZ
        self._since_last_fall = 0.0

        self.timer = self.create_timer(self._dt, self._on_timer)
        self.get_logger().info('mock_imu_publisher started (synthetic data)')

    def _on_timer(self):
        self._t += self._dt
        self._since_last_fall += self._dt

        # Gentle simulated sway (as if riding over slightly uneven pavement)
        sway = 0.15 * math.sin(2 * math.pi * 0.5 * self._t)
        noise = random.uniform(-0.03, 0.03)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.linear_acceleration.x = 0.2 * math.sin(2 * math.pi * 1.2 * self._t) + noise
        msg.linear_acceleration.y = sway + noise
        msg.linear_acceleration.z = 9.81 + noise

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.05 * math.sin(2 * math.pi * 0.2 * self._t)

        # Small constant tilt as a placeholder quaternion (identity-ish)
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        self.imu_pub.publish(msg)

        if self._since_last_fall >= FALL_EVENT_PERIOD_S:
            self._since_last_fall = 0.0
            self.get_logger().warn('Injecting synthetic fall event for testing')
            self.fall_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = MockImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
