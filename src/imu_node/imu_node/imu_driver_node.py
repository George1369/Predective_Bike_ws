#!/usr/bin/env python3
"""
imu_driver_node.py

Phase 1 driver node for the 9-axis IMU (e.g. BNO055 / ICM-20948 over I2C).

Publishes:
    /imu/data        (sensor_msgs/Imu)       - orientation, angular velocity, linear accel
    /imu/fall_event   (std_msgs/Bool)          - True on detected fall/impact

TODO (hardware bring-up):
    - Replace the `_read_hardware()` stub with real I2C/SPI reads
      (e.g. via smbus2 for BNO055, or an existing ROS 2 IMU driver).
    - Tune FALL_ACCEL_THRESHOLD_G and STILLNESS_WINDOW_S against real
      fall/impact test data before trusting this for anything safety-critical.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

# --- Tunable thresholds (placeholder values -- validate empirically) ---
PUBLISH_RATE_HZ = 50.0
FALL_ACCEL_THRESHOLD_G = 2.5     # spike above this magnitude may indicate impact
FREEFALL_ACCEL_THRESHOLD_G = 0.3  # near-zero magnitude may indicate free-fall
STILLNESS_WINDOW_S = 1.5          # post-event stillness confirms a fall vs. a bump


class ImuDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver_node')

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.fall_pub = self.create_publisher(Bool, '/imu/fall_event', 10)

        self._post_event_timer = None
        self._pending_fall_check = False

        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._on_timer)
        self.get_logger().info('imu_driver_node started (stub hardware read)')

    def _read_hardware(self):
        """
        Placeholder for real sensor read.
        Replace with actual I2C/SPI transaction to the IMU.
        Must return (accel_xyz_g, gyro_xyz_dps, quat_wxyz).
        """
        accel_xyz_g = (0.0, 0.0, 1.0)   # resting on a flat surface
        gyro_xyz_dps = (0.0, 0.0, 0.0)
        quat_wxyz = (1.0, 0.0, 0.0, 0.0)
        return accel_xyz_g, gyro_xyz_dps, quat_wxyz

    def _on_timer(self):
        accel_g, gyro_dps, quat = self._read_hardware()

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.linear_acceleration.x = accel_g[0] * 9.81
        msg.linear_acceleration.y = accel_g[1] * 9.81
        msg.linear_acceleration.z = accel_g[2] * 9.81

        msg.angular_velocity.x = math.radians(gyro_dps[0])
        msg.angular_velocity.y = math.radians(gyro_dps[1])
        msg.angular_velocity.z = math.radians(gyro_dps[2])

        msg.orientation.w = quat[0]
        msg.orientation.x = quat[1]
        msg.orientation.y = quat[2]
        msg.orientation.z = quat[3]

        self.imu_pub.publish(msg)

        self._check_fall(accel_g)

    def _check_fall(self, accel_g):
        magnitude_g = math.sqrt(sum(a * a for a in accel_g))
        if magnitude_g > FALL_ACCEL_THRESHOLD_G or magnitude_g < FREEFALL_ACCEL_THRESHOLD_G:
            # In a real implementation: start a STILLNESS_WINDOW_S timer,
            # then confirm the fall only if motion stays low afterward
            # (avoids false positives from potholes/curbs).
            self.get_logger().warn(
                f'Possible fall/impact event: |a| = {magnitude_g:.2f} g'
            )
            self.fall_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = ImuDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
