#!/usr/bin/env python3
"""ROS 2 driver for the Waveshare A121 Range Sensor I2C distance firmware.

The A121 must be flashed with the ``i2c_distance_detector`` firmware.  It is
connected to Raspberry Pi I2C-1 (GPIO2/SDA and GPIO3/SCL) at address 0x52 by
default.  The optional BUSY line can be connected to a free GPIO; the detector
status register remains the authoritative completion/error signal.
"""

import csv
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:  # Allows parser/register tests on non-Pi development PCs.
    SMBus = None
    i2c_msg = None

try:
    from gpiozero import Button
except ImportError:  # BUSY is optional; do not make GPIO a runtime requirement.
    Button = None

try:
    from bike_msgs.msg import RangeSensorState
except ModuleNotFoundError:  # pragma: no cover - direct source-tree unit tests
    class RangeSensorState:  # type: ignore[override]
        def __init__(self):
            self.header = type('Header', (), {'stamp': None, 'frame_id': ''})()
            self.distance_m = 0.0
            self.presence = False
            self.motion = False
            self.signal_strength = 0.0
            self.confidence = 0.0


REG_DETECTOR_STATUS = 0x0003
REG_DISTANCE_RESULT = 0x0010
REG_PEAK0_DISTANCE = 0x0011
REG_PEAK0_STRENGTH = 0x001B
REG_START = 0x0040
REG_END = 0x0041
REG_MAX_STEP_LENGTH = 0x0042
REG_CLOSE_RANGE_LEAKAGE_CANCEL = 0x0043
REG_SIGNAL_QUALITY = 0x0044
REG_MAX_PROFILE = 0x0045
REG_THRESHOLD_METHOD = 0x0046
REG_PEAK_SORTING = 0x0047
REG_NUM_FRAMES_RECORDED_THRESHOLD = 0x0048
REG_FIXED_AMPLITUDE_THRESHOLD_VALUE = 0x0049
REG_THRESHOLD_SENSITIVITY = 0x004A
REG_REFLECTOR_SHAPE = 0x004B
REG_FIXED_STRENGTH_THRESHOLD_VALUE = 0x004C
REG_COMMAND = 0x0100
REG_APPLICATION_ID = 0xFFFF

COMMAND_APPLY_CONFIG_AND_CALIBRATE = 1
COMMAND_MEASURE_DISTANCE = 2
COMMAND_RESET_MODULE = 1381192737
PROFILE5 = 5
THRESHOLD_CFAR = 3
PEAK_STRONGEST = 2
REFLECTOR_GENERIC = 1

DISTANCE_COUNT_MASK = 0x0F
DISTANCE_CALIBRATION_NEEDED = 1 << 9
DISTANCE_ERROR = 1 << 10
DETECTOR_BUSY = 1 << 31
DETECTOR_ERROR_MASK = 0x13FF0000


def signed_u32(value):
    """Return a two's-complement 32-bit register value as a Python int."""
    return value - (1 << 32) if value & (1 << 31) else value


class A121I2C:
    """Small implementation of the Waveshare I2C register protocol."""

    def __init__(self, bus_number, address, busy_gpio=-1):
        if SMBus is None or i2c_msg is None:
            raise RuntimeError('Missing smbus2. Install python3-smbus2 or pip install smbus2.')
        self._bus = SMBus(bus_number)
        self._address = address
        self._busy = None
        if busy_gpio >= 0:
            if Button is None:
                raise RuntimeError('Missing gpiozero for BUSY GPIO support. Install python3-gpiozero.')
            self._busy = Button(busy_gpio)

    def close(self):
        if self._busy is not None:
            self._busy.close()
        self._bus.close()

    def write_u32(self, register, value):
        value &= 0xFFFFFFFF
        payload = [register >> 8, register & 0xFF,
                   value >> 24, (value >> 16) & 0xFF,
                   (value >> 8) & 0xFF, value & 0xFF]
        self._bus.i2c_rdwr(i2c_msg.write(self._address, payload))

    def read_u32(self, register):
        write = i2c_msg.write(self._address, [register >> 8, register & 0xFF])
        read = i2c_msg.read(self._address, 4)
        self._bus.i2c_rdwr(write, read)
        data = list(read)
        return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]

    def wait_ready(self, timeout_s=2.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self.read_u32(REG_DETECTOR_STATUS)
            if status & DETECTOR_ERROR_MASK:
                raise RuntimeError(f'A121 detector error: 0x{status:08X}')
            if not status & DETECTOR_BUSY:
                return
            time.sleep(0.005)
        raise TimeoutError('Timed out waiting for A121 detector')

    def initialize(self, start_mm, end_mm):
        application_id = self.read_u32(REG_APPLICATION_ID)
        if application_id != 1:
            raise RuntimeError(
                f'A121 firmware application id is {application_id}, expected 1 (i2c_distance_detector).')
        self.write_u32(REG_COMMAND, COMMAND_RESET_MODULE)
        time.sleep(0.1)
        self.write_u32(REG_START, start_mm)
        self.write_u32(REG_END, end_mm)
        self.write_u32(REG_MAX_STEP_LENGTH, 0)
        self.write_u32(REG_CLOSE_RANGE_LEAKAGE_CANCEL, 0)
        self.write_u32(REG_SIGNAL_QUALITY, 15000)
        self.write_u32(REG_MAX_PROFILE, PROFILE5)
        self.write_u32(REG_THRESHOLD_METHOD, THRESHOLD_CFAR)
        self.write_u32(REG_PEAK_SORTING, PEAK_STRONGEST)
        self.write_u32(REG_NUM_FRAMES_RECORDED_THRESHOLD, 100)
        self.write_u32(REG_FIXED_AMPLITUDE_THRESHOLD_VALUE, 100000)
        self.write_u32(REG_THRESHOLD_SENSITIVITY, 500)
        self.write_u32(REG_REFLECTOR_SHAPE, REFLECTOR_GENERIC)
        self.write_u32(REG_FIXED_STRENGTH_THRESHOLD_VALUE, 0)
        self.write_u32(REG_COMMAND, COMMAND_APPLY_CONFIG_AND_CALIBRATE)
        self.wait_ready(timeout_s=5.0)

    def measure(self):
        self.write_u32(REG_COMMAND, COMMAND_MEASURE_DISTANCE)
        self.wait_ready()
        result = self.read_u32(REG_DISTANCE_RESULT)
        if result & DISTANCE_ERROR:
            raise RuntimeError(f'A121 distance measurement error: 0x{result:08X}')
        if result & DISTANCE_CALIBRATION_NEEDED:
            raise RuntimeError('A121 calibration required; restarting configuration.')

        count = result & DISTANCE_COUNT_MASK
        peaks = []
        for index in range(count):
            distance_mm = self.read_u32(REG_PEAK0_DISTANCE + index)
            strength_db = signed_u32(self.read_u32(REG_PEAK0_STRENGTH + index)) / 1000.0
            peaks.append((distance_mm, strength_db))
        return peaks


class RadarDriverNode(Node):
    def __init__(self):
        super().__init__('radar_driver_node')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x52)
        self.declare_parameter('busy_gpio', -1)
        self.declare_parameter('min_range_mm', 250)
        self.declare_parameter('max_range_mm', 3000)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('presence_distance_m', 3.0)
        self.declare_parameter('log_directory', str(Path.home() / 'bike_adas_logs'))

        self.range_pub = self.create_publisher(Float32, '/radar/range', 10)
        self.state_pub = self.create_publisher(RangeSensorState, '/radar/state', 10)
        self._sensor = None
        self._last_connect_attempt = 0.0
        self._log_directory = Path(os.path.expanduser(self.get_parameter('log_directory').value))
        self._range_log_file = None
        self._range_writer = None
        self._open_log_file()

        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0:
            raise ValueError('publish_rate_hz must be greater than zero')
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

    def _open_log_file(self):
        self._log_directory.mkdir(parents=True, exist_ok=True)
        path = self._log_directory / 'radar_state_log.csv'
        is_new = not path.exists()
        self._range_log_file = open(path, 'a', newline='', encoding='utf-8')
        self._range_writer = csv.writer(self._range_log_file)
        if is_new:
            self._range_writer.writerow(['timestamp', 'distance_m', 'presence', 'signal_strength_db'])
            self._range_log_file.flush()

    def _connect(self):
        now = time.monotonic()
        if now - self._last_connect_attempt < 5.0:
            return False
        self._last_connect_attempt = now
        sensor = None
        try:
            sensor = A121I2C(
                int(self.get_parameter('i2c_bus').value),
                int(self.get_parameter('i2c_address').value),
                int(self.get_parameter('busy_gpio').value),
            )
            sensor.initialize(
                int(self.get_parameter('min_range_mm').value),
                int(self.get_parameter('max_range_mm').value),
            )
            self._sensor = sensor
            self.get_logger().info('A121 I2C distance detector connected and calibrated')
            return True
        except Exception as exc:
            if sensor is not None:
                sensor.close()
            self.get_logger().warn(f'A121 unavailable: {exc}')
            return False

    def _disconnect(self):
        if self._sensor is not None:
            self._sensor.close()
        self._sensor = None

    def _on_timer(self):
        if self._sensor is None and not self._connect():
            return
        try:
            peaks = self._sensor.measure()
        except Exception as exc:
            self.get_logger().warn(f'A121 measurement failed: {exc}')
            self._disconnect()
            return

        if peaks:
            distance_mm, strength_db = peaks[0]
            distance_m = distance_mm / 1000.0
            presence = distance_m <= float(self.get_parameter('presence_distance_m').value)
        else:
            distance_m, strength_db, presence = 0.0, 0.0, False

        self.range_pub.publish(Float32(data=distance_m))
        state = RangeSensorState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'radar_link'
        state.distance_m = distance_m
        state.presence = presence
        state.motion = False  # Distance firmware does not report motion classification.
        state.signal_strength = strength_db
        state.confidence = 1.0 if peaks else 0.0
        self.state_pub.publish(state)

        self._range_writer.writerow([
            datetime.now(timezone.utc).isoformat(), f'{distance_m:.3f}', presence, f'{strength_db:.3f}'
        ])
        self._range_log_file.flush()

    def destroy_node(self):
        self._disconnect()
        if self._range_log_file is not None:
            self._range_log_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RadarDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
