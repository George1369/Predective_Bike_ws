#!/usr/bin/env python3
"""
Basic radar driver node for the Waveshare A121 range sensor.

This implementation uses a simple text-based interface for development:
- it reads a line from stdin or a serial device,
- parses a distance sample such as 'distance:1.42',
- publishes it as a Float32 on /radar/range.

It is intentionally lightweight so it can be adapted to UART/I2C later.
"""

import csv
import os
from datetime import datetime
from pathlib import Path

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    from bike_msgs.msg import RangeSensorState
except ModuleNotFoundError:
    class RangeSensorState:
        def __init__(self):
            self.header = type('Header', (), {'stamp': None, 'frame_id': ''})()
            self.distance_m = 0.0
            self.presence = False
            self.motion = False
            self.signal_strength = 0.0
            self.confidence = 0.0


def parse_distance_sample(sample):
    if not sample:
        return None
    text = sample.strip().lower()
    if not text.startswith('distance:'):
        return None
    try:
        return float(text.split(':', 1)[1])
    except ValueError:
        return None


def parse_range_state(sample):
    if not sample:
        return None
    text = sample.strip().lower()
    if not text.startswith('range:'):
        return None

    payload = text[len('range:'):]
    parts = payload.split(';')
    values = {}
    for part in parts:
        if ':' not in part:
            continue
        if part.startswith('distance:'):
            key = 'distance'
            value = part[len('distance:'):]
        else:
            key, value = part.split(':', 1)
        values[key.strip()] = value.strip()

    if 'distance' not in values:
        return None

    try:
        distance = float(values['distance'])
        presence = values.get('presence', '0').lower() in {'1', 'true', 'yes'}
        motion = values.get('motion', '0').lower() in {'1', 'true', 'yes'}
        signal_strength = float(values.get('signal_strength', '0.0'))
        confidence = float(values.get('confidence', '0.0'))
        return distance, presence, motion, signal_strength, confidence
    except ValueError:
        return None


class RadarDriverNode(Node):
    def __init__(self):
        super().__init__('radar_driver_node')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)

        self.declare_parameter('log_directory', str(Path.home() / 'bike_adas_logs'))

        self.range_pub = self.create_publisher(Float32, '/radar/range', 10)
        self.state_pub = self.create_publisher(RangeSensorState, '/radar/state', 10)
        self.timer = self.create_timer(0.1, self._on_timer)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.log_directory = Path(os.path.expanduser(self.get_parameter('log_directory').value))
        self._serial = None
        self._range_log_file = None
        self._raw_log_file = None
        self._connect_serial()
        self._open_log_files()

        self.get_logger().info(f'radar_driver_node started on {self.serial_port} at {self.baud_rate}')

    def _connect_serial(self):
        try:
            self._serial = serial.Serial(self.serial_port, self.baud_rate, timeout=0.2)
            self.get_logger().info('Serial port connected')
        except (serial.SerialException, FileNotFoundError) as exc:
            self.get_logger().warn(f'Unable to open serial port {self.serial_port}: {exc}')
            self._serial = None

    def _open_log_files(self):
        self.log_directory.mkdir(parents=True, exist_ok=True)
        range_log_path = self.log_directory / 'radar_state_log.csv'
        raw_log_path = self.log_directory / 'radar_raw.log'
        is_new = not range_log_path.exists()
        self._range_log_file = open(range_log_path, 'a', newline='', encoding='utf-8')
        self._range_writer = csv.writer(self._range_log_file)
        if is_new:
            self._range_writer.writerow([
                'timestamp', 'distance_m', 'presence', 'motion', 'signal_strength', 'confidence', 'raw_line'
            ])
            self._range_log_file.flush()
        self._raw_log_file = open(raw_log_path, 'a', encoding='utf-8')

    def _log_raw_line(self, line):
        if self._raw_log_file is None:
            return
        timestamp = datetime.utcnow().isoformat() + 'Z'
        self._raw_log_file.write(f'{timestamp},{line}\n')
        self._raw_log_file.flush()

    def _log_range_state(self, distance, presence, motion, signal_strength, confidence, raw_line):
        if self._range_writer is None:
            return
        timestamp = datetime.utcnow().isoformat() + 'Z'
        self._range_writer.writerow([
            timestamp,
            f'{distance:.3f}',
            str(presence),
            str(motion),
            f'{signal_strength:.3f}',
            f'{confidence:.3f}',
            raw_line,
        ])
        self._range_log_file.flush()

    def _close_logs(self):
        if self._range_log_file is not None:
            self._range_log_file.close()
        if self._raw_log_file is not None:
            self._raw_log_file.close()

    def _on_timer(self):
        if self._serial is None:
            return

        try:
            line = self._serial.readline().decode('ascii', errors='ignore').strip()
        except serial.SerialException as exc:
            self.get_logger().warn(f'Serial read failed: {exc}')
            return

        if not line:
            return

        self._log_raw_line(line)

        distance = parse_distance_sample(line)
        if distance is not None:
            msg = Float32(data=distance)
            self.range_pub.publish(msg)

        state = parse_range_state(line)
        if state is not None:
            distance_m, presence, motion, signal_strength, confidence = state
            msg = RangeSensorState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'radar_link'
            msg.distance_m = distance_m
            msg.presence = presence
            msg.motion = motion
            msg.signal_strength = signal_strength
            msg.confidence = confidence
            self.state_pub.publish(msg)
            self._log_range_state(distance_m, presence, motion, signal_strength, confidence, line)


def main(args=None):
    rclpy.init(args=args)
    node = RadarDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close_logs()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
