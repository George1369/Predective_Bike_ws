#!/usr/bin/env python3
"""
gps_driver_node.py

Phase 1 driver node for a UART GPS/GNSS module (e.g. u-blox NEO-M9N).

Publishes:
    /gps/fix     (sensor_msgs/NavSatFix)
    /gps/speed   (std_msgs/Float32)     - speed over ground, m/s

TODO (hardware bring-up):
    - Replace `_read_hardware()` with real UART reads + NMEA/UBX parsing
      (e.g. pyserial + pynmea2, or gpsd via ROS 2 gpsd_client).
    - Confirm serial port name (commonly /dev/ttyAMA0 or /dev/ttyUSB0 on Pi)
      and baud rate (commonly 9600 or 38400) against your specific module.
"""

import csv
import math
import os
import re
from datetime import datetime
from pathlib import Path

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32

PUBLISH_RATE_HZ = 1.0  # typical GPS fix rate


def _dms_to_decimal(degrees, minutes, hemisphere):
    value = float(degrees) + float(minutes) / 60.0
    if hemisphere in {'S', 'W'}:
        value = -value
    return value


def parse_nmea_sentence(sentence):
    sentence = sentence.strip()
    if sentence.startswith('$'):
        sentence = sentence[1:]
    if not sentence:
        return 0.0, 0.0, 0.0, 0.0, False

    parts = sentence.split(',')
    if not parts:
        return 0.0, 0.0, 0.0, 0.0, False

    talker = parts[0][:2]
    valid_talkers = {'GP', 'GN', 'GL', 'BD', 'GA'}

    if talker in valid_talkers and parts[0].endswith('RMC'):
        if len(parts) < 12:
            return 0.0, 0.0, 0.0, 0.0, False
        status = parts[2]
        if status != 'A':
            return 0.0, 0.0, 0.0, 0.0, False
        lat = _dms_to_decimal(parts[3][:2], parts[3][2:], parts[4])
        lon = _dms_to_decimal(parts[5][:3], parts[5][3:], parts[6])
        speed_knots = float(parts[7]) if parts[7] else 0.0
        speed_mps = speed_knots * 0.5144444444
        return lat, lon, 0.0, speed_mps, True

    if talker in valid_talkers and parts[0].endswith('GGA'):
        if len(parts) < 9:
            return 0.0, 0.0, 0.0, 0.0, False
        fix_quality = int(parts[6]) if parts[6] else 0
        if fix_quality == 0:
            return 0.0, 0.0, 0.0, 0.0, False
        lat = _dms_to_decimal(parts[2][:2], parts[2][2:], parts[3])
        lon = _dms_to_decimal(parts[4][:3], parts[4][3:], parts[5])
        alt = float(parts[9]) if parts[9] else 0.0
        return lat, lon, alt, 0.0, True

    return 0.0, 0.0, 0.0, 0.0, False


class GpsDriverNode(Node):
    def __init__(self):
        super().__init__('gps_driver_node')

        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('log_directory', str(Path.home() / 'bike_adas_logs'))

        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.speed_pub = self.create_publisher(Float32, '/gps/speed', 10)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.log_directory = Path(os.path.expanduser(self.get_parameter('log_directory').value))
        self._serial = None
        self._last_sentence = None
        self._gps_log_file = None
        self._gps_writer = None
        self._raw_nmea_log_file = None
        self._connect_serial()
        self._open_log_files()

        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._on_timer)
        self.get_logger().info(f'gps_driver_node started on {self.serial_port} at {self.baud_rate}')

    def _connect_serial(self):
        try:
            self._serial = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            self.get_logger().info('Serial port connected')
        except (serial.SerialException, FileNotFoundError) as exc:
            self.get_logger().warn(f'Unable to open serial port {self.serial_port}: {exc}')
            self._serial = None

    def _open_log_files(self):
        self.log_directory.mkdir(parents=True, exist_ok=True)
        gps_log_path = self.log_directory / 'gps_fix_log.csv'
        raw_nmea_path = self.log_directory / 'gps_nmea.log'
        is_new = not gps_log_path.exists()
        self._gps_log_file = open(gps_log_path, 'a', newline='', encoding='utf-8')
        self._gps_writer = csv.writer(self._gps_log_file)
        if is_new:
            self._gps_writer.writerow([
                'timestamp', 'fix_status', 'latitude', 'longitude',
                'altitude_m', 'speed_mps', 'raw_nmea'
            ])
            self._gps_log_file.flush()
        self._raw_nmea_log_file = open(raw_nmea_path, 'a', encoding='utf-8')

    def _log_raw_nmea(self, sentence):
        if self._raw_nmea_log_file is None:
            return
        timestamp = datetime.utcnow().isoformat() + 'Z'
        self._raw_nmea_log_file.write(f'{timestamp},{sentence}\n')
        self._raw_nmea_log_file.flush()

    def _log_fix_data(self, fix_status, lat, lon, alt, speed, raw_sentence):
        if self._gps_writer is None:
            return
        timestamp = datetime.utcnow().isoformat() + 'Z'
        self._gps_writer.writerow([
            timestamp,
            'FIX' if fix_status else 'NO_FIX',
            f'{lat:.8f}',
            f'{lon:.8f}',
            f'{alt:.2f}',
            f'{speed:.3f}',
            raw_sentence or '',
        ])
        self._gps_log_file.flush()

    def _close_logs(self):
        if self._gps_log_file is not None:
            self._gps_log_file.close()
        if self._raw_nmea_log_file is not None:
            self._raw_nmea_log_file.close()

    def _read_hardware(self):
        """
        Read the next NMEA sentence from the GPS serial port and parse it.
        Must return (lat_deg, lon_deg, alt_m, speed_mps, has_fix).
        """
        if self._serial is None:
            return 0.0, 0.0, 0.0, 0.0, False

        try:
            line = self._serial.readline().decode('ascii', errors='ignore').strip()
        except serial.SerialException as exc:
            self.get_logger().warn(f'Serial read failed: {exc}')
            return 0.0, 0.0, 0.0, 0.0, False

        if not line:
            return 0.0, 0.0, 0.0, 0.0, False

        self._last_sentence = line
        self._log_raw_nmea(line)
        return parse_nmea_sentence(line)

    def _on_timer(self):
        lat, lon, alt, speed_mps, has_fix = self._read_hardware()

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.status.status = (
            NavSatStatus.STATUS_FIX if has_fix else NavSatStatus.STATUS_NO_FIX
        )
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt

        self.fix_pub.publish(msg)
        self.speed_pub.publish(Float32(data=speed_mps))
        self._log_fix_data(has_fix, lat, lon, alt, speed_mps, self._last_sentence)


def main(args=None):
    rclpy.init(args=args)
    node = GpsDriverNode()
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
