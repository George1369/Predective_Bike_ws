#!/usr/bin/env python3
"""
gps_driver_node.py

UART GPS/GNSS driver node for the Waveshare L76K module on Raspberry Pi 5.

Publishes:
    /gps/fix     (sensor_msgs/NavSatFix)  - position + fix status
    /gps/speed   (std_msgs/Float32)       - speed over ground, m/s

Hardware:
    Serial port : /dev/serial0 (GPIO UART, physical pins 8/10)
    Baud rate   : 9600 (L76K default)

    Wiring:
        GPS VCC  -> Pi pin 4  (5V)
        GPS GND  -> Pi pin 6  (GND)
        GPS TX   -> Pi pin 10 (GPIO15 / RXD0)
        GPS RX   -> Pi pin 8  (GPIO14 / TXD0)

    Use /dev/serial0 rather than a ttyAMA*/ttyS* name. Raspberry Pi OS maps
    that stable symlink to the UART currently assigned to GPIO14/15.
"""

import csv
import os
from datetime import datetime
from pathlib import Path

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32

PUBLISH_RATE_HZ = 1.0  # standard GPS fix rate


# ---------------------------------------------------------------------------
# NMEA helpers
# ---------------------------------------------------------------------------

def _dms_to_decimal(raw_deg_min: str, hemisphere: str) -> float:
    """Convert NMEA degree-minutes string (DDDMM.mmmmm) to decimal degrees."""
    if not raw_deg_min:
        return 0.0
    dot = raw_deg_min.index('.')
    deg_chars = dot - 2          # always 2 digits of minutes before decimal
    degrees = float(raw_deg_min[:deg_chars])
    minutes = float(raw_deg_min[deg_chars:])
    value = degrees + minutes / 60.0
    if hemisphere in {'S', 'W'}:
        value = -value
    return value


def parse_nmea_sentence(sentence: str):
    """
    Parse a single NMEA sentence and return
    (lat_deg, lon_deg, alt_m, speed_mps, has_fix).

    Accepts any GNSS talker prefix (GP, GN, GL, GA, GB …) so that
    multi-constellation receivers that emit $GNRMC / $GNGGA are handled
    correctly — not just single-system $GPRMC / $GPGGA sentences.
    """
    sentence = sentence.strip()
    # strip leading '$' and optional checksum suffix (*XX)
    if sentence.startswith('$'):
        sentence = sentence[1:]
    if '*' in sentence:
        sentence = sentence[:sentence.index('*')]
    if not sentence:
        return 0.0, 0.0, 0.0, 0.0, False

    parts = sentence.split(',')
    msg_type = parts[0]  # e.g. 'GNRMC', 'GPRMC', 'GNGGA', 'GPGGA' …

    # ------------------------------------------------------------------
    # RMC — Recommended Minimum Specific GNSS Data
    # Fields: type,time,status,lat,NS,lon,EW,speed_kn,course,date,...
    # ------------------------------------------------------------------
    if msg_type.endswith('RMC'):
        if len(parts) < 8:
            return 0.0, 0.0, 0.0, 0.0, False
        status = parts[2]          # 'A' = valid, 'V' = void
        if status != 'A':
            return 0.0, 0.0, 0.0, 0.0, False
        lat = _dms_to_decimal(parts[3], parts[4])
        lon = _dms_to_decimal(parts[5], parts[6])
        speed_knots = float(parts[7]) if parts[7] else 0.0
        speed_mps = speed_knots * 0.5144444444
        return lat, lon, 0.0, speed_mps, True

    # ------------------------------------------------------------------
    # GGA — Global Positioning System Fix Data
    # Fields: type,time,lat,NS,lon,EW,quality,sats,hdop,alt,M,...
    # ------------------------------------------------------------------
    if msg_type.endswith('GGA'):
        if len(parts) < 10:
            return 0.0, 0.0, 0.0, 0.0, False
        fix_quality = int(parts[6]) if parts[6] else 0
        if fix_quality == 0:       # 0 = no fix
            return 0.0, 0.0, 0.0, 0.0, False
        lat = _dms_to_decimal(parts[2], parts[3])
        lon = _dms_to_decimal(parts[4], parts[5])
        alt = float(parts[9]) if parts[9] else 0.0
        return lat, lon, alt, 0.0, True

    return 0.0, 0.0, 0.0, 0.0, False


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class GpsDriverNode(Node):
    def __init__(self):
        super().__init__('gps_driver_node')

        # Parameters (override at runtime with --ros-args -p key:=value)
        self.declare_parameter('serial_port', '/dev/serial0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('log_directory',
                               str(Path.home() / 'bike_adas_logs'))

        self._serial_port = self.get_parameter('serial_port').value
        self._baud_rate = self.get_parameter('baud_rate').value
        self._log_dir = Path(
            os.path.expanduser(
                self.get_parameter('log_directory').value))

        # Publishers
        self._fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self._speed_pub = self.create_publisher(Float32, '/gps/speed', 10)

        # State
        self._serial = None
        self._last_sentence = ''
        self._gps_log_file = None
        self._gps_writer = None
        self._raw_nmea_log_file = None

        self._connect_serial()
        self._open_log_files()

        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._on_timer)
        self.get_logger().info(
            f'gps_driver_node started — port={self._serial_port} '
            f'baud={self._baud_rate}')

    # ------------------------------------------------------------------
    # Serial
    # ------------------------------------------------------------------

    def _connect_serial(self):
        try:
            self._serial = serial.Serial(
                self._serial_port,
                self._baud_rate,
                timeout=1.0,
            )
            self.get_logger().info(
                f'Serial port {self._serial_port} connected')
        except (serial.SerialException, FileNotFoundError) as exc:
            self.get_logger().warn(
                f'Unable to open serial port {self._serial_port}: {exc}')
            self._serial = None

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _open_log_files(self):
        self._log_dir.mkdir(parents=True, exist_ok=True)

        fix_path = self._log_dir / 'gps_fix_log.csv'
        nmea_path = self._log_dir / 'gps_nmea.log'

        is_new = not fix_path.exists()
        self._gps_log_file = open(fix_path, 'a', newline='', encoding='utf-8')
        self._gps_writer = csv.writer(self._gps_log_file)
        if is_new:
            self._gps_writer.writerow([
                'timestamp', 'fix_status',
                'latitude', 'longitude', 'altitude_m',
                'speed_mps', 'raw_nmea',
            ])
            self._gps_log_file.flush()

        self._raw_nmea_log_file = open(nmea_path, 'a', encoding='utf-8')

    def _log_raw_nmea(self, sentence: str):
        if self._raw_nmea_log_file is None:
            return
        ts = datetime.utcnow().isoformat() + 'Z'
        self._raw_nmea_log_file.write(f'{ts},{sentence}\n')
        self._raw_nmea_log_file.flush()

    def _log_fix(self, has_fix: bool, lat, lon, alt, speed, raw: str):
        if self._gps_writer is None:
            return
        ts = datetime.utcnow().isoformat() + 'Z'
        self._gps_writer.writerow([
            ts,
            'FIX' if has_fix else 'NO_FIX',
            f'{lat:.8f}',
            f'{lon:.8f}',
            f'{alt:.2f}',
            f'{speed:.3f}',
            raw,
        ])
        self._gps_log_file.flush()

    def _close_logs(self):
        for f in (self._gps_log_file, self._raw_nmea_log_file):
            if f is not None:
                f.close()

    # ------------------------------------------------------------------
    # Hardware read
    # ------------------------------------------------------------------

    def _read_hardware(self):
        """
        Drain all buffered NMEA sentences from the serial port and return
        the most recent valid fix found in this cycle.
        Returns (lat_deg, lon_deg, alt_m, speed_mps, has_fix).
        """
        if self._serial is None:
            return 0.0, 0.0, 0.0, 0.0, False

        best = (0.0, 0.0, 0.0, 0.0, False)

        try:
            # Read every sentence currently buffered — GPS emits ~13 lines/sec
            # but the timer only fires at 1 Hz, so we drain the whole burst
            # and keep the most recent valid fix rather than a single line.
            while self._serial.in_waiting > 0:
                raw = self._serial.readline()
                line = raw.decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                self._last_sentence = line
                self._log_raw_nmea(line)
                result = parse_nmea_sentence(line)
                if result[4]:    # has_fix — prefer the newest valid sentence
                    best = result
        except serial.SerialException as exc:
            self.get_logger().warn(f'Serial read error: {exc}')
            return 0.0, 0.0, 0.0, 0.0, False

        return best

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _on_timer(self):
        lat, lon, alt, speed_mps, has_fix = self._read_hardware()

        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = 'gps_link'
        fix_msg.status.status = (
            NavSatStatus.STATUS_FIX if has_fix else NavSatStatus.STATUS_NO_FIX
        )
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = lat
        fix_msg.longitude = lon
        fix_msg.altitude = alt
        # Covariance left as zeros — L76K does not report position uncertainty
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self._fix_pub.publish(fix_msg)
        self._speed_pub.publish(Float32(data=speed_mps))
        self._log_fix(has_fix, lat, lon, alt, speed_mps, self._last_sentence)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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
