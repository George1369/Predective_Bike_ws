#!/usr/bin/env python3
"""Simple screen display node for GPS and radar data."""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

try:
    from bike_msgs.msg import RangeSensorState
except ModuleNotFoundError:  # pragma: no cover - fallback for tests
    class RangeSensorState:  # type: ignore[override]
        pass


def format_display_text(data):
    gps_lat = data.get('gps_lat', 0.0)
    gps_lon = data.get('gps_lon', 0.0)
    gps_speed = data.get('gps_speed', 0.0)
    radar_distance = data.get('radar_distance', 0.0)
    radar_presence = data.get('radar_presence', False)
    radar_motion = data.get('radar_motion', False)
    radar_confidence = data.get('radar_confidence', 0.0)

    presence_text = 'PRESENT' if radar_presence else 'CLEAR'
    motion_text = 'MOVING' if radar_motion else 'STILL'

    return (
        '=== BICYCLE ADAS DISPLAY ===\n'
        f'GPS: lat={gps_lat:.4f} lon={gps_lon:.4f}\n'
        f'Speed: {gps_speed:.2f} m/s\n'
        f'Radar: {radar_distance:.2f} m | {presence_text} | {motion_text}\n'
        f'Confidence: {radar_confidence:.2f}'
    )


def build_map_overlay(lat, lon, speed, obstacle=False):
    hazard_text = 'Hazard: obstacle ahead' if obstacle else 'Hazard: clear'
    route_text = 'Route: live tracking' if not obstacle else 'Route: reroute advised'
    return (
        '=== SIMULATED MAP ===\n'
        f'Lat: {lat:.4f}\n'
        f'Lon: {lon:.4f}\n'
        f'Speed: {speed:.2f} m/s\n'
        f'{route_text}\n'
        f'{hazard_text}\n'
        'Map: [o] bike   [x] hazard'
    )


class ScreenDisplayNode(Node):
    def __init__(self):
        super().__init__('screen_display_node')
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self._gps_cb, 10)
        self.speed_sub = self.create_subscription(Float32, '/gps/speed', self._speed_cb, 10)
        self.range_sub = self.create_subscription(Float32, '/radar/range', self._range_cb, 10)
        self.state_sub = self.create_subscription(RangeSensorState, '/radar/state', self._state_cb, 10)

        self._gps_lat = 0.0
        self._gps_lon = 0.0
        self._gps_speed = 0.0
        self._radar_distance = 0.0
        self._radar_presence = False
        self._radar_motion = False
        self._radar_confidence = 0.0

        self.timer = self.create_timer(0.5, self._on_timer)

    def _gps_cb(self, msg):
        self._gps_lat = msg.latitude
        self._gps_lon = msg.longitude

    def _speed_cb(self, msg):
        self._gps_speed = msg.data

    def _range_cb(self, msg):
        self._radar_distance = msg.data

    def _state_cb(self, msg):
        self._radar_distance = msg.distance_m
        self._radar_presence = msg.presence
        self._radar_motion = msg.motion
        self._radar_confidence = msg.confidence

    def _on_timer(self):
        data = {
            'gps_lat': self._gps_lat,
            'gps_lon': self._gps_lon,
            'gps_speed': self._gps_speed,
            'radar_distance': self._radar_distance,
            'radar_presence': self._radar_presence,
            'radar_motion': self._radar_motion,
            'radar_confidence': self._radar_confidence,
        }
        text = format_display_text(data)
        map_overlay = build_map_overlay(self._gps_lat, self._gps_lon, self._gps_speed)
        os.system('clear')
        print(text)
        print('')
        print(map_overlay)


def main(args=None):
    rclpy.init(args=args)
    node = ScreenDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
