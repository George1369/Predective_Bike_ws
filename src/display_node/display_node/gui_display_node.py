#!/usr/bin/env python3
"""Graphical dashboard for GPS and radar data on an HDMI display."""

import math
import os
import tempfile
import threading
import tkinter as tk
from tkinter import ttk
import urllib.request
from urllib.error import URLError, HTTPError

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

try:
    from bike_msgs.msg import RangeSensorState
except ModuleNotFoundError:  # pragma: no cover
    class RangeSensorState:  # type: ignore[override]
        pass


def build_osm_tile_url(lat, lon, zoom=15):
    clipped_lat = max(-85.0, min(85.0, lat))
    lon = ((lon + 180.0) % 360.0) - 180.0
    lat_rad = math.radians(clipped_lat)
    n = 2 ** zoom
    x_tile = int((lon + 180.0) / 360.0 * n)
    y_tile = int((1.0 - math.log(math.tan(lat_rad) + (1.0 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
    x_tile = max(0, min(x_tile, n - 1))
    y_tile = max(0, min(y_tile, n - 1))
    return f'https://tile.openstreetmap.org/{zoom}/{x_tile}/{y_tile}.png'


class DashboardNode(Node):
    def __init__(self):
        super().__init__('gui_display_node')
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

        self.root = tk.Tk()
        self.root.title('Bike ADAS Dashboard')
        self.root.configure(bg='#020617')
        self.root.attributes('-fullscreen', True)

        self._build_ui()
        self._spin_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._spin_thread.start()
        self._update_ui()

    def _spin_ros(self):
        rclpy.spin(self)

    def _build_ui(self):
        container = ttk.Frame(self.root, padding=24)
        container.pack(fill='both', expand=True)
        container.configure(style='Dashboard.TFrame')

        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('Dashboard.TFrame', background='#020617')
        style.configure('Title.TLabel', background='#020617', foreground='#f8fafc', font=('Helvetica', 28, 'bold'))
        style.configure('Section.TLabel', background='#020617', foreground='#38bdf8', font=('Helvetica', 18, 'bold'))
        style.configure('Value.TLabel', background='#020617', foreground='#f8fafc', font=('Helvetica', 20))
        style.configure('Small.TLabel', background='#020617', foreground='#cbd5e1', font=('Helvetica', 14))

        ttk.Label(container, text='Bicycle ADAS Dashboard', style='Title.TLabel').pack(anchor='w', pady=(0, 24))

        self.status_var = tk.StringVar(value='STATUS: SAFE')
        status_frame = tk.Frame(container, bg='#0f172a', bd=2, relief='ridge')
        status_frame.pack(fill='x', pady=(0, 16))
        tk.Label(status_frame, textvariable=self.status_var, bg='#0f172a', fg='#4ade80', font=('Helvetica', 24, 'bold'), padx=18, pady=14).pack(anchor='w')

        content = tk.Frame(container, bg='#020617')
        content.pack(fill='both', expand=True)

        gps_frame = tk.Frame(content, bg='#111827', bd=2, relief='groove')
        gps_frame.pack(fill='x', pady=(0, 12), padx=2)
        tk.Label(gps_frame, text='GPS', bg='#111827', fg='#38bdf8', font=('Helvetica', 18, 'bold'), padx=12, pady=10).pack(anchor='w')
        self.lat_var = tk.StringVar(value='lat: 0.0000')
        self.lon_var = tk.StringVar(value='lon: 0.0000')
        self.speed_var = tk.StringVar(value='speed: 0.00 m/s')
        tk.Label(gps_frame, textvariable=self.lat_var, bg='#111827', fg='#f8fafc', font=('Helvetica', 18), padx=12, pady=4).pack(anchor='w')
        tk.Label(gps_frame, textvariable=self.lon_var, bg='#111827', fg='#f8fafc', font=('Helvetica', 18), padx=12, pady=4).pack(anchor='w')
        tk.Label(gps_frame, textvariable=self.speed_var, bg='#111827', fg='#f8fafc', font=('Helvetica', 18), padx=12, pady=4).pack(anchor='w')

        map_frame = tk.Frame(content, bg='#111827', bd=2, relief='groove')
        map_frame.pack(fill='x', pady=(0, 12), padx=2)
        tk.Label(map_frame, text='Map', bg='#111827', fg='#38bdf8', font=('Helvetica', 18, 'bold'), padx=12, pady=10).pack(anchor='w')
        self.map_var = tk.StringVar(value='Route: live tracking')
        tk.Label(map_frame, textvariable=self.map_var, bg='#111827', fg='#f8fafc', font=('Helvetica', 16), padx=12, pady=6).pack(anchor='w')
        self.icon_var = tk.StringVar(value='⚪ Safe')
        tk.Label(map_frame, textvariable=self.icon_var, bg='#111827', fg='#4ade80', font=('Helvetica', 18, 'bold'), padx=12, pady=6).pack(anchor='w')
        self.map_canvas = tk.Canvas(map_frame, width=520, height=220, bg='#0f172a', highlightthickness=0)
        self.map_canvas.pack(fill='x', padx=12, pady=(0, 12))

        radar_frame = tk.Frame(content, bg='#111827', bd=2, relief='groove')
        radar_frame.pack(fill='x', pady=(0, 8), padx=2)
        tk.Label(radar_frame, text='Radar', bg='#111827', fg='#38bdf8', font=('Helvetica', 18, 'bold'), padx=12, pady=10).pack(anchor='w')
        self.distance_var = tk.StringVar(value='distance: 0.00 m')
        self.presence_var = tk.StringVar(value='presence: CLEAR')
        self.motion_var = tk.StringVar(value='motion: STILL')
        self.confidence_var = tk.StringVar(value='confidence: 0.00')
        tk.Label(radar_frame, textvariable=self.distance_var, bg='#111827', fg='#f8fafc', font=('Helvetica', 18), padx=12, pady=4).pack(anchor='w')
        tk.Label(radar_frame, textvariable=self.presence_var, bg='#111827', fg='#f8fafc', font=('Helvetica', 18), padx=12, pady=4).pack(anchor='w')
        tk.Label(radar_frame, textvariable=self.motion_var, bg='#111827', fg='#f8fafc', font=('Helvetica', 18), padx=12, pady=4).pack(anchor='w')
        tk.Label(radar_frame, textvariable=self.confidence_var, bg='#111827', fg='#f8fafc', font=('Helvetica', 18), padx=12, pady=4).pack(anchor='w')

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

    def _draw_map_placeholder(self):
        self.map_canvas.delete('all')
        self.map_canvas.create_rectangle(0, 0, 520, 220, fill='#1e293b', outline='#38bdf8')
        self.map_canvas.create_text(260, 95, text='OpenStreetMap tile\nloading...', fill='#f8fafc', font=('Helvetica', 14, 'bold'))

    def _draw_map_overlay(self):
        self.map_canvas.delete('all')
        self.map_canvas.create_rectangle(0, 0, 520, 220, fill='#0f172a', outline='#334155')

        try:
            tile_url = build_osm_tile_url(self._gps_lat, self._gps_lon, zoom=15)
            with urllib.request.urlopen(tile_url, timeout=2) as response:
                tile_bytes = response.read()
            if tile_bytes:
                handle, tmp_path = tempfile.mkstemp(suffix='.png')
                os.close(handle)
                with open(tmp_path, 'wb') as fh:
                    fh.write(tile_bytes)
                try:
                    self._map_photo = tk.PhotoImage(file=tmp_path)
                    self.map_canvas.create_image(0, 0, anchor='nw', image=self._map_photo)
                except tk.TclError:
                    self._draw_map_placeholder()
                finally:
                    try:
                        os.remove(tmp_path)
                    except OSError:
                        pass
            else:
                self._draw_map_placeholder()
        except (URLError, HTTPError, OSError):
            self._draw_map_placeholder()

        self.map_canvas.create_oval(240, 95, 280, 135, fill='#38bdf8', outline='#f8fafc', width=2)
        self.map_canvas.create_text(260, 148, text='bike', fill='#f8fafc', font=('Helvetica', 10, 'bold'))

        if self._radar_presence:
            self.map_canvas.create_rectangle(320, 40, 470, 140, outline='#f87171', width=3)
            self.map_canvas.create_text(395, 90, text='warning zone', fill='#f87171', font=('Helvetica', 12, 'bold'))

    def _update_ui(self):
        self.lat_var.set(f'lat: {self._gps_lat:.4f}')
        self.lon_var.set(f'lon: {self._gps_lon:.4f}')
        self.speed_var.set(f'speed: {self._gps_speed:.2f} m/s')
        self.distance_var.set(f'distance: {self._radar_distance:.2f} m')
        self.presence_var.set(f'presence: {"PRESENT" if self._radar_presence else "CLEAR"}')
        self.motion_var.set(f'motion: {"MOVING" if self._radar_motion else "STILL"}')
        self.confidence_var.set(f'confidence: {self._radar_confidence:.2f}')

        if self._radar_presence:
            self.status_var.set('STATUS: WARNING')
            self.root.configure(bg='#7f1d1d')
            self.icon_var.set('⚠️ Warning')
            self.map_var.set(f'Route: obstacle ahead • {self._radar_distance:.2f} m')
        else:
            self.status_var.set('STATUS: SAFE')
            self.root.configure(bg='#020617')
            self.icon_var.set('⚪ Safe')
            self.map_var.set('Route: live tracking')

        self._draw_map_overlay()
        self.root.after(250, self._update_ui)

    def start(self):
        try:
            self.root.mainloop()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        node.start()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
