# Raspberry Pi 5 Hardware Setup

This workspace runs the Waveshare 6.25-inch HDMI display, Waveshare A121
Range Sensor with its I2C distance-detector firmware, and Waveshare L76K GPS
module. It does not yet support an IMU or camera because their part numbers
and interfaces have not been specified.

## 1. Raspberry Pi OS prerequisites

Use 64-bit Raspberry Pi OS with a ROS 2 Jazzy installation available in your
shell. Enable the two hardware interfaces, disable the serial login shell, and
install the operating-system packages:

```bash
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_serial 2
sudo apt update
sudo apt install -y python3-smbus2 python3-gpiozero python3-lgpio i2c-tools
sudo usermod -aG i2c,dialout "$USER"
```

Log out and back in after changing group membership. Check the buses before
launching the application:

```bash
ls -l /dev/serial0 /dev/i2c-1
i2cdetect -y 1
```

The radar should appear at `52` in the `i2cdetect` output. If it does not,
turn power off before correcting wiring.

## 2. Display

Connect the display HDMI port to the Pi, the display touch USB port to the Pi,
and power the display with its own 5 V USB-C supply capable of at least 400 mA.
Do not rely on an undervoltage Pi supply for the display.

Append the following vendor timing configuration to `/boot/firmware/config.txt`
on current Raspberry Pi OS, then reboot:

```ini
hdmi_group=2
hdmi_mode=87
hdmi_force_hotplug=1
max_framebuffer_width=720
max_framebuffer_height=1560
hdmi_timings=720 0 22 10 78 1560 0 13 3 13 0 0 0 57 0 75000000 0
```

The dashboard is fullscreen and is designed for the panel's portrait 720x1560
mode. Press `Esc` to close it during bench testing.

## 3. A121 radar (I2C distance detector)

Flash the Waveshare `i2c_distance_detector` firmware first. The node rejects
any other firmware application ID rather than silently publishing bad data.

Wire the A121 as follows:

| A121 pin | Raspberry Pi pin |
| --- | --- |
| 5V | 5V (physical pin 2 or 4) |
| GND | GND (for example physical pin 6) |
| SDA | GPIO2/SDA1 (physical pin 3) |
| SCL | GPIO3/SCL1 (physical pin 5) |
| BUSY (optional) | GPIO4 (physical pin 7) |

The driver uses the I2C detector-status register for completion, so BUSY is
optional. If it is connected, pass its BCM GPIO number when launching:

```bash
ros2 launch display_node bike_dashboard.launch.py radar_busy_gpio:=4
```

The default I2C address is decimal `82` (`0x52`). If you changed the board's
address resistors, pass the new decimal address with `radar_i2c_address:=...`.

## 4. L76K GPS

Connect the GPS antenna and wire the module exactly as follows:

| L76K pin | Raspberry Pi pin |
| --- | --- |
| VCC | 5V |
| GND | GND |
| TX | GPIO15/RXD, physical pin 10 |
| RX | GPIO14/TXD, physical pin 8 |
| PPS | Not connected |

The node uses `/dev/serial0`, Raspberry Pi OS's stable GPIO-UART symlink, at
the L76K default of 9600 baud. Verify raw NMEA output before running ROS:

```bash
sudo apt install -y minicom
minicom -D /dev/serial0 -b 9600
```

Initial satellite acquisition can take about 35 seconds outdoors with an
unobstructed view of the sky.

## 5. Run

```bash
cd ~/Predective_Bike_ws
source ~/ros2_jazzy/install/setup.bash
colcon build --symlink-install
source install/setup.bash
./run_bike_dashboard.sh
```

The launcher uses the active ROS environment and safely sources the workspace
overlay. Do not type a literal placeholder such as `/opt/ros/<your_ros2_distro>`.
