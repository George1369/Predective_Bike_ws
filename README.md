# bike_ws -- Smart Bicycle ADAS ROS 2 Workspace

Targets ROS 2 Jazzy on Raspberry Pi OS (64-bit) on a Raspberry Pi 5. The
hardware deployment instructions are in [docs/RASPBERRY_PI_SETUP.md](docs/RASPBERRY_PI_SETUP.md).

## Packages

| Package        | Status                | Purpose |
|----------------|------------------------|---------|
| `bike_msgs`    | Ready                  | Custom message types (Detection2D, RadarTarget, TrackedObject, RiskState) |
| `imu_node`     | Stub hardware read     | No IMU model has been selected yet; it must not be used for safety decisions |
| `gps_node`     | Stub hardware read     | Phase 1: real GPS driver (fill in `_read_hardware()`) |
| `camera_node`  | Placeholder only       | Phase 1c/2: bring up after IMU + GPS are validated |
| `radar_node`   | A121 I2C distance mode | Waveshare A121 with `i2c_distance_detector` firmware |
| `mock_sensors` | Ready                  | Synthetic IMU/GPS publishers for PC-only development |

## Build

```bash
cd ~/Predective_Bike_ws
colcon build --symlink-install
source install/setup.bash
```

## PC-side workflow (no hardware attached)

Terminal 1 -- start synthetic sensors:
```bash
ros2 launch mock_sensors mock_sensors.launch.py
```

Terminal 2 -- verify data is flowing:
```bash
ros2 topic hz /imu/data      # should read ~50 Hz
ros2 topic hz /gps/fix       # should read ~1 Hz
ros2 topic echo /gps/speed
```

Terminal 3 -- record a bag for later replay/regression testing:
```bash
ros2 bag record /imu/data /imu/fall_event /gps/fix /gps/speed -o mock_bag_01
```

Later, replay that bag while developing sensor_fusion_node/risk_assessment_node
without needing the mock publishers (or the Pi) running at all:
```bash
ros2 bag play mock_bag_01
```

## Pi-side workflow (real hardware)

1. Complete the one-time Pi configuration in [docs/RASPBERRY_PI_SETUP.md](docs/RASPBERRY_PI_SETUP.md), including the HDMI display mode, I2C, UART, and serial permissions.
2. Build and launch the three connected components:
   ```bash
   cd ~/Predective_Bike_ws
   source ~/ros2_jazzy/install/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ./run_bike_dashboard.sh
   ```
3. In another terminal, verify the data:
   ```bash
   ros2 topic echo /gps/fix
   ros2 topic echo /radar/state
   ```

## Notes

- Topic names and message types in `mock_sensors` intentionally match what the
  real driver nodes publish, so `sensor_fusion_node` and `risk_assessment_node`
  (to be added next) can be developed against either source interchangeably.
- `bike_msgs` has no hardware dependency and can be built/tested entirely on
  your PC from day one.
- The IMU and camera packages remain intentionally unimplemented because no
  corresponding hardware has been specified. They are not started by the dashboard launch file.
