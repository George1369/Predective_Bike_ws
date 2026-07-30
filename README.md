# bike_ws -- Smart Bicycle ADAS ROS 2 Workspace

Targets ROS 2 Lyrical Luth on Ubuntu 26.04 (Pi) and should be built with the
same distro/Ubuntu combo on your PC to avoid version skew.

## Packages

| Package        | Status                | Purpose |
|----------------|------------------------|---------|
| `bike_msgs`    | Ready                  | Custom message types (Detection2D, RadarTarget, TrackedObject, RiskState) |
| `imu_node`     | Stub hardware read     | Phase 1: real IMU driver (fill in `_read_hardware()`) |
| `gps_node`     | Stub hardware read     | Phase 1: real GPS driver (fill in `_read_hardware()`) |
| `camera_node`  | Placeholder only       | Phase 1c/2: bring up after IMU + GPS are validated |
| `radar_node`   | Placeholder only       | Phase 1c/2: bring up last (most complex driver) |
| `mock_sensors` | Ready                  | Synthetic IMU/GPS publishers for PC-only development |

## Build

```bash
cd ~/bike_ws
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

## Pi-side workflow (real hardware, one sensor at a time)

1. Fill in `_read_hardware()` in `imu_node/imu_node/imu_driver_node.py` for your
   actual IMU part (I2C address, register map).
2. Build and run just that node:
   ```bash
   colcon build --packages-select imu_node bike_msgs --symlink-install
   source install/setup.bash
   ros2 run imu_node imu_driver_node
   ```
3. In another terminal, confirm real data on `/imu/data` at the expected rate
   and sanity-check values against known reference (e.g. resting = ~9.81 m/s^2 on Z).
4. Record a short rosbag2 of real data, pull it to your PC, and compare its
   shape/noise characteristics against the mock data -- adjust the mock
   publisher's noise/motion model if it's unrealistic.
5. Repeat steps 1-4 for `gps_node`, then move on to `camera_node`, then
   `radar_node`, per the phased roadmap.

## Notes

- Topic names and message types in `mock_sensors` intentionally match what the
  real driver nodes publish, so `sensor_fusion_node` and `risk_assessment_node`
  (to be added next) can be developed against either source interchangeably.
- `bike_msgs` has no hardware dependency and can be built/tested entirely on
  your PC from day one.
