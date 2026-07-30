# camera_node -- not yet implemented

Bring this up after imu_node and gps_node are working end-to-end.

Planned entry point: `camera_capture_node.py`
Publishes: /camera/image_raw (sensor_msgs/Image), /camera/camera_info
Then extend into vision_ai_node for YOLO inference once the raw
image stream is validated at target FPS.
