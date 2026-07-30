# radar_node -- not yet implemented

Bring this up last (most complex driver: SPI/UART + TI mmWave config profile).

Planned entry point: `radar_driver_node.py`
Publishes: /radar/targets (bike_msgs/RadarTarget[] wrapped in a custom array msg,
or one RadarTarget per detected object on a topic array)
