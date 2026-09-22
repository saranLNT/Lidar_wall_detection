# wall_scan_to_pointcloud

Converts `sensor_msgs/msg/LaserScan` from `/wall_scan` into `sensor_msgs/msg/PointCloud2`.

Default behavior:
- Input: `/wall_scan`
- Output: `/wall_scan_cloud`
- Output frame: `laser`

Launch:
- `ros2 launch wall_scan_to_pointcloud wall_scan_to_pointcloud.launch.py`
