# car_lidar_nav2

ROS 2 package for pure LiDAR localization and Nav2-based navigation.

This package integrates:

- `lidar_loc`: custom LiDAR localization node
- `lidar_filter_node`: simple LaserScan outlier filtering node
- `costmap_cleaner`: costmap clear helper node
- Nav2 launch, map, and parameter configuration

## Features

- Static-map navigation based on 2D LiDAR
- Custom LiDAR localization without AMCL
- Integrated Nav2 bringup launch
- Included demo map and navigation parameters
- Optional LiDAR filtering and costmap clearing utilities

## Package Structure

```text
car_lidar_nav2/
├── config/
│   └── nav2_params.yaml
├── launch/
│   ├── navigation2.launch.py
│   ├── lidar_loc_test.launch.py
│   └── amcl_test.launch.py
├── maps/
│   ├── room.yaml
│   └── room.pgm
├── src/
│   ├── lidar_loc.cpp
│   ├── lidar_filter_node.cpp
│   └── costmap_cleaner.cpp
├── CMakeLists.txt
├── package.xml
└── LICENSE
```

## Dependencies

This package depends on:

- ROS 2
- `ament_cmake`
- `rclcpp`
- `rclpy`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`
- `std_msgs`
- `std_srvs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `cv_bridge`
- OpenCV
- `nav2_bringup`
- `nav2_map_server`
- `nav2_lifecycle_manager`
- `rviz2`

## Build

In your ROS 2 workspace:

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select car_lidar_nav2
source install/setup.bash
```

## Run

Launch the localization and navigation stack:

```bash
ros2 launch car_lidar_nav2 navigation2.launch.py
```

Optional arguments:

```bash
ros2 launch car_lidar_nav2 navigation2.launch.py use_sim_time:=false map:=/absolute/path/to/map.yaml
```

## Main Nodes

### `lidar_loc`

Custom localization node that:

- subscribes to `map`
- subscribes to LaserScan data
- estimates robot pose in the map
- publishes `map -> odom` transform

### `lidar_filter_node`

Simple LaserScan filter that removes isolated outlier beams.

Default parameters:

- `source_topic`: `/scan`
- `pub_topic`: `/scan_filtered`
- `outlier_threshold`: `0.1`

### `costmap_cleaner`

Clears costmaps after receiving an initial pose.

## Configuration

### Map

Default map:

- [room.yaml](C:/Users/HaiChen/Desktop/car_lidar_nav2/maps/room.yaml)
- [room.pgm](C:/Users/HaiChen/Desktop/car_lidar_nav2/maps/room.pgm)

### Nav2 Parameters

Main Nav2 config file:

- [nav2_params.yaml](C:/Users/HaiChen/Desktop/car_lidar_nav2/config/nav2_params.yaml)

### Launch

Main launch file:

- [navigation2.launch.py](C:/Users/HaiChen/Desktop/car_lidar_nav2/launch/navigation2.launch.py)

## Common Debug Commands

Check TF:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

Check topics:

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /initialpose --once
```

Check Nav2 lifecycle:

```bash
ros2 lifecycle nodes
ros2 lifecycle get /lifecycle_manager_navigation
```

## Notes

- The package currently uses a custom localization implementation rather than `nav2_amcl`.
- Localization quality depends heavily on correct TF, odometry quality, laser extrinsics, and map accuracy.
- If localization drifts badly, first verify `base_link`, `laser`, and `odom` frame consistency.

## License

This repository includes a `LICENSE` file. See [LICENSE](C:/Users/HaiChen/Desktop/car_lidar_nav2/LICENSE).
