# car_lidar_nav2

这是一个基于 ROS 2 的纯激光导航功能包，集成了自定义激光定位节点与 Nav2 导航配置，可用于 2D LiDAR 静态地图环境下的定位与路径规划。

## 功能简介

本包主要包含以下功能：

- `lidar_loc`：自定义激光定位节点
- `lidar_filter_node`：激光数据离群点滤波节点
- `costmap_cleaner`：代价地图清理辅助节点
- Nav2 启动文件、地图文件和参数配置

适用于：

- 仅依赖 2D LiDAR 的室内导航场景
- 基于静态地图的 ROS 2 导航系统
- 使用 Nav2 进行路径规划与运动控制的机器人平台

## 目录结构

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

## 依赖环境

本功能包依赖以下 ROS 2 组件和系统库：

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


如果缺少依赖，可在工作区中使用 `rosdep` 安装。

## 编译方法

在你的 ROS 2 工作区中执行：

```bash
cd ~/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select car_lidar_nav2
source install/setup.bash
```

如果你使用的不是 `humble`，请将命令中的发行版名称替换成你自己的 ROS 2 版本。

## 启动方式

使用主启动文件运行定位与导航：

```bash
ros2 launch car_lidar_nav2 navigation2.launch.py
```

也可以传入启动参数，例如：

```bash
ros2 launch car_lidar_nav2 navigation2.launch.py use_sim_time:=false map:=/absolute/path/to/map.yaml
```

## 主要节点说明

### 1. `lidar_loc`

自定义激光定位节点，主要功能包括：

- 订阅 `map`
- 订阅激光扫描数据
- 估计机器人在地图中的位姿
- 发布 `map -> odom` 变换

源码位置：

- [src/lidar_loc.cpp](C:/Users/HaiChen/Desktop/car_lidar_nav2/src/lidar_loc.cpp)

### 2. `lidar_filter_node`

用于对 LaserScan 数据进行简单离群点滤波，减少孤立异常点对定位和代价地图的影响。

默认参数：

- `source_topic`: `/scan`
- `pub_topic`: `/scan_filtered`
- `outlier_threshold`: `0.1`

源码位置：

- [src/lidar_filter_node.cpp](C:/Users/HaiChen/Desktop/car_lidar_nav2/src/lidar_filter_node.cpp)

### 3. `costmap_cleaner`

在接收到初始位姿后尝试清理代价地图，适合作为辅助调试节点使用。

源码位置：

- [src/costmap_cleaner.cpp](C:/Users/HaiChen/Desktop/car_lidar_nav2/src/costmap_cleaner.cpp)

## 配置文件说明

### 地图文件

默认地图位于：

- [maps/room.yaml](C:/Users/HaiChen/Desktop/car_lidar_nav2/maps/room.yaml)
- [maps/room.pgm](C:/Users/HaiChen/Desktop/car_lidar_nav2/maps/room.pgm)

### Nav2 参数

Nav2 参数文件位于：

- [config/nav2_params.yaml](C:/Users/HaiChen/Desktop/car_lidar_nav2/config/nav2_params.yaml)

该文件包含：

- 控制器参数
- 全局/局部代价地图参数
- 规划器参数
- 行为树相关参数
- 速度平滑参数

### 启动文件

主启动文件：

- [launch/navigation2.launch.py](C:/Users/HaiChen/Desktop/car_lidar_nav2/launch/navigation2.launch.py)

测试启动文件：

- [launch/lidar_loc_test.launch.py](C:/Users/HaiChen/Desktop/car_lidar_nav2/launch/lidar_loc_test.launch.py)
- [launch/amcl_test.launch.py](C:/Users/HaiChen/Desktop/car_lidar_nav2/launch/amcl_test.launch.py)

## 常用调试命令

### 检查 TF

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

### 检查 Topic

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /initialpose --once
```

### 检查 Nav2 生命周期

```bash
ros2 lifecycle nodes
ros2 lifecycle get /lifecycle_manager_navigation
```

## 使用说明与注意事项

- 当前功能包使用的是自定义激光定位实现，并不是 `nav2_amcl`
- 定位效果高度依赖以下因素：
  - `TF` 是否正确
  - 里程计是否稳定
  - 激光雷达外参是否准确
  - 地图与真实环境是否一致
- 如果定位漂移严重，建议优先检查：
  - `base_link`、`laser`、`odom` 坐标系是否统一
  - 激光话题是否正常
  - 地图原点与分辨率是否正确

## License

本项目许可证见：

- [LICENSE](C:/Users/HaiChen/Desktop/car_lidar_nav2/LICENSE)
