# Livox ROS 2 驱动

本目录是 `slam_ws` 的 Livox ROS 2 Jazzy 驱动。它只保留 ROS 2 主线，直接发布 `sensor_msgs/PointCloud2` 或 `livox_interfaces2/CustomMsg`，不再构建 ROS 1、PCL 或 Boost 兼容分支。

## 编译

在 Jazzy 容器或同等 ROS 2 环境中，从工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to livox_ros_driver2 \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

只重新编译驱动时：

```bash
colcon build --base-paths src --packages-select livox_ros_driver2 \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 启动与配置

驱动启动文件位于 `launch_ROS2/`，按设备选择对应文件：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

启动文件中的 `user_config_path` 指向 `config/` 下的设备 JSON。修改 JSON 前确认设备 IP、主机网卡和 `pcl_data_type` 与实际雷达一致；这里的 `pcl_data_type` 是 Livox SDK 的线缆数据编码，不是 PCL 库依赖。

驱动参数只有 ROS 2 发布所需的主线参数：

| 参数 | 含义 | 常用值 |
| --- | --- | --- |
| `xfer_format` | 点云消息类型 | `0`=`PointCloud2`，`1`=`CustomMsg` |
| `multi_topic` | 是否按设备拆分话题 | `0` 共用，`1` 每台一个 |
| `data_src` | 数据源 | `0` 原始雷达 |
| `publish_freq` | 发布频率（Hz） | `10.0` |
| `frame_id` | 点云和 IMU 的坐标系 | `livox_frame` |
| `user_config_path` | Livox JSON 配置 | 设备对应 JSON |
| `cmdline_input_bd_code` | 设备绑定码 | 与现场配置一致 |

`xfer_format` 取其他值会在节点初始化时记录错误并拒绝启动，避免进入无法发布的循环。

## 输出话题和字段

单雷达模式下默认发布：

- `/livox/lidar`：`PointCloud2` 或 `livox_interfaces2/CustomMsg`
- `/livox/imu`：`sensor_msgs/Imu`

`PointCloud2` 的每点字段为 `x`、`y`、`z`、`intensity`、`tag`、`line` 和 `timestamp`。`timestamp` 是点的绝对纳秒时间以 `float64` 存储；`CustomMsg` 使用 `timebase` 加 `offset_time` 表示同一时间信息。

驱动入口会丢弃空点云、未知数据类型、长度不足或载荷越界的 SDK 包，并记录节流诊断。IMU 包按其协议长度单独校验。坏包不会进入下游定位。

## 测试

构建测试并运行驱动行为回归：

```bash
colcon build --base-paths src --packages-select livox_ros_driver2 \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
cd build/livox_ros_driver2
ctest --output-on-failure -R \
  'test_imu_extrinsic|test_packet|test_queue_exit|test_ros_messages|test_startup'
```

测试覆盖 SDK 包边界、队列退出、点云/IMU 消息编码、外参发布和启动配置；其中 `test_ros_messages` 还验证非法点云格式会消费队列后退出，不会让发布线程空转。

## 与 SLAM 的连接

Keel LIO 使用 `/livox/lidar` 和 `/livox/imu`。映射和导航启动文件会按需要自动启动本驱动；实机运行时不要再启动第二个 Livox 驱动进程，否则同一话题会混入两路数据，定位结果不可解释。
