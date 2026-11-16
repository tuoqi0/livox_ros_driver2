# Livox ROS 2 驱动

本目录是 `slam_ws` 的 Livox ROS 2 Jazzy 驱动。它只保留 ROS 2 主线，直接发布 `sensor_msgs/PointCloud2` 或 `livox_ros_driver2/msg/CustomMsg`，不再构建 ROS 1、PCL 或 Boost 兼容分支。

驱动只接收在线雷达数据（`data_src=0`），不直接读取 LVX/LVX2 录制文件。ROS 2 bag 使用 `ros2 bag play` 回放；LVX2 文件在完整工作区构建并加载 Keel 环境后，用 `ros2 run keel_lio lvx2_to_bag` 转换；Jazzy Python 运行依赖、单位和回放步骤见 `src/keel_lio/tools/README.md`。

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

本包只提供驱动节点和设备 JSON，不再安装独立的 ROS 1/旧版 launch 目录。
建图或定位时使用 Keel 的统一入口，它会自动启动一个驱动进程：

```bash
ros2 launch keel_lio map.launch.py config:=mid360s.yaml
ros2 launch keel_lio nav.launch.py config:=mid360s.yaml \
  prior_pcd_file:=/data/maps/site/map.pcd
```

需要单独检查驱动时，直接运行安装后的节点，并传入设备 JSON：

```bash
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 \
  -p publish_freq:=10.0 -p frame_id:=livox_frame \
  -p user_config_path:="$(ros2 pkg prefix livox_ros_driver2)/share/livox_ros_driver2/config/MID360_config.json"
```

`user_config_path` 指向 `config/` 下的设备 JSON。修改 JSON 前确认设备 IP、主机网卡和
`pcl_data_type` 与实际雷达一致；这里的 `pcl_data_type` 是 Livox SDK 的线缆数据编码，
不是 PCL 库依赖。实机使用 `map/nav` 时不要再单独启动上面的节点。

驱动参数只有 ROS 2 发布所需的主线参数：

| 参数 | 含义 | 常用值 |
| --- | --- | --- |
| `xfer_format` | 点云消息类型 | `0`=`PointCloud2`，`1`=`CustomMsg` |
| `multi_topic` | 是否按设备拆分话题 | `0` 共用，`1` 每台一个 |
| `data_src` | 数据源 | `0` 原始雷达 |
| `publish_freq` | 发布频率（Hz） | `10.0` |
| `frame_id` | 点云和 IMU 的坐标系 | `livox_frame` |
| `user_config_path` | Livox JSON 配置 | 设备对应 JSON |

设备筛选以当前 JSON 的 `lidar_configs[].ip` 为准。未配置设备的点云和 IMU 在驱动收包
入口丢弃并记录节流诊断，设备发现回调也不会为其注册发布设备或下发启动命令。
初始化失败后再次加载配置，会先清理上次的设备索引、发布队列与外参，再按新 JSON 建立
设备；这不是运行中热加载。旧版 `cmdline_input_bd_code` 参数已经移除。

Keel 的 `map/nav` 使用 `multi_topic=0`，配置中应只选一台雷达。多台明确列入 JSON 的
设备仍会共用话题；其他驱动进程的发布也不受本进程筛选影响。

`xfer_format` 取其他值会在节点初始化时记录错误并拒绝启动，避免进入无法发布的循环。

## 输出话题和字段

单雷达模式下默认发布：

- `/livox/lidar`：`PointCloud2` 或 `livox_ros_driver2/msg/CustomMsg`
- `/livox/imu`：`sensor_msgs/Imu`

`PointCloud2` 的每点字段为 `x`、`y`、`z`、`intensity`、`tag`、`line` 和 `timestamp`。`timestamp` 是点的绝对纳秒时间以 `float64` 存储；`CustomMsg` 使用 `timebase` 加 `offset_time` 表示同一时间信息。

驱动入口会丢弃空点云、未知数据类型、长度不足或载荷越界的 SDK 包，并记录节流诊断。IMU 包按其协议长度单独校验。坏包不会进入下游定位。

## 测试
驱动通过 Keel 的实际 ROS 输入链路验证；本包不再维护与生产链路重复的纯 SDK 单测。

## 与 SLAM 的连接

Keel LIO 使用 `/livox/lidar` 和 `/livox/imu`。映射和导航启动文件会按需要自动启动本驱动；实机运行时不要再启动第二个 Livox 驱动进程，否则同一话题会混入两路数据，定位结果不可解释。
