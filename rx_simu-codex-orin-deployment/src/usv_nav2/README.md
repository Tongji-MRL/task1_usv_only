# 无人船 Nav2 Humble 接入包

适用环境为 Ubuntu 22.04、ROS2 Humble、ArduPilot Rover 和双电机差速无人船。船体尺寸按 1.2 m × 0.7 m 配置。本包采用 NavfnPlanner 全局规划、DWBLocalPlanner 局部规划，并将处理好的 LaserScan 接入标准 costmap。

ROS1 的 move_base 和 DWAPlannerROS 不直接用于此环境。这里保留相同的标准导航架构，使用 ROS2 Nav2 对应组件。DWB 显式选用 LimitedAccelGenerator，按照当前速度和加速度范围生成动态窗口候选速度。

本包是可在目标 ROS2 环境联调的初始实现；本次本地验证覆盖纯 Python 逻辑、参数关联和文件语法，尚未在 ROS2、ArduPilot SITL 或实船上完成运行验证。Windows 上通过这些检查不等于已经通过 ROS2 启动测试。

**数据接口**

| 输入或输出 | 类型 | 用途 |
|---|---|---|
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | 飞控融合后的位置、姿态和机体系速度 |
| `/usv/odom` | `nav_msgs/msg/Odometry` | 校验后以 Reliable QoS 原样转发给 Nav2 的里程计 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 全局、局部代价地图的实时障碍物输入 |
| `/mavros/state` | `mavros_msgs/msg/State` | 控制桥检查连接、解锁和 GUIDED 状态 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | map_server 发布的水域边界与固定障碍底图 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 米制地图目标点接口 |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | 米制地图路径点接口 |
| `/usv/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 输出，x 为前向速度、z 为偏航角速度 |
| `/usv/setpoint_preview` | `mavros_msgs/msg/PositionTarget` | 经过数据检查后的控制预览 |
| `/usv/control_status` | `std_msgs/msg/String` | 控制输入是否齐全、是否超时 |
| `/mavros/setpoint_raw/local` | `mavros_msgs/msg/PositionTarget` | 启用控制后发送给 MAVROS 的 Rover 指令 |

默认 `enable_control:=false`，仅发布 Nav2 指令和控制预览。控制桥不会自动解锁或切换飞控模式，也不修改飞控参数。预览中的实际指令同样经过状态检查；未解锁或未进入 GUIDED 时预览为零，未经过状态门控的规划输出仍可查看 `/usv/cmd_vel`。

**坐标系约定**

第一版直接采用飞控已经融合好的本地 ENU 状态。全局和局部 costmap 都使用 `map`；局部地图仍是随船移动的滚动窗口。这里没有额外制造一个声称连续的 `odom` 坐标系。

`odom_tf` 读取 MAVROS 里程计，要求 `header.frame_id=map`、`child_frame_id=base_link`，发布：

```text
map → base_stabilized → base_link → 雷达实际 frame_id
```

`map → base_stabilized` 保留船体位置和 yaw；`base_stabilized → base_link` 补回 roll、pitch。两条变换组合后等于飞控的完整姿态。`base_stabilized` 与船体同原点、Z 轴竖直，可供点云投影使用。TF 沿用传感器消息时间戳，不使用“现在时间”掩盖过期数据。

`base_link` 原点假定在 1.2 m × 0.7 m 船体矩形的中心。如果飞控估计的机体参考点偏离船体中心，必须先确认位置参考点、GNSS 杆臂补偿和雷达外参，并相应移动 footprint 顶点，不能直接套用中心对称外形。

MAVROS ROS2 的里程计使用 Best Effort QoS，Humble Nav2 的里程计订阅使用 SystemDefaultsQoS。为兼容默认请求 Reliable 的 DDS 配置，`odom_tf` 以 Best Effort 接收、以 Reliable 发布 `/usv/odom`，保留原始消息的坐标、时间戳、速度和协方差。它不补造消息，也不提高定位精度。Nav2 使用 `/usv/odom`；控制桥仍独立检查原始 MAVROS 输入。

地图原点、朝向和飞控 ENU 原点必须一致。当前 `odom_tf` 不做经纬度转换、不做地图配准，也不做额外 EKF 融合。它会拒绝帧名不匹配的消息，不会通过改名假装坐标已对齐。已有 TF 发布器时，设置 `publish_odom_tf:=false`，并确保提供等价变换；此开关只关闭 TF，里程计转发仍保留。不要让 MAVROS、FAST-LIO 或其他节点同时发布同一船体 TF。

如果飞控重新初始化或修正导致本地坐标跳变，地图关系也会受影响，需要中止当前任务、检查原点并重新对齐。后续若加入真实连续里程计，可再分离 `map → odom → base_link`。

**安装与构建**

将本目录复制到 Ubuntu 工作空间的 `src/usv_nav2`，在已安装 ROS2 Humble、已初始化 rosdep 的环境中执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/usv_ws
rosdep install --from-paths src/usv_nav2 --ignore-src -r -y
colcon build --symlink-install --packages-select usv_nav2
source install/setup.bash
```

MAVROS、雷达驱动和实际串口连接由现有硬件启动流程负责。本包不会启动第二个 MAVROS 实例。先确认这些输入：

```bash
ros2 topic info /mavros/local_position/odom -v
ros2 topic echo /mavros/local_position/odom --once --qos-reliability best_effort
ros2 topic hz /mavros/local_position/odom
ros2 topic info /scan -v
ros2 topic hz /scan
```

**雷达接入**

已有可靠的 `/scan` 时，直接使用，不启动本包的点云转换节点。扫描消息要有正确的时间戳、`frame_id`、角度范围和距离范围，并且其坐标系能够变换到 `map`。

只有三维点云时，先检查消息类型：

```bash
ros2 topic type /livox/lidar
```

`pointcloud_to_laserscan` 要求 `sensor_msgs/msg/PointCloud2`。如果得到 `livox_ros_driver2/msg/CustomMsg`，应调整驱动输出模式或增加消息转换节点；重映射话题名称不会改变消息类型。

测量雷达相对 `base_link` 的位置和姿态，使用 `sensor_tf.launch.py` 发布安装外参。这个启动文件要求显式提供 `x`、`y`、`z`、`roll`、`pitch`、`yaw`、`lidar_frame`，没有假定安装位置。已有 URDF 或静态 TF 时不再重复发布。

启动导航之后，可在另一终端启动可选转换器：

```bash
ros2 launch usv_nav2 cloud_to_scan.launch.py cloud_topic:=/livox/lidar
```

默认将点云变换到 `base_stabilized` 再投影：这能使用整帧时间戳的姿态补偿横摇和俯仰，但不等于逐点运动去畸变。默认高度范围为船体参考点下方 0.5 m 到上方 1.5 m，距离 0.3–20 m，扫描周期假设 0.1 s；这些是待根据实际安装高度和点云频率调整的初值。还需要过滤船体自身、浪花和其他无效回波。转换器的角度元数据由标准节点生成，未复用自写 DWA 的固定点数角度公式。

`inf_is_valid=false`，不把缺失回波自动作为整条射线空闲的证据。动态层依据后续有效射线清除旧障碍；`observation_persistence=0` 不是障碍格子的自动过期机制。测试中要观察移动目标离开后的残留格子，必要时再增加经过验证的障碍物时效处理。

**水域地图**

`maps/demo_area.json` 和 `demo_water.yaml/pgm` 是人工构造的 40 m × 40 m 测试水域，包含一个矩形障碍。它们用于软件联调，不代表真实比赛场地。

真实地图可仿照 `demo_area.json`：将已测量的可航行边界和禁航多边形填为 `map` 坐标系中的米制坐标，生成 PGM 和地图 YAML。多边形需简单、无自交；生成后应在 RViz 核对边界方向、比例和船的位置。

```bash
ros2 run usv_nav2 make_water_map /path/to/your_area.json /path/to/maps/water
```

输出 `water.yaml`、`water.pgm`。允许航行多边形内填为空闲，多边形外与 exclusions 填为占用。默认拒绝覆盖已有地图；需要重生成时显式传入 `--overwrite`。本工具不推断水深、岸线，也不把雷达未探测到的区域自动宣布为可航行。

两个 costmap 都加载底图；实时障碍层以最大值叠加，使传感器的清除操作不会覆盖底图中的禁航区。全局规划禁止进入未知区域。目标点必须在底图覆盖范围内，坐标单位为米。

**启动导航和检查路径**

真实输入与地图已经对齐后：

```bash
ros2 launch usv_nav2 navigation.launch.py map:=/path/to/maps/water.yaml
```

`map` 是必填参数，不会默认使用示例水域。若仅做软件联调，可以明确选用示例：

```bash
ros2 launch usv_nav2 navigation.launch.py \
  map:=$(ros2 pkg prefix --share usv_nav2)/maps/demo_water.yaml
```

启动后用 `ros2 topic info /usv/odom -v` 检查 Nav2 订阅和 Reliable 发布者是否匹配，用 `ros2 topic hz /usv/odom` 确认转发正常。默认的原始里程计输入和转发输出是两个不同话题，不能设置为同一个话题。

在 RViz 中检查 `map`、扫描、TF、船体外形和局部代价地图，然后使用 Nav2 Goal 工具指定目标。GNSS 定位不使用 AMCL，不需要用 2D Pose Estimate 人为初始化位置。也可以通过 action 发送米制目标，例如下面的 `(10, 0)` 仅对应示例水域：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
ros2 topic echo /usv/cmd_vel
ros2 topic echo /usv/control_status
```

启动文件也接受 `odom_topic`（原始输入）、`nav_odom_topic`（转发输出）、`scan_topic`、`params_file`、`publish_odom_tf`、`use_sim_time`、`rviz`。使用 rosbag 仿真时将 `use_sim_time:=true`，并提供对应 `/clock`；点云转换器也要设置相同参数。正常实船运行保持 false。

本包配置了定期重规划和路径跟踪。规划失败时任务失败退出，不自动执行倒船、旋转或清空障碍地图等恢复行为。DWB 在完成最终航向时仍可能输出原地转向，适用于已确认支持该动作的双电机差速船。

**连接 Rover 控制输出**

在 SITL 或实机联调阶段，确认飞控速度控制与左右转向方向正确后，可通过同一启动文件的 `enable_control:=true` 启用实际输出：

```bash
ros2 launch usv_nav2 navigation.launch.py \
  map:=/path/to/maps/water.yaml enable_control:=true
```

控制桥仅在飞控已连接、已解锁、处于 GUIDED，且指令、扫描和里程计均有效时转发非零速度。模式切换后要求收到新指令，不重放解锁前缓存。指令、里程计或扫描超过 0.5 s、飞控状态超过 2 s 未更新时输出零速度和零转向速率。此检查只验证消息有效性及新鲜度，不代表已经验证 RTK 固定解或 EKF 估计质量。飞控自身失联保护仍应保留。

接口使用 `PositionTarget.FRAME_BODY_NED`，掩码 1511（速度＋yaw rate），提供 m/s 和 rad/s，不把速度填入归一化 thrust。Python 端保持 ROS 的正向前进、逆时针正转；MAVROS 负责一次 FLU/ENU 到 FRD/NED 的符号转换。最终应在所用 MAVROS 和 Rover 固件版本上验证正向、左转、右转、停止和断流响应。

**初始参数和适用范围**

| 参数 | 初值 | 说明 |
|---|---:|---|
| footprint | 1.2 m × 0.7 m 矩形 | 原点假定船体中心 |
| footprint_padding | 0.1 m | 外形额外余量；不能替代定位误差评估 |
| 最大前向速度 | 0.4 m/s | 低速联调初值 |
| 最大转向速率 | 0.4 rad/s | 低速联调初值 |
| 前向加速／减速 | 0.2 m/s² | 必须按实测能力调整 |
| 转向加速／减速 | 0.4 rad/s² | 必须按实测能力调整 |
| 控制周期 | 10 Hz | 与 DWB sim_period=0.1 s 一致 |
| 预测时间 | 3 s | 与运动能力和有效观测范围一起调整 |
| 局部地图 | 30 m × 30 m，0.1 m/格 | 地图覆盖范围不代表雷达探测保证 |
| 障碍物观测范围 | 15 m | 转换器默认量程 20 m，实际取决于雷达与目标 |
| 到点位置／航向容差 | 0.5 m / 0.25 rad | 判定阈值，不是实测精度 |

参数集中在 `config/nav2_params.yaml`。文件使用 JSON 形式的合法 YAML，启动器也接受普通 YAML。修改速度上限时同时更新 DWB 与 rover_bridge；修改船体参考点时同时更新两个 costmap 的 footprint。

DWB 使用平面速度模型，没有建模水流、侧滑和推进器动态。当前不生成横向速度或倒车指令。需要以实测停车距离、转向过程和路径跟踪误差来调整参数；这份配置还不是经过水上验证的运动模型，也不提供持续定点保持功能。

**本地检查**

在本包目录执行 `python3 -m unittest discover -s test -v` 可运行无需 ROS 的测试，覆盖坐标旋转分解、消息时间检查、控制字段和超时、里程计转发回调、地图轴方向、禁航边界以及规划器与控制桥参数一致性。回调测试使用消息替身，不代表 ROS2 或 DDS 测试。ROS2 生命周期、DDS QoS、Nav2 插件加载、真实雷达 TF、Rover 固件响应需要在 Ubuntu 目标环境完成验证。

实现依据：[Nav2 Humble 配置](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)、[DWB 动态窗口生成器](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_plugins/src/limited_accel_generator.cpp)、[Rover Guided 指令](https://ardupilot.org/dev/docs/mavlink-rover-commands.html)、[MAVROS ROS2 坐标转换](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/setpoint_raw.cpp)、[MAVROS 里程计 QoS](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/local_position.cpp)、[Humble Nav2 里程计 QoS](https://github.com/ros-navigation/navigation2/blob/humble/nav2_util/src/odometry_utils.cpp)。
