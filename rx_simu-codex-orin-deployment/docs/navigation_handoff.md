# navigation 项目交接：无人船 Nav2 + ArduPilot Rover

交接日期：2026-10-29（Asia/Shanghai）

目标项目：`navigation`，远程主机 `zjy-miniserver`，项目目录 `/home/zjy/usv_nav`。

## 1. 接手结论

用户希望采用标准导航栈：飞控提供融合定位，雷达提供障碍物观测，规划器生成路径及速度指令，ArduPilot Rover 执行底层控制。

用户最初指定 ROS1 的 `move_base + DWAPlannerROS`；确认实际环境为 **Ubuntu 22.04 + ROS2 Humble** 后，方案已改为 **Nav2 + NavfnPlanner + DWBLocalPlanner**。保留标准 costmap 和 `/scan` 接入方式，不移植原仓库自写的 `dwa_avoid/obs` 转换逻辑。

目前已生成独立 ROS2 包 `usv_nav2`，完成 Windows 上的单元测试及静态检查。**尚未在 Ubuntu ROS2、ArduPilot SITL 或实船上运行验证，也没有部署到远程项目。** 下一阶段应先在目标环境检查、构建，再完成 SITL 闭环联调。

## 2. 用户已确认的信息与待确认项

| 项目 | 已确认内容 |
|---|---|
| ROS 环境 | ROS2 Humble，Ubuntu 22.04 |
| 飞控软件 | ArduPilot Rover |
| 船体外形 | 长 1.2 m × 宽 0.7 m |
| 推进方式 | 双电机差速 |
| 定位方向 | 使用飞控融合后的位置估计，可由 RTK-GNSS 支持 |
| 障碍物输入 | 希望采用标准 `/scan → costmap` |
| 雷达处理 | 用户表示“好像可以跑通 pointcloud_to_laserscan”，尚未核实 |
| 当前联调关注点 | 在 SITL 中模拟 RTK 定位 |

接手时从环境和配置读取以下信息，已有信息不要重复询问：

- ArduPilot Rover 固件版本、SITL 启动方式和船体模型；MAVROS 版本与连接端口。
- 远程目录当前有哪些代码、是否为 colcon 工作空间、ROS2 是否实际安装。项目名称和路径已由应用查询确认，目录内容尚未检查。
- 雷达型号、消息类型、话题名、频率、视场、有效量程、安装外参。
- 飞控本地坐标原点、航向基准、机体位置参考点、GNSS 杆臂设置。
- 实测速度、加减速度、停车距离和转向能力；真实场地边界与禁航区。

当前没有给出用户其他业务代码的具体接口；本包尚未接入任务决策、感知识别或比赛任务状态机。

## 3. 已有文件及移交方式

以下文件位于 **Windows 本机**，远程 Linux 项目不会自动看到这些路径。

本地输出目录：

```text
C:\Users\admin\Documents\Codex\2026-10-15\https-gitee-com-control-engineering-45\outputs
```

| 文件 | 内容 |
|---|---|
| `navigation_handoff.md` | 本交接文件 |
| `usv_nav2_humble.zip` | 完整 ROS2 包，ZIP 内顶层目录为 `usv_nav2/` |
| `usv_nav2/README.md` | 安装、坐标系、地图、雷达及控制接口说明 |
| `usv_nav2/config/nav2_params.yaml` | Nav2、代价地图及适配节点参数 |
| `usv_nav2/launch/navigation.launch.py` | 主启动文件 |
| `usv_nav2/launch/cloud_to_scan.launch.py` | 可选 PointCloud2 转 LaserScan |
| `usv_nav2/launch/sensor_tf.launch.py` | 根据实测值发布雷达安装外参 |
| `usv_nav2/usv_nav2/odom_tf.py` | 里程计 QoS 转发和可选 TF 发布 |
| `usv_nav2/usv_nav2/rover_bridge.py` | Nav2 Twist 到 Rover MAVROS 指令 |
| `usv_nav2/usv_nav2/core.py` | 几何、时间戳和指令有效性逻辑 |
| `usv_nav2/usv_nav2/make_water_map.py` | 米制可航行多边形生成 PGM/YAML 地图 |
| `usv_nav2/behavior_trees/` | 单目标和多途经点导航行为树 |
| `usv_nav2/maps/` | 人工测试水域及输入多边形 |
| `usv_nav2/rviz/navigation.rviz` | RViz 显示配置 |
| `usv_nav2/test/` | 不依赖 ROS 运行环境的单元测试 |

本次交接核验：ZIP 共 24 个文件，29,037 字节，CRC 校验通过；ZIP 中每个文件均与当前源码一致。

```text
usv_nav2_humble.zip SHA256:
559c826089a07f8a9cf86d7c70ebd7db80cb65b8753741febb8e879af6fb07df
```

将本文件和 ZIP 一起移交到远程项目。先检查 `/home/zjy/usv_nav`，若它确实作为 colcon 工作空间，可将包放入 `/home/zjy/usv_nav/src/usv_nav2`。若已经存在同名包，先比较差异再合并，不覆盖已有修改。此目录布局是建议，尚未执行文件传输或解压。

原参考仓库：[ros-unmanned-ship V1.10](https://gitee.com/control_engineering_45/ros-unmanned-ship/tree/V1.10/src/dwa_avoid)。本地实际仓库根目录为：

```text
C:\Users\admin\Desktop\PhD_Tongji\RobotX2026\ros-unmanned-ship-V1.10\ros-unmanned-ship-V1.10
```

原仓库包含 `navigation`、`ros_navigation`、`dwa_avoid` 等不同实现，没有名为 `planning` 的包。已有标准导航代码属于 ROS1，部分流程依赖 FAST-LIO、点云投影和地图发布。此次生成的是独立 ROS2 接入包；原仓库未修改。

## 4. 当前数据流和接口

```text
RTK/GNSS + IMU 等 → Rover EKF → MAVROS /mavros/local_position/odom
                                  ├─ odom_tf → /usv/odom → Nav2
                                  └─ rover_bridge 检查原始定位数据

PointCloud2 → 过滤船体/无效回波 → pointcloud_to_laserscan → /scan
                                                          ↓
水域底图 /map → global_costmap、local_costmap → Navfn + DWB
                                                          ↓
                                                   /usv/cmd_vel
                                                          ↓
                                                    rover_bridge
                                                          ↓
                                             /mavros/setpoint_raw/local
                                                          ↓
                                              Rover 速度/转向控制
```

点云过滤、雷达驱动、MAVROS 实例和仿真世界不由本包启动。上述图中的过滤环节仍需根据真实数据配置。

| 接口 | 类型 | 说明 |
|---|---|---|
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | 飞控融合状态，预期父帧 `map`、子帧 `base_link` |
| `/usv/odom` | `nav_msgs/msg/Odometry` | 校验后原样转发，提供 Reliable QoS 给 Nav2 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 两个 costmap 的障碍观测输入 |
| `/mavros/state` | `mavros_msgs/msg/State` | 连接、解锁和模式状态 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 给定米制终点及终点朝向，Nav2 自行规划 |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | 给定一组米制途经位姿 |
| `/usv/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x` 前向速度，`angular.z` 偏航角速度 |
| `/usv/setpoint_preview` | `mavros_msgs/msg/PositionTarget` | 经过状态检查的输出预览 |
| `/usv/control_status` | `std_msgs/msg/String` | 检查结果与超时原因 |
| `/mavros/setpoint_raw/local` | `mavros_msgs/msg/PositionTarget` | 启用控制后发送给 MAVROS |

### 坐标与定位

- 规划使用米制笛卡尔坐标。经纬度目标需要先转到与定位一致的 `map`，当前包没有实现这项转换。
- 初版全局与局部 costmap 均使用 `map`，局部 costmap 是滚动窗口。
- TF 为 `map → base_stabilized → base_link → lidar_frame`。前段保留 xyz 和 yaw，后段恢复 roll/pitch，组合后等于原始飞控姿态。
- TF 使用消息原始时间戳；不通过更新时间戳或重命名帧掩盖数据延迟、坐标不一致。
- 地图与飞控本地 ENU 坐标必须配准。EKF 原点重置、跳变处理、地理 datum 配准目前均未实现。
- `base_link` 暂按船体矩形中心解释。实际参考点偏移时，需要同步修正 footprint 和传感器安装关系。
- 不要求 FAST-LIO 或 AMCL。若以后加入独立连续里程计，再考虑真实的 `map → odom → base_link` 分层。
- MAVROS 里程计以 Best Effort 接收，适配器以 Reliable 发布，兼容 Humble Nav2 的系统默认订阅 QoS；这不会提高定位精度。关闭 `publish_odom_tf` 后仍保留里程计转发。

### 雷达与地图

- 已有合格 `/scan` 时直接接入。`pointcloud_to_laserscan` 的输入必须是 `PointCloud2`；Livox `CustomMsg` 不能靠改话题名变成 PointCloud2。
- 可选转换器默认投影到 `base_stabilized`，高度范围 -0.5～1.5 m、距离 0.3～20 m、周期 0.1 s，均为待核实初值。整帧姿态补偿不等于逐点去畸变。
- 本包选择水域边界和禁航区静态底图，加实时障碍层；不要求先用 LiDAR SLAM 建全局地图。雷达也不能证明所有未观测区域都可航行。
- 示例地图是人工 40 × 40 m 可航行区域，含一个矩形障碍；加边缘 padding 后栅格尺寸为 210 × 210、分辨率 0.2 m。它不代表真实场地。
- 两个 costmap 均有 static、obstacle、inflation 三层。障碍层以最大值叠加；全局规划 `allow_unknown=false`。
- `inf_is_valid=false`；`observation_persistence=0` 不代表障碍格自动过期。移动目标离开后的残留障碍需要专项验证。

### 控制出口

- 默认 `enable_control=false`，不创建向飞控发送 setpoint 的发布者；不自动解锁、不切模式。
- 使用 `FRAME_BODY_NED=8`、掩码 `1511`，有效控制字段为 vx、vy 和 yaw_rate；vy 固定为 0。单位为 m/s、rad/s。
- ROS 端保持前向为正、逆时针转向为正。MAVROS 做一次 FLU/ENU 到 FRD/NED 转换，桥接节点不再手动反转 yaw 符号。
- 非零输出要求 connected、armed、GUIDED，且指令、里程计、扫描、状态均新鲜有效。指令/定位/扫描超时为 0.5 s，状态超时为 2 s。
- 看门狗使用单调时钟检查接收间隔，同时检查传感器消息源时间戳。模式资格变化后要求新指令。
- 预览也经过上述门控；未解锁或不在 GUIDED 时预览为零。查看未门控的规划速度应使用 `/usv/cmd_vel`。
- **目前没有 RTK Fixed、EKF 健康或定位协方差门控。** GPS 丢失但飞控仍发布新鲜预测状态时，当前桥不一定停船。进程退出/崩溃后的响应还依赖飞控自身保护。

## 5. 当前初始参数

这些参数用于初步低速联调，不是经过水上辨识的参数或精度保证。

| 参数 | 当前值 |
|---|---|
| 全局规划插件 | `nav2_navfn_planner/NavfnPlanner`，Humble 名称 |
| 局部规划插件 | `dwb_core::DWBLocalPlanner` |
| 轨迹生成器 | `dwb_plugins::LimitedAccelGenerator` |
| 碰撞评价 | `ObstacleFootprint`，使用矩形外形 |
| footprint | `[[0.6,0.35],[0.6,-0.35],[-0.6,-0.35],[-0.6,0.35]]` m |
| footprint padding | 0.1 m |
| inflation radius | 1.0 m，代价膨胀范围，不是保证的净空距离 |
| 最大前向速度 / 偏航角速度 | 0.4 m/s / 0.4 rad/s |
| 前向加速、减速幅值 | 0.2 m/s² |
| 转向加速、减速幅值 | 0.4 rad/s² |
| 横向速度 / 倒车 | 禁用 |
| 控制频率 / sim_period | 10 Hz / 0.1 s |
| 轨迹预测时长 | 3 s |
| 局部地图 | 30 × 30 m，0.1 m/格 |
| 障碍标记 / 射线清除范围 | 15 m / 20 m |
| 到点位置 / 航向容差 | 0.5 m / 0.25 rad |

船体尺寸、障碍观测范围、膨胀范围和到点容差是不同概念，不能统称为“避障半径”或“定位精度”。速度上限修改时同步更新 DWB 和控制桥。

行为树执行定期重规划与路径跟踪，失败时退出任务，没有自动倒船、清图或旋转恢复。DWB 最终航向对齐仍可能原地转向。当前平面模型没有显式建模水流、侧滑、推进器滞后，也没有持续定点保持功能。

## 6. 已完成验证与尚未完成的验证

| 验证项 | 状态与范围 |
|---|---|
| 单元测试 | 已运行并通过 20 项：几何分解、时间戳、字段掩码、指令门控、超时、地图方向、参数关联、里程计回调 |
| Python 语法 | 12 个 Python 文件通过 Python 3.10 语法解析；执行检查的本机 Python 为 3.12 |
| 文件检查 | XML、JSON 形式 YAML 参数及示例地图检查通过 |
| ZIP | 本次再次核验 CRC 和源码一致性通过 |
| ROS2 构建和节点启动 | 未验证 |
| DDS QoS 实际匹配、TF 同步、Nav2 生命周期/插件加载 | 未验证 |
| Rover SITL 控制响应、RTK 模拟、雷达仿真、避障闭环 | 未验证 |
| 实船定位精度、停车距离、转向和抗流能力 | 未验证 |

回调测试使用消息替身，不是 ROS2 集成测试。参数文件采用 JSON 形式的合法 YAML，启动器也可读取普通 YAML。

复现纯逻辑测试（在源码包目录中）：

```bash
python3 -m unittest discover -s test -v
```

## 7. SITL 模拟 RTK：已经回答的结论

SITL 可以模拟 GNSS 测量、RTK Fixed/Float 状态，以及误差和延迟。推荐继续采用“仿真运动 → 模拟传感器 → Rover EKF → MAVROS → Nav2”的链路，保留飞控定位接口。

已经查询的当前 ArduPilot 源码中，u-blox 仿真后端支持以下参数；**用户的固件版本和后端尚未确认，不能直接假定全部支持。**

| 参数示例 | 含义 |
|---|---|
| `SIM_GPS1_FIXTYPE=6` | 报告 RTK Fixed；5 为 Float，3 为普通 3D Fix |
| `SIM_GPS1_ACC=0.02` | 报告 2 cm 精度指标，并不自动给坐标加入 2 cm 误差 |
| `SIM_GPS1_HNSE=0.02` | 给水平位置加入半径 2 cm 的缓慢环形漂移，不是白噪声标准差 |
| `SIM_GPS1_LAG_MS` | 模拟 GPS 延迟，单位 ms |

先在实际 SITL MAVProxy 中查看 `param show SIM_GPS*`，按固件版本和 GPS 后端核实后再设参数。旧版本名称不同，某些后端不使用相同的状态/误差配置；必须检查实际遥测结果。

设置 Fixed 状态、报告精度和注入误差是三件不同的事。此仿真不需要真实基站，也不能替代对实际 RTCM/NTRIP 链路、载波相位解算、天线多路径或实船定位精度的验证。

如果要求“失去 RTK Fixed 自动减速/停止”，下一阶段要增加明确的 GPS/EKF 质量门控和恢复策略，再验证 Float、普通定位、失锁、恢复等状态。当前包尚无此功能。

## 8. 建议的接续顺序与验收结果

### A. 检查远程环境，接收代码并构建

先读取目标项目自身的 `AGENTS.md`、现有源码和启动配置，核实 Ubuntu、ROS2、MAVROS、ArduPilot/SITL 版本。保持现有工程布局；新增包不能覆盖其他人的工作。

若已将包放入该目录的 `src/usv_nav2`，且 rosdep 已初始化：

```bash
source /opt/ros/humble/setup.bash
cd /home/zjy/usv_nav
rosdep install --from-paths src/usv_nav2 --ignore-src -r -y
colcon build --symlink-install --packages-select usv_nav2
source install/setup.bash
```

验收结果：依赖可解析、包可构建、三个 Python 入口及启动文件可发现。发现 Humble 兼容性问题时在本包修复，并记录实际版本与错误。

### B. 接通 SITL 定位与仿真障碍观测

- 明确使用的船体/差速模型及其局限；目前尚未创建船体仿真模型、SITL 参数文件或自动启动脚本。
- MAVROS 接收正常定位状态，验证帧名、ENU 朝向、时间戳、速度方向和 `/usv/odom` QoS。
- 选择现有仿真环境的 LiDAR 或生成与船体位姿、障碍物世界一致的 LaserScan。GPS 仿真本身不提供验证 costmap 所需的 `/scan`。
- 配准仿真世界、GPS 本地坐标、底图和目标点。记录仿真原点。
- 明确时钟方案。使用 `use_sim_time=true` 时必须有一致的 `/clock`；不能只因运行 SITL 就假定已存在 ROS 仿真时钟。

验收结果：RViz 中船体、障碍物、底图和目标坐标一致；转弯时障碍物不会随船体错误移动。

### C. 默认关闭控制，先验证规划

传感器和 TF 已就绪后，可显式使用人工地图启动软件联调：

```bash
ros2 launch usv_nav2 navigation.launch.py \
  map:=$(ros2 pkg prefix --share usv_nav2)/maps/demo_water.yaml \
  enable_control:=false rviz:=false
```

有图形环境时可启用 RViz。`map` 参数必填；该例中场景与定位也必须配准到示例坐标。

验收结果：生命周期节点激活、目标 action 可用、全局路径绕开静态和扫描障碍、矩形 footprint 生效、产生合理 `/usv/cmd_vel`，飞控输出仍关闭。

### D. 在纯 SITL 中验证完整控制链

确认连接对象确实为 SITL 后，再用 `enable_control=true` 完成闭环测试。目标接收、模式设置与解锁流程按仿真环境实施；本包不会自动操作这些状态。真实船只测试应另行安排。

验收至少包含：前进、左右转向符号、停止、到点、静态障碍绕行、动态观测清除、不可达目标、取消任务、断开指令/扫描/里程计后的零输出，以及桥接进程退出后的飞控响应。

对每个场景记录实际话题、参数、结果和日志。通过软件闭环不代表船体水动力模型或实船参数已验证。

### E. 增加定位质量测试

先核实模拟 RTK 状态与误差参数确实生效，再分别测试高精度输入、位置误差、延迟、Float/失锁和恢复。对新增质量门控单独验证，不用“消息仍在发布”替代定位健康判定。

## 9. 可直接给接手助手的任务说明

> 请接手 navigation 项目中的无人船导航联调。用户已确认 Ubuntu 22.04、ROS2 Humble、ArduPilot Rover、1.2 × 0.7 m 双电机差速船，采用 Nav2 的 Navfn + DWB，使用飞控融合里程计和雷达 `/scan`。已有 `usv_nav2_humble.zip` 初始实现及本交接文件，20 项本地测试通过，但 ROS2/SITL/实船尚未运行验证。先检查 `/home/zjy/usv_nav` 的现状和项目指令，确认代码包是否已传入；存在同名代码时先比较，再完成构建、SITL 定位/雷达输入、规划和控制闭环。请核实 ArduPilot 实际版本后配置模拟 RTK，特别区分 Fixed 状态、报告精度和注入误差。当前控制桥尚无 RTK/EKF 质量门控。优先完成可复现的纯 SITL 测试，准确记录已验证与未验证的部分。

## 10. 参考来源与范围备注

- [Nav2 Humble 配置](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)
- [DWB LimitedAccelGenerator](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_plugins/src/limited_accel_generator.cpp)
- [Rover Guided MAVLink 指令](https://ardupilot.org/dev/docs/mavlink-rover-commands.html)
- [MAVROS ROS2 setpoint_raw 坐标处理](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/setpoint_raw.cpp)
- [MAVROS ROS2 里程计与 QoS](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/local_position.cpp)
- [Nav2 Humble 里程计订阅](https://github.com/ros-navigation/navigation2/blob/humble/nav2_util/src/odometry_utils.cpp)
- [ArduPilot SITL 参数使用](https://ardupilot.org/dev/docs/SITL_simulation_parameters.html)
- [ArduPilot 模拟 GPS 参数与误差实现](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_GPS.cpp)
- [ArduPilot 模拟 u-blox RTK 状态](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_GPS_UBLOX.cpp)

参考源码链接中的分支会变化，实际联调时记录所用版本或提交。不能将当前主分支的参数直接当作用户固件的参数。

当前聊天移动到远程 navigation 项目的尝试未成功；项目归属和代码传输均未完成，本文件用于手动交接。

聊天末尾还插入过独立的相机连接板 CAD 任务：已完成只读预检，用户同意在当前装配新增独立改版并保留原件，但在建模前中断，尚未修改或保存 CAD。它应在 CAD 工作中另行接续，不属于本次 navigation 联调范围。
