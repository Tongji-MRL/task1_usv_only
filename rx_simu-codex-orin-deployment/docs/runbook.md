# VRX / ArduRover / Nav2 运行说明

## 依赖和版本

基于本机 `/opt/ros/humble` 与 `/home/zjy/vrx_docking/install`。ROS、Gazebo GPU 驱动和既有 VRX 的环境调查见 `environment_baseline.md`。`config/dependencies.lock.json` 记录固件、插件、VRX SHA 和新增 deb 的版本与 SHA256。VRX 资产依赖既有安装；本仓库不包含其完整源码，也不是任意干净 Ubuntu 的一键安装器。

`python3 scripts/bootstrap.py` 在 `external/` 下载/解包依赖、建立独立源码检出、应用兼容补丁并构建。已有原工程用作 Git 对象和子模块来源，不更改其工作树。`ARDUPILOT_SOURCE` 可覆盖本地来源；`VRX_ROOT` 可覆盖 VRX 路径。版本不匹配时中止，不自动升级飞控。启动器还检查固件工作树和递归子模块干净、插件工作树精确等于已跟踪补丁，以及 VRX 提交匹配；各次运行另记实际二进制 SHA256。

新增 MAVROS 2.16.0、GeographicLib egm96-5、pointcloud_to_laserscan 及构建依赖解包到 `external/deps`，不修改系统 apt 数据库。下载包若在仓库镜像下架，需要保留的 `external/debs` 缓存或同 SHA256 包。`scripts/env.sh` 只影响当前 shell，并选择 Garden、NVIDIA EGL、ROS domain 42 和 Gazebo partition `usv_nav_validation`。

## 控制和坐标链

```
Nav2 /usv/cmd_vel → rover_bridge → MAVROS → ArduRover 4.4.0
  → JSON PWM → ArduPilotPlugin → VRX 左右推进器（牛顿）
VRX 专用 IMU + GNSS → ArduRover EKF → MAVROS odom → /usv/odom + TF
VRX 16 线点云 → 船体过滤 → 水平投影 → 无回波有效性检查 → /scan
```

左右输出分别是 SERVO1/3、功能 73/74。PWM 1000/1500/2000 映射到 -300/0/+300 N；固定转角的双推进器通过差动转向。正 ROS 角速度为逆时针，仅 MAVROS 执行命令符号转换。专用 IMU 位于模型参考点，模型与传感器的 FLU/FRD 转换在生成的 SDF 中明确配置。

`/usv/thrust_left/right` 观测输入的推力命令；`/usv/thruster_left/right_feedback` 来自物理推进器的 `/thrust/ang_vel`。验证器根据当次 SDF 系数还原物理插件使用的力，并记录 `thrust_source` 和反馈新鲜度。该模型直接施加力，尚未描述实船电机滞后。命令、模型推力和船体运动仍分别保存；旧记录只有输入命令时明确标记，不能混作反馈记录。

世界为 ENU，固定地图/GNSS 基准为 `(-33.724223, 150.679736, 0)`，初始艏向东。MAVROS 的飞控局部里程计使用 `odom`；`map_origin` 从飞控 `GPS_GLOBAL_ORIGIN` 计算 `map → odom` 的水平平移，`odom_tf` 发布 `odom → base_stabilized → base_link`，LiDAR 外参由单个静态发布者发布。地图共享 EKF 的垂向基准，仅做水面的二维地理配准。Gazebo 真值只用于传感器生成和验证，不进入导航定位。

MAVROS 源系统号明确设置为 255，飞控 `SYSID_MYGCS=255`，原生地面站失联动作固定为 `FS_ACTION=2`（HOLD），`FS_TIMEOUT=1.5`。MAVROS 2.16 的默认源系统号 1 不符合这组地面站/RC override 配置。GUIDED 速度指令的三秒原生超时和地面站失联保护分别记录。

推进器始终由飞控驱动；ROS/Gazebo bridge 全部为 GZ_TO_ROS。默认控制关闭。`--control` 才允许桥发布真实速度指令，且桥仍要求 GUIDED、正常解锁与新鲜输入。测试脚本只对本机会话执行正常解锁，无 force-arm。

## WAM-V 参数

`scripts/prepare_navigation.py` 从本次生成的碰撞几何创建独立配置，轮廓凸包包含推进器，约 5.37 m × 2.54 m。最终导航速度上限 0.3 m/s、角速度上限 0.4 rad/s，padding 0.5 m、膨胀半径 4.5 m、衰减系数 1.0。DWB 使用 `StandardTrajectoryGenerator`、3 s 预测、0.05 m 局部栅格和 `ObstacleFootprint.scale=1.0`；飞控加减速上限仍为 0.2 m/s²。内部到点容差 0.4 m / 0.2 rad，独立验收仍为 0.5 m / 0.25 rad。

这些参数由实际失败驱动调整：原 0.4 m/s、padding 0.2 m、膨胀 3 m、障碍评分 0.02 的移动障碍试验发生接触，且取消后有约 0.86 m 的惯性位移。降低速度、增大船体裕量和障碍评分后重新进行完整导航回归。膨胀半径本身不是最小净空保证，实际净空必须查看测量结果。原包实船参数没有替换成 WAM-V 参数。

## 启动和测试

先 `source scripts/env.sh`，每次只运行一个本仓库会话。SITL 端口 UDP 9002/14550、TCP 5760/5762 必须空闲。启动器前台监督进程必须存活；目标固件检测父进程退出后会自行退出。

```bash
python3 scripts/sim_session.py start --name baseline --navigation --control
# 在另一个已 source 环境的终端：
python3 scripts/validate.py goal --goals '[[5,0,0]]' --label single
python3 scripts/validate.py goal --goals '[[3,-3,-0.6],[6,-4,0]]' --label multi
python3 scripts/validate_environment.py dynamic_map
python3 scripts/validate_environment.py pause
python3 scripts/validate_faults.py
python3 scripts/sim_session.py stop
```

故障测试会替换/退出桥或 MAVROS；后续测试应重新启动场景。`sim_session.py stop` 仅按本次保存的 PID、进程启动时间和进程组停止工作区进程。

**重置方式：停止会话后重新启动整个会话。** 直接发送 Gazebo `reset all` 已实测造成未来时间戳和 TF/扫描失效，桥保持零输出，但链路不会自动完整恢复。不要把物理世界单独归零当作一次新的有效测试。

自动独立场景批测（先停止手动会话）：

```bash
python3 scripts/run_suite.py --cases single_1 single_2 single_3 multi_1 multi_2 multi_3 static_obstacle cancel unreachable
python3 scripts/run_suite.py --cases wind waves wind_waves gps_input_fixed
python3 scripts/run_suite.py --cases feedback_goal cancel
python3 scripts/run_suite.py --cases dynamic_map dynamic_navigation pause motors protocol disarm collision_sensor mavros_exit sitl_exit fault_inputs
python3 scripts/run_suite.py --cases gnss
python3 scripts/summarize_results.py
```

`reset` 是已知失败的直接 Gazebo 重置诊断，可单独用 `--cases reset` 复现；它不属于推荐重启流程。`collision_sensor` 故意在解锁关闭时使障碍物与船相交，作为接触检测阳性对照，不计入正常航行的无碰撞统计。动态航行夹具在第 3 个仿真秒把 2 m 方形障碍放在 (6, 7)，以 0.14 m/s 沿负 Y 方向运动，第 28 秒移开；每 0.5 仿真秒更新一次位置。这是有运动脚本的刚体障碍，未模拟其他船舶的水动力或避让规则。

风速初值 3 m/s、方向 240°、固定风随机种子 10、随机增益 0。波浪使用 VRX gain 0.3、周期 5 s；gain 是波浪模型参数，不等同于实测显著波高。每轮保留完整 SDF。GNSS 注入随机种子 42、10 Hz：

```bash
python3 scripts/sim_session.py start --name gnss --navigation --control --gps-input
python3 scripts/validate_environment.py gnss
```

`GPS_TYPE=14` 通过 `GPS_INPUT` 注入 fix_type、报告精度、误差和延迟。导航仍使用飞控 EKF。该实验测量现有质量门控缺口，不新增 RTK/EKF 门控策略。

## 结果解释

### 规划、实际航迹、目标位姿与地图可视化

被动记录器会订阅真实 `/plan`、`/local_plan`、`/transformed_global_plan`、`/map` 和全局/局部 costmap，保留消息源时间戳、接收时间及坐标系。它不发送控制指令。运行前先停止已有手动会话：

```bash
source scripts/env.sh
python3 scripts/run_suite.py --cases dynamic_navigation --record-navigation
python3 scripts/plot_navigation.py "$(cat runs/latest.txt)" --animate
python3 scripts/summarize_results.py
```

单目标等其他用例也可加 `--record-navigation`；绘图时用 `--label single_1` 等对应名称。原始路径和地图保存在当次运行的 `visualization/raw/`，输出如下：

- `trajectory_comparison.png`：首次规划、避障阶段规划、障碍移开后重规划、VRX 真值轨迹、飞控 EKF 轨迹，以及末端位置/航向放大图。末端测量取任务完成瞬间，后续漂移另行显示。
- `maps.png`：100 m × 100 m 静态导航地图、最近通过时的全局和局部代价地图，以及 DWB 局部预测。
- `navigation_replay.mp4` / `.gif`：每 1 仿真秒取一帧，6 帧/秒回放；保留任务结束后的漂移观测。每帧仅使用该时刻以前已接收的路径和地图，并标注地图源时间年龄。
- `manifest.json`：输入记录、地图、生成图像和视频的 SHA256、坐标配准说明与结果。`recording.json` 记录订阅消息计数。

Nav2 全局路径是持续重规划的空间参考，DWB 局部轨迹是短时预测；两者都不是整段固定的带时间约束的期望船体轨迹。静态地图是预设可航区域及禁航区，并非 SLAM 建图；LiDAR 障碍及其膨胀区在实时 costmap 中。全程航迹图只叠加静态底图，避免把过去的动态障碍膨胀区误认为整段航程都有效。

2026-10-29 补录示例：[轨迹与末端对照](../runs/20261029_105554_dynamic_navigation/visualization/trajectory_comparison.png)、[地图](../runs/20261029_105554_dynamic_navigation/visualization/maps.png)、[动态回放](../runs/20261029_105554_dynamic_navigation/visualization/navigation_replay.mp4)。此轮无碰撞，位置误差 0.387 m、航向误差 0.167 rad、最小净空 1.847 m；不能与此前录屏轮次的数值混用。

### 测量与判定

`*_result.json` 提供结果，`*.jsonl` 保留独立真值、EKF、指令、推力、时间戳、输入状态和净空。动作状态 4/5/6 分别为成功/取消/中止。测试进程退出 0 只代表记录成功写出，是否达标以 JSON 的 `passed` 为准；失败不会删除或覆盖旧运行。

`runs/suite_results.json` 追加保存尝试，`scripts/summarize_results.py` 生成 Git 管理的 `docs/results.json` 和 `docs/results_summary.md`，并在各导航运行目录生成 `trajectory.png`。`provenance.json` 保存固件、插件、补丁和依赖锁校验值；后期运行还包含 Git 提交、工作树补丁及 `source_snapshot.tar.gz`，因此调试期间的未提交代码也可追溯。原始记录、飞控 DataFlash 日志和视频留在忽略目录，复制项目时需要另外保存这些证据。

验收使用实际动作完成时的真值误差；另记录 10 秒漂移。零指令、推力归零和船速低于阈值是三项不同观测。平面净空使用包含推进器的碰撞轮廓凸包，偏保守；接触传感器用于补充检查。风浪下的三维接触及未建模对象仍需单独评估。

## 调试记录

1. 无 sudo：使用固定版本 deb 在用户目录解包，补齐 MAVROS 运行时所需 `diagnostic_updater` 新共享库及 egm96-5。
2. 指定 4.4.0 JSON 解析器将 `true/false` 解析为整数失败：插件输出 0/1；保持固件源码不变。
3. 插件旧帧计数 uint16 在 65,535 回绕：改为 uint32；添加低频帧计数观测。PWM 0 与失联时显式发送零推力，拒绝截断数据包。
4. SITL CLI 声明 `--rate` 却未处理该选项：去掉该选项，按目标源码支持的接口运行。
5. MAVROS timesync 的整数参数不可传浮点数：`max_rtt_sample` 设为整数 50。
6. 原 1875×16 LiDAR 加 5 Hz 画面产生扫描延迟：保留 16 线，水平 720 点、10 Hz；画面改为 1 Hz，使用 NVIDIA 无窗口渲染。
7. 点云投影无点不能直接当自由空间：只有原始水平射线证实无障碍的方向保留 +Inf，其余空投影为 NaN；桥拒绝全 NaN、过期时间戳或缺失 TF 的扫描。
8. DWB 0.1 m 栅格与 0.02 m/s 初始速度窗口产生量化停滞：局部栅格改 0.05 m；6 s 预测试验导致近障全部轨迹无效，已保留失败并恢复 3 s。冷启动静止 EKF 速度偏差达到约 0.02 m/s，速度死区改为 0.03 m/s。评分日志进一步证实毫米每秒轨迹在栅格边界得分更低，最终改用 `StandardTrajectoryGenerator`，按整个 3 s 时域模拟加减速；飞控仍限制 0.2 m/s²。单目标和多途经点各三个独立冷启动通过。
9. Action endpoint 存在不代表生命周期已激活：测试器等待 bt_navigator active。EKF 开始发布不代表普通解锁已可用：等待正常预解锁检查收敛，不强制解锁。
10. EKF 航向有约 0.02 rad 偏差，内部 0.25 rad 容差曾使真值误差达到 0.263 rad；内部航向容差收紧到 0.20 rad，真值验收标准保持 0.25 rad。位置内部容差 0.40 m，真值验收保持 0.50 m。
11. 风在初始化阶段使船漂移，飞控 EKF 原点随之偏移；飞控 ORGN 日志证实约 0.52 m 的偏差。新增 `map_origin` 使用飞控地理原点配准固定地图，保留所有源时间戳。它不订阅真值，不重设飞控原点。
12. ArduRover 4.4.0 的 `SITL_State.cpp` 会把 Rover 通道 0/2 限制到 1000–2000，故通过 motor test 发送 900/2100 不能测试插件接收越界 PWM。此错误测试保留在历史记录；`protocol` 案例改为停止 FCU 后发送独立无效协议夹具，验证越界 PWM、禁用 PWM、短包和错误 magic 均无推力输出。这些夹具不用于导航或正常推进控制。
13. 原移动障碍试验中，原始扫描和 costmap 都观测到障碍，随后船体仍发生接触，排除“未收到障碍”作为该次碰撞的原因。按实测惯性和碰撞轮廓增加裕量并调高障碍评分，保留失败日志后重测。调参成功不能推导为任意移动障碍均可避让。

轨迹生成器与栅格评分的依据是 Nav2 1.1.20 的 [LimitedAccelGenerator](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_plugins/src/limited_accel_generator.cpp)、[StandardTrajectoryGenerator](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_plugins/src/standard_traj_generator.cpp) 和 [MapGridCritic](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/map_grid.cpp)，以及本机 `runs/dwb_scores.json` 中的实际评分。

`scripts/audit_graph.py` 检查话题 QoS、源时间戳和时钟配置；`build/tf_audit/tf_audit` 用实际 DDS 发布者 GID 检查每条 TF 边是否唯一。MAVROS 会创建通用 TF publisher，即使动态位置 TF 关闭，因此话题上的 publisher 数量不能直接当作每条边的发布者数。MAVROS `/param/get_parameters` 代理飞控参数，不是普通 ROS 节点参数查询；其未设置结果不代表时钟为墙钟。

## 本地检查

```bash
source scripts/env.sh
bash scripts/build.sh
(cd src/usv_nav2 && python3 -m unittest discover -s test -v)
python3 -m unittest discover -s src/usv_sim/test -v
python3 -m compileall -q scripts src
git diff --check
```
