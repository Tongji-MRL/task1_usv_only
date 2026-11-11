# Task 1 纯 USV 方案 · Orin 部署与上船验证

更新：2026-11-11。本文只讲 Task 1 纯 USV 方案在 Orin 上的部署与分阶段验证；Orin 基础环境（ROS 2 Humble、MAVROS、MID-360 驱动、飞控连接、地图）沿用 [hardware_deployment.md](../hardware_deployment.md)。方案与任务逻辑见 [usv_only_plan.md](usv_only_plan.md) 与 [usv_only_runbook.md](usv_only_runbook.md)。

## 0. 结论：现在能不能部署

**可以部署到 Orin 做分阶段验证，还不能直接用于比赛。**

| 层级 | 状态 |
|---|---|
| 纯逻辑（规划、建图、灯态、任务顺序、上报内容） | 开发电脑上 153 项离线测试中 148 项通过、1 项跳过，另 4 项是原有仿真节点的测试（缺 ROS 无法导入）；Python 3.10 语法检查通过 |
| 闭环仿真（简化船模、理想传感器、按 handbook 计分） | 见第 9 节；Delta、照片、Alpha 三种场地共 84 个场景全部完成且满分，无错侧 |
| ROS 节点、启动文件、构建脚本 | **从未在 ROS 环境运行过**，第 3 节是第一次验证 |
| 真实 MID-360、D435、飞控控制 | **未做**；识色阈值、聚类高度窗口都没见过真实数据 |
| 岸上中继（protobuf / MQTT） | **未运行过**，需用官方测试服务器验证（第 7 节） |

所以顺序必须是：同步代码 → 构建与无硬件测试 → 录数据测距离 → 只建图不控船 → 预览 → 分级放开控制。任何一层没通过，不进入下一层。

## 1. 同步代码到 Orin

本地这份代码**不在 git 管理下**，Task 1 纯 USV 的改动只存在于这个文件夹。两种同步方式任选其一。

**方式 A：打包拷贝（不需要 git）**。在 Windows 上（PowerShell，仓库上一级目录）：

```bash
tar --exclude=runs --exclude=build --exclude=install --exclude=log --exclude=.venv --exclude=__pycache__ -czf rx_simu_task1.tgz rx_simu-codex-orin-deployment
```

```bash
scp rx_simu_task1.tgz <用户>@<Orin地址>:~/
```

在 Orin 上，先备份已有工作区，再覆盖源码（`.env.boat`、`runs/`、`install/` 等本机文件不在包里，不会被覆盖）：

```bash
cd ~ && cp -a rx_simu rx_simu_backup_$(date +%Y%m%d_%H%M%S)
```

```bash
tar -xzf ~/rx_simu_task1.tgz -C ~ && rsync -a ~/rx_simu-codex-orin-deployment/ ~/rx_simu/ && rm -rf ~/rx_simu-codex-orin-deployment
```

**方式 B：提交到团队分支**。把本文件夹的改动提交到 `codex/orin-deployment` 的新分支并推送，Orin 上 `git fetch` 后切换。这样以后改动可追溯，推荐比赛前改用这种方式。

同步后确认关键文件在：

```bash
cd ~/rx_simu && ls src/robotx_task1/robotx_task1/{mission_core,gap_planner,landmark_map,buoy_mapper,usv_mission,robocommand}.py src/robotx_task1/launch/task1_usv.launch.py scripts/ocs_relay.py
```

## 2. 额外依赖

基础依赖按 hardware_deployment.md 第 3–5 节。Task 1 额外需要：

```bash
sudo apt install --no-upgrade ros-humble-realsense2-camera ros-humble-tf2-msgs ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins
```

可选：要用 `require_rtk_fixed:=true`（RTK 固定解才穿缝）时，需要 MAVROS 的 `gps_status` 插件：

```bash
sudo apt install --no-upgrade ros-humble-mavros-extras
```

并在 `config/real/mavros.yaml` 的 `plugin_allowlist` 里加入 `gps_status`（这个文件目前没改，加之前先在岸上验证 MAVROS 能正常启动）。

## 3. 构建与无硬件验证（Orin 上第一次必做）

```bash
cd ~/rx_simu && source scripts/hardware_underlay.sh && bash scripts/build_hardware.sh
```

```bash
source scripts/hardware_env.sh && (cd src/robotx_task1 && python3 -m unittest discover -s test 2>&1 | tail -15)
```

```bash
export ROS_DOMAIN_ID=162 ROS_LOCALHOST_ONLY=1 && python3 -m unittest discover -s src/robotx_task1/test_ros -p 'test_survey_ros.py' -v
```

```bash
python3 -m unittest discover -s src/robotx_task1/test_ros -p 'test_usv_nodes.py' -v
```

```bash
ros2 launch robotx_task1 task1_usv.launch.py --show-args
```

通过标志：

- colcon 显示 **4 packages finished**；`ros2 pkg executables robotx_task1` 列出 `buoy_mapper`、`usv_mission`、`robocommand`、`survey_check`、`survey_analyze`。
- 第一组离线测试中，本方案的测试全部 OK。`test_review_timing`、`test_station_permission`、`test_traffic_mission`、`test_traffic_queue` 这 4 个是原有仿真任务节点的测试（开发电脑缺 ROS 消息包，从未运行过），本方案不启动这些节点，它们的结果不影响部署；`test_survey_rosbags` 未装 `rosbags` 时显示 skipped 属正常；随机扫查需 1–2 分钟。`test_ros/test_startup.py` 同样属于原有仿真节点，这里不运行。
- `test_survey_ros.py` 2 项 OK（rosbag2 读写与分析、检查节点无传感器时报失败）。
- `test_usv_nodes.py` 6 项 OK：建图节点从点云建出路标和障碍栅格、时间戳等于传感器时间；任务节点无输入或未解锁时输出零、解锁后才开始搜索、里程计或障碍栅格过期立即 HOLD、配置了边界但没有 `map→odom` 时不许运动；上报节点没有定位不发心跳。
- `--show-args` 列出 `enable_control`、`enable_motion`、`color_range`、`boundary` 等参数。

任何一步失败，保存完整输出带回来排查，不要跳过。

## 4. 现场配置

### 4.1 `.env.boat`

在 hardware_deployment.md 第 7 节已填内容基础上，补充相机安装外参（相机机体系 x 前、y 左、z 上，相对 `base_link`，米 / 弧度）：

```bash
export USV_CAMERA_FRAME='camera_link'
export USV_CAMERA_X=''   # 实测
export USV_CAMERA_Y=''
export USV_CAMERA_Z=''
export USV_CAMERA_ROLL=''
export USV_CAMERA_PITCH=''
export USV_CAMERA_YAW=''
```

### 4.2 两个必须实测的数

先按 [usv_only_survey.md](usv_only_survey.md) 录 bag 并分析，得到：

- **可靠识色距离 `C`**（米）：启动参数 `color_range:=C`；观察距离默认取 `C - 0.5`。
- **浮标点云高度范围**：启动参数 `min_z`、`max_z`（相对 `base_link`），以及 `.env.boat` 的 `USV_MARK_MIN_Z/MAX_Z`。

### 4.3 场地边界

`boundary` 是四个字母角浮标在 **map 系**的坐标（东、北，米；原点是 `USV_MAP_LAT/LON`），按场地一圈的顺序填写。获取方法：船停在每个角浮标旁，读 RTK 经纬度，再换算：

```bash
python3 -c "from usv_sim.map_origin import local_xy; import os; print(local_xy(<纬度>, <经度>, float(os.environ['USV_MAP_LAT']), float(os.environ['USV_MAP_LON'])))"
```

四个结果按顺序写成 `[[x1,y1],[x2,y2],[x3,y3],[x4,y4]]`。用途：

- 建图节点只接受边界内、且离角浮标 1 m 以外的目标，角浮标不占 10 个路标名额；
- 规划把边界当成墙，船与边界线保持 0.8 m 以上；
- 找入口阶段沿边界内 2 m 的巡查点绕场。

不填边界也能运行，但船不知道不能出界，也没有巡查路线。

**场内的非任务浮标**（例如 Alpha 场地南侧的两个无灯起点门浮标）用 `markers` 参数给出，格式和换算方法同 `boundary`：`[[x1,y1],[x2,y2]]`。建图节点把它们排除在 10 个路标名额之外（离它们 1 m 内的目标都不算），并作为障碍点随 `/task1/obstacles` 发布，船会绕开它们。不填时，这类浮标会被当成任务浮标，识别为“灭”并占用路标名额。

## 5. 启动顺序

每个终端先执行 `cd ~/rx_simu && source scripts/hardware_env.sh`。按 hardware_deployment.md 第 8 节启动：

| 终端 | 内容 | 说明 |
|---|---|---|
| A | MAVROS | 8-A |
| B | map_origin | 8-B，发布 `map→odom`，边界换算依赖它 |
| C | 雷达安装 TF | 8-C |
| D | MID-360 驱动 | 8-D |
| F | livox_filter | 8-F |
| G | cloud_to_scan | 8-G，rover_bridge 需要 `/scan` |
| **不启动** | **8-E 导航预览（Nav2）** | 速度指令只能有一个发布者；Task 1 的启动文件自带 odom_tf 与 rover_bridge |

再开三个终端：

**H：RealSense 只开彩色流**（参数名随驱动版本不同，先用 `ros2 launch realsense2_camera rs_launch.py --show-args` 核对）：

```bash
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=false enable_infra1:=false enable_infra2:=false rgb_camera.color_profile:=640x480x15
```

按 usv_only_plan.md 5.3 固定低曝光和白平衡，数值记入记录。

**I：相机安装 TF**：

```bash
ros2 run tf2_ros static_transform_publisher --x "${USV_CAMERA_X:?}" --y "${USV_CAMERA_Y:?}" --z "${USV_CAMERA_Z:?}" --roll "${USV_CAMERA_ROLL:?}" --pitch "${USV_CAMERA_PITCH:?}" --yaw "${USV_CAMERA_YAW:?}" --frame-id base_link --child-frame-id "${USV_CAMERA_FRAME:?}"
```

**J：Task 1**（第 6 节按阶段改 `enable_control` / `enable_motion`）：

```bash
ros2 launch robotx_task1 task1_usv.launch.py params_file:="$USV_WORKSPACE/runs/hardware_config/nav2.yaml" color_range:=<C> min_z:=<最低> max_z:=<最高> boundary:='<第4.3节>' markers:='<第4.3节，没有就省略>' ocs_host:=<操作站IP>
```

检查话题与状态：

```bash
ros2 topic echo /task1/mission_status --once
ros2 topic echo /task1/mapper_status --once
ros2 topic echo /task1/landmarks --once
ros2 topic echo /task1/robocommand --once
```

每次上船前先开记录：

```bash
USV_RUN="$USV_WORKSPACE/runs/task1_$(date +%Y%m%d_%H%M%S)" && mkdir -p "$USV_RUN" && ros2 bag record -o "$USV_RUN/bag" /mavros/state /mavros/local_position/odom /mavros/global_position/global /usv/odom /usv/cmd_vel /usv/setpoint_preview /usv/control_status /livox/lidar /usv/lidar/filtered /scan /tf /tf_static /camera/color/image_raw /camera/color/camera_info /task1/landmarks /task1/obstacles /task1/mapper_status /task1/mission_status /task1/robocommand
```

## 6. 分阶段上船

每一阶段都要有人持遥控器，随时切 MANUAL / HOLD。任务节点检测到离开 GUIDED 会立即中止并保持零指令；切回 GUIDED 也不会自行恢复，需重启 J。

| 阶段 | 设置 | 操作 | 通过条件 |
|---|---|---|---|
| 0 岸上静态 | `enable_control:=false enable_motion:=false` | 上电、全部启动，船不解锁 | `mission_status.inputs_ok=true`；`/task1/mapper_status` 有路标；四个角浮标不在路标里；`/usv/control_status` 以 `PREVIEW:` 开头 |
| 1 建图与识色 | 同上 | 遥控器手动开船靠近浮标 | 路标数量、位置与现场一致，相邻浮标未合并；近距离对着浮标 2–4 s 后灯态正确（0 灭、1 闪红、2 闪绿、3 闪蓝、4 常蓝）；靠近 Task 3 平台后 `obstacle_cells>0` |
| 2 上报 | 同上，岸上中继运行 | 同阶段 1 | 测试服务器日志中有 2 Hz Heartbeat 和 SafePassageReport，位置与灯态正确 |
| 3 指令预览 | `enable_control:=false enable_motion:=true` | 调用 `/task1/start`，手动开船 | `/usv/setpoint_preview` 方向合理；没有指令到飞控 |
| 4 找入口与绕圈 | `enable_control:=true enable_motion:=true` | 解锁、切 GUIDED、`/task1/start`；绕完入口后切 MANUAL | 找入口时不进浮标群；顺时针绕满一圈 |
| 5 单个红绿门 | 同上 | 绕入口后继续，穿过一个红绿门后切 MANUAL | 对准横偏 ≤0.15 m、航向 ≤5°；绿在左、红在右；无接触 |
| 6 完整流程 | 同上 | 全程自主 | 识别全部红绿；逆时针绕出口；状态 `COMPLETE` |

启动任务：

```bash
ros2 service call /task1/start std_srvs/srv/Trigger
```

速度参数 `cruise_speed`、`thread_speed`、`max_yaw_rate` 默认 0.2，不得超过 rover_bridge 的上限。

## 7. 岸上操作站中继

规则规定只有操作站能连 RoboCommand，船上的 `robocommand` 节点把内容用 UDP 发给操作站。操作站（不需要 ROS）：

```bash
python3 -m pip install paho-mqtt protobuf && git clone https://github.com/robonation/robocommand
```

先用官方测试服务器验证（`robocommand/RobotX_2026` 下 `docker compose up --build`）：

```bash
python3 scripts/ocs_relay.py --gen robocommand/RobotX_2026/gen/python --team-id <队伍ID> --vehicle-id USV1 --broker localhost --declare-task1 core
```

比赛时 `--broker` 换成场地的 RoboCommand 地址（Alpha 到 Delta 为 `192.168.65.2`–`192.168.68.2`），操作站用 DHCP。若导入生成代码报 protobuf 版本错误，按官方仓库 README 中的版本安装匹配的 `protobuf`。

## 8. 停止与应急

- 立即停止：遥控器切 MANUAL / HOLD，任务节点进入 ABORT，零指令。
- 正常结束：Ctrl-C 终端 J（退出前发零指令），再按 hardware_deployment.md 第 11 节关闭其余部分和记录。
- 不用 `killall`，不 force-arm。

## 9. 已知问题与限制

**闭环仿真结果（2026-11-11 晚，按 handbook Core 计分；修复见 usv_only_runbook.md 第 11 节）**：

| 场地 | 完成 | 满分 | 用时 |
|---|---|---|---|
| Delta 32 / 40 / 43 / 45 / 53 像素/米 | 各 12/12 | 各 12 | 4–19 分钟 |
| 照片场地 | 20/20 | 20 | 5–8 分钟 |
| Alpha（RoboCommand 截图，25 m × 30 m） | 4/4 | 4 | 6–9 分钟 |

满分为 305（Delta、Alpha：3 绿 3 红）和 285（照片场地）。全部场景无错侧、无碰撞。识色距离 3.5 m 时普遍更慢；最慢一个（Delta 32）用了 18.6 分钟，离 20 分钟总时限很近。Alpha 的回放视频在 `runs/task1_alpha_viz/videos/`。仿真是简化模型：没有风流和惯性侧滑，传感器在固定距离内必然看见，验证的是决策逻辑。

**仍需注意**

- Alpha 截图面板写的是“入口 buoy-10、出口 buoy-1”，现场看到的是上方 1 号闪蓝；两种都已仿真通过，但以现场实际为准，船靠识别闪烁/常亮判断，不依赖编号。
- 窄缝里船体离浮标最近约 0.15–0.2 m，对控制精度要求很高，阶段 5 要先单独验证。
- “绕圈时圈外浮标到正横”的判定需向裁判确认。
- 入口浮标在浮标群内部时，绕圈前不得不进入浮标区（会记录 `field_entered_before_entry`）。
- 航向：飞控 EKF 必须用双天线 GPS 航向；`heading_deg` 按真北起顺时针上报，未解锁报 `STATE_KILLED`，二者 proto 均未定义，需确认。
- 进入边界的其他船会在障碍栅格里留下痕迹，确认后不清除。
- 识色阈值、曝光、相机时间戳都没见过真实数据，阶段 1 是第一次真正检验。
