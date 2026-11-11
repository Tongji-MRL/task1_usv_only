# 环境要求与依赖安装

本文主要面向 x86_64 仿真服务器；Orin ARM64 实船只安装 ROS 导航、MAVROS 和真实雷达驱动，见 [实机部署流程](../docs/hardware_deployment.md)。不要在 Orin 复用本仓库锁定的 amd64 deb 或仿真 bootstrap。

实船导航的独立构建入口是 `bash scripts/build_hardware.sh`，支持 Orin ARM64 和用于离线检查的 x86_64；运行节点前使用 `source scripts/hardware_env.sh`。该流程在本仓库 `.venv` 中安装 [hardware-pip.txt](hardware-pip.txt)，使用系统 NumPy / SciPy，并隔离用户目录中的 Python 包。可选 MAVROS/Livox 工作空间必须在 `.env.boat` 中显式填写，不默认任何工作空间已经存在。它只构建实船导航所需三包，产物位于 `build/hardware`、`install/hardware`，不启动硬件或控制。第一次使用请从[从零理解、部署与测试](../docs/hardware_deployment.md)开始，而不是先执行本文件后面的仿真安装命令。

本项目运行 **ROS 2 Humble / Nav2 → MAVROS → ArduRover 4.4.0 SITL → Gazebo Garden / VRX WAM-V**，并包含 RobotX 2026 Task 1 的场景、规则规划、动态交通避让和实验记录工具。

本文按 **2026-11-02 服务器实际环境、构建脚本和依赖导入**整理。当前服务器已完成闭环验证；全新 Ubuntu 从零安装的流程尚未在另一台机器完整复测。`scripts/bootstrap.py` 需要预先存在 ROS、Garden 和已构建的指定 VRX，不能单独完成所有系统安装。这里的依赖包含 C++、ROS、Gazebo、模型和地理数据，不能仅通过 `pip install -r requirements.txt` 安装。

## 1. 平台与资源

| 项目 | 已验证环境 / 要求 |
|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS，x86_64 / amd64；当前锁定的 deb 含 amd64 二进制 |
| Python | 系统 Python 3.10.12；使用 `/usr/bin/python3` 和 ROS Humble 对应的 Python 环境 |
| 编译器 | GCC/G++ 11.4、CMake 3.22.1、C++17、colcon |
| GPU | 服务器使用 RTX 4090 24 GB、NVIDIA 驱动 535.247.01，已验证 EGL 无窗口渲染 |
| CPU / 内存 | 服务器为 i9-14900K、约 62 GiB 内存；尚未测定最低硬件配置 |
| 磁盘 | 源码和报告较小，第三方源码、构建及记录单独占空间；完整点云 rosbag 的实测单轮约 10.25 GB |
| 网络 | 安装阶段需要访问配置的 apt 软件源、GitHub 和 GeographicLib 数据下载源；GitHub 私有依赖需要对应访问权限 |

无窗口运行仍需要可用的渲染设备：相机和 GPU LiDAR 依赖渲染，不能用关闭传感器的场景代替避障验收。当前组合选择 Garden；服务器虽然还装有 Harmonic，运行脚本会指定 Gazebo Sim 7。

## 2. 固定版本

源码提交、额外 deb 的完整版本和 SHA256 以 [dependencies.lock.json](../config/dependencies.lock.json) 为准。系统包及 Python 库的下表数值是已验证环境记录，并非全部已由安装脚本锁定。

| 组件 | 已验证版本 / 固定提交 |
|---|---|
| ArduRover | **V4.4.0，`d0356316eb0fa34ed1c9f39f7b059b9ae7ba4664`**，包含对应递归子模块 |
| ArduPilot Gazebo 插件 | `80dc9664dda7efd6c5d1a71a79198d1d6b708c0e` + [4.4.0 兼容补丁](../patches/ardupilot_gazebo_rover44.patch) |
| VRX | `fd6a6d36f3fa25a3c34b3ee3e89987f24d31b7fe`，使用现有 `vrx_docking` 工程的模型及插件 |
| ROS 2 | Humble，基础安装路径 `/opt/ros/humble` |
| Nav2 | 1.1.20；普通导航使用 Navfn + DWB，Task 1 使用仓库自带规则规划插件 + DWB |
| Gazebo Garden | `gz-sim7` 7.9.0；`gz-transport12` 12.2.2；`gz-msgs9` 9.5.1；`gz-rendering7` 7.5.0；`gz-plugin2` 2.0.4 |
| SDFormat | `python3-sdformat13` 13.9.0，场景生成需要 Python 绑定和 `gz sdf` |
| ROS / Gazebo 桥 | `ros-humble-ros-gzgarden-*` 0.244.11，ROS 包名仍为 `ros_gz_bridge` 等 |
| MAVROS / extras / msgs / libmavconn | 2.16.0，解包在 `external/deps`，由环境脚本加入搜索路径 |
| pointcloud_to_laserscan | 2.0.1，保留 `/scan` 输入；Task 1 还使用点云 VoxelLayer 和交通预测占据层 |
| OMPL | ROS 包 `ros-humble-ompl` 1.7.0；Task 1 的 Dubins 搜索估计需要其 CMake 导出目标 |
| GeographicLib | 1.52；`egm96-5` geoid 数据由 bootstrap 下载并校验 |

启动器核对飞控、递归子模块、插件补丁和 VRX 提交。不应使用升级固件或任意更新 VRX 来跳过版本检查。

可选 MID-360 仿真使用 `livox_gz_sim` 固定提交 `4ca09c3876c2ae60c63f7507578ced57f638e236` 与仓库内 Garden 补丁，执行 `bash scripts/build_mid360.sh` 单独构建。它复用上表的 Humble / Garden 依赖；当前仅输出 PointCloud2，无需安装硬件 Livox 驱动或 SDK，不提供 CustomMsg。完整模型、CSV 和构建结果在 `external/livox_gz_sim`；详见 [模型参数、运行与限制](../docs/loiter_mid360.md)。

补充验证使用独立的 `WaveVisual` 退出修复：`bash scripts/build_render_fix.sh` 从固定 VRX 提交提取三个源文件到 `external/vrx_render_fix`，应用本仓库补丁，生成 `build/vrx_render_fix/libWaveVisual.so`。`scripts/build.sh` 已包含此步骤；`scripts/env.sh` 优先加载修复库，不覆盖 `VRX_ROOT` 的源码或安装。需要 `patch` 命令和已有 `libWaves.so`。MID-360 插件同时禁用自身 ROS 信号处理器，使 Gazebo 正常接收退出信号。

自动等待 LOITER 是显式可选功能，更新后需一起重建消息与消费者：

```bash
source scripts/env.sh
colcon build --symlink-install --base-paths src \
  --packages-select robotx_task1_interfaces usv_nav2 usv_sim robotx_task1 robotx_task1_nav \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
bash scripts/build_mid360.sh
bash scripts/build_render_fix.sh
source scripts/env.sh
```

新增 `MotionPermission.station_action`、`TrafficSafety.hold_clear/hold_envelope_radius`；旧节点必须停止后统一更新，旧接口版本的 rosbag 需要对应版本的消息定义。补充实验流程与通过范围见 [仿真补充验证](../docs/simulation_verification.md)。

## 3. 系统与 Python 依赖

新机器先配置 ROS Humble 和 OSRF apt 软件源。入口可参考 [ROS Humble Ubuntu 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 和 [Gazebo Garden Ubuntu 安装说明](https://gazebosim.org/docs/garden/install_ubuntu/)。下面是本项目使用的主要包，**仅用于尚未配置的新机器**；当前服务器无需重复安装。

```bash
sudo apt update
sudo apt install \
  build-essential cmake git curl wget pkg-config ccache gawk libtool patch \
  python3-dev python3-pip python3-setuptools python3-venv \
  python3-colcon-common-extensions python3-empy python3-pexpect \
  python3-numpy python3-scipy python3-yaml python3-matplotlib \
  python3-opencv python3-pil python3-serial python3-psutil \
  libxml2-dev libxslt1-dev libeigen3-dev libopencv-dev \
  ffmpeg fonts-noto-cjk

sudo apt install \
  ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-ompl ros-humble-xacro \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins \
  ros-humble-ros-gzgarden-interfaces \
  ros-humble-ros-gzgarden-bridge ros-humble-ros-gzgarden-sim \
  gz-garden python3-sdformat13 \
  libgz-sim7-dev libgz-rendering7-dev libgz-plugin2-dev \
  libgz-transport12-dev libgz-msgs9-dev
```

apt 候选版本会变化；安装后需对照版本表及锁文件。选择 `ros-gzgarden` 对应的软件包，避免无意中链接到其他 Gazebo 主版本。固定版本 deb 被镜像下架时，需要原有 `external/debs/` 缓存或具有相同 SHA256 的包，bootstrap 不会自动改用新版本。

Python 依赖实测如下；NumPy、SciPy、OpenCV 等在当前环境来自 Ubuntu 包，避免通过一次全量 `pip upgrade` 改变其与 ROS 的兼容关系。

| Python 模块 | 实测版本 | 用途 |
|---|---|---|
| `numpy` / `scipy` | 1.21.5 / 1.8.0 | 点云处理、船体几何、坐标变换、运动预测与评估 |
| `yaml`（PyYAML） | 5.4.1 | ROS / Nav2 参数生成与读取 |
| `pymavlink` | 2.4.49 | 飞控状态、测试指令和 GNSS 注入 |
| `em`（empy） | 3.3.4 | ROS 接口生成和 ArduPilot 构建 |
| `pexpect` | 4.9.0 | ArduPilot 工具依赖 |
| `future` / `lxml` | 1.0.0 / 6.1.2 | ArduPilot / pymavlink 工具依赖 |
| `matplotlib` | 3.5.1 | 轨迹、误差、地图和回放绘图 |
| `cv2` / `PIL`（Pillow） | 4.5.4 / 9.0.1 | 真实相机录像和中文标注 |
| `setuptools` | 79.0.1 | 当前服务器 Python 包构建工具 |

需要补齐用户目录中的飞控 Python 依赖时，在系统 Python 3.10 环境执行：

```bash
python3 -m pip install --user \
  'pymavlink==2.4.49' 'empy==3.3.4' 'pexpect==4.9.0' \
  'future==1.0.0' 'lxml==6.1.2'
```

`rclpy`、`tf2_ros`、ROS 消息包和 `sdformat13` 由系统 / ROS 安装提供。本项目不通过 PyPI 安装这些组件。MAVProxy 1.8.74 在服务器已有，但当前会话启动器直接运行 `ardurover`，MAVProxy 不是闭环运行的必需进程。MP4 编码依赖 FFmpeg 4.4.2；中文标注脚本使用 `fonts-noto-cjk` 提供的 `/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc`。

## 4. 预先准备两个独立源码目录

| 环境变量 | 本服务器默认值 | 必须满足的条件 |
|---|---|---|
| `ARDUPILOT_SOURCE` | `/home/zjy/ardurover-usv` | 本地 Git 仓库包含指定固件提交，递归子模块源码可用；bootstrap 从这里复制到独立 `external/ardupilot` |
| `VRX_ROOT` | `/home/zjy/vrx_docking` | HEAD 是指定 VRX 提交，已构建，包含 `install/setup.bash` 和 WAM-V 模型资源 |

本服务器直接复用这两个目录作为来源，**不切换其分支、不覆盖其构建，也不启动已有 docking 控制器**。其他机器可指定新的目录，例如：

```bash
export ARDUPILOT_SOURCE="$HOME/ardupilot_nav_source"
export VRX_ROOT="$HOME/vrx_docking_nav"
```

在这些新目录还不存在时，可按以下方式准备来源；VRX 仓库需要有访问权限：

```bash
git clone https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_SOURCE"
git -C "$ARDUPILOT_SOURCE" checkout --detach d0356316eb0fa34ed1c9f39f7b059b9ae7ba4664
git -C "$ARDUPILOT_SOURCE" submodule update --init --recursive

git clone https://github.com/Ericzhaoforu/vrx_docking.git "$VRX_ROOT"
git -C "$VRX_ROOT" checkout --detach fd6a6d36f3fa25a3c34b3ee3e89987f24d31b7fe
(
  cd "$VRX_ROOT"
  source /opt/ros/humble/setup.bash
  export GZ_VERSION=garden
  colcon build --packages-up-to vrx_gz --cmake-clean-cache \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
)
```

检查 `VRX_ROOT/vrx_urdf/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro` 和 `VRX_ROOT/install/setup.bash` 均存在。指定 fork 的源码和资源是独立依赖，克隆 `usv_nav` 不会自动包含它们；不能假定任意版本的上游 VRX 可直接替代。

## 5. 安装本仓库依赖并构建

从仓库根目录运行，使用系统 Python，退出 Conda 等替代 Python 环境：

```bash
python3 scripts/bootstrap.py
source scripts/env.sh
```

bootstrap 会依次：

1. 将锁定的 MAVROS、pointcloud_to_laserscan、GeographicLib、RapidJSON、Asio 等 deb 下载到 `external/debs/`，校验后解包至 `external/deps/`，不更改系统 apt 数据库。
2. 下载并校验 `egm96-5` 数据，设置运行时 geoid 搜索路径。
3. 建立独立的固定版 ArduPilot 检出和子模块，编译 SITL Rover。
4. 检出固定版 Gazebo 插件，应用仓库中的兼容补丁并构建。
5. 构建 `src/` 中的五个 ROS 包，以及 Task 1 Gazebo 插件和 TF 检查工具。

后续只修改本仓库源码时，可运行 `bash scripts/build.sh`。构建前先停止当前仿真，避免覆盖进程已加载的共享库。每个新终端都需要 `source scripts/env.sh`；若使用自定义 `VRX_ROOT`，应先导出该变量。

默认 `ROS_DOMAIN_ID=42`、`GZ_PARTITION=usv_nav_validation`，可通过 `USV_ROS_DOMAIN_ID`、`USV_GZ_PARTITION` 覆盖。SITL 使用本机 UDP 9002 / 14550、TCP 5760 / 5762，一次只启动一个本仓库会话；更换 ROS domain 不会改变飞控端口。

## 6. 安装后的检查

以下检查不会启用控制或解锁飞控：

```bash
source scripts/env.sh
python3 -c 'import rclpy, numpy, scipy, yaml, pymavlink, sdformat13, cv2, PIL, matplotlib'
ros2 pkg prefix mavros
ros2 pkg prefix pointcloud_to_laserscan
ros2 pkg prefix robotx_task1_interfaces
test -f "$GEOGRAPHICLIB_GEOID_PATH/egm96-5.pgm"
test -f /usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc

(cd src/usv_nav2 && python3 -m unittest discover -s test -v)
python3 -m unittest discover -s src/usv_sim/test -v
python3 -m unittest discover -s src/robotx_task1/test -v
ctest --test-dir build/robotx_task1_nav --output-on-failure
git diff --check
```

这些是依赖导入与逻辑检查，不能替代真实闭环验证。先启动控制关闭的场景：

```bash
python3 scripts/sim_session.py start --task1 --navigation --name dependency_preview
```

另一个已加载环境的终端可运行 `python3 scripts/task1_status.py`，结束使用 `python3 scripts/sim_session.py stop`。确认退出并清理完成后，才单独运行完整港区实验：

```bash
python3 scripts/run_task1_suite.py --course harbor --group dynamic \
  --seed 1 --changes 1 --dynamic cross --record-maps --record-sensors
```

这个批测命令会核验本次 SITL，启用导航控制、正常解锁、执行任务并清理。不要与手动预览会话同时运行。详细动作、地图和视频导出见 [Task 1 运行说明](../docs/task1/runbook.md)；普通导航流程见 [运行说明](../docs/runbook.md)。

## 7. 常见依赖问题

| 现象 | 检查与处理 |
|---|---|
| `No module named rclpy` / ROS 消息不可用 | 检查是否使用系统 Python 3.10，重新加载 `scripts/env.sh`，确认未混入 Conda 环境 |
| 找不到 `sdformat13` 或场景生成失败 | 安装 `python3-sdformat13`，核对 Garden / SDFormat 13 组合和 VRX 资源路径 |
| 找不到 `ompl::ompl` | 检查 `ros-humble-ompl` 与 ROS 环境；单独安装其他 OMPL 包不能保证提供同一个 CMake 目标 |
| MAVROS 报 geoid 数据缺失 | 检查 bootstrap 数据校验是否成功，以及 `GEOGRAPHICLIB_GEOID_PATH` 是否指向解包目录 |
| 固定版本 deb 下载失败 | 使用原缓存或同版本、同 SHA256 的包；不能删除锁文件后直接升级代替复现 |
| 相机无图 / LiDAR 无有效回波 | 检查 GPU 驱动、EGL、Gazebo 日志和资源；仅节点启动不代表传感器正常 |
| 中文标注无法加载字体 | 安装 `fonts-noto-cjk` 并检查上面的字体文件路径 |
| 启动提示版本或端口冲突 | 保留指定版本，先停止本仓库旧会话；不修改已有硬件工程来满足检查 |

## 8. GitHub 仓库包含什么

仓库保存源码、ROS 配置、2026 浮标及工作船模型、兼容补丁、测试、版本锁和简洁实验报告。`external/`、`build/`、`install/`、`log/`、`runs/` 以及视频、rosbag 等产物由 [.gitignore](../.gitignore) 排除；不会随普通克隆取得。

历史报告中的 `runs/...` 链接指向服务器本地证据，GitHub 上没有对应大文件。复现实验会生成新的运行目录；若需复核历史视频和原始记录，应另行获取报告中列出的归档并核对 SHA256。已验证结果与尚未解决的限制见 [碰撞修复实验报告](../docs/task1/experiments/20261102_collision_fix.md)。
