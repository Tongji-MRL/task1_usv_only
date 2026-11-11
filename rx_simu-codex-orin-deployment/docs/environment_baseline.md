# 服务器环境基线

调查日期：2026-10-29，主机 `ailad-mini-server`，工作目录 `/home/zjy/usv_nav`。以下区分实测安装状态与计划采用的版本；可用内存、磁盘及 apt 候选版本会随环境变化。

## 已有环境

| 项目 | 调查结果 |
|---|---|
| 系统 | Ubuntu 22.04.5 LTS，Linux 6.8 |
| CPU / 内存 | i9-14900K，32 个逻辑 CPU，约 62 GiB 内存 |
| GPU | RTX 4090，24 GiB，驱动 535.247.01 |
| 磁盘余量 | 调查时 `/home` 约 293 GB，shared SSD 约 482 GB |
| 开发工具 | Python 3.10.12、GCC 11.4、CMake 3.22、colcon |
| ROS | Humble，安装于 `/opt/ros/humble`，当前 shell 需显式 source |
| Nav2 | 1.1.20；Navfn、DWB、LimitedAccelGenerator、ObstacleFootprint 可用 |
| Gazebo | Garden 7.9.0 与 Harmonic 8.11.0 共存 |
| ROS Gazebo 桥 | 已安装 `ros-humble-ros-gzgarden-*` 0.244.11，链接 Garden 的 transport 12 / msgs 9 |
| 飞控工具 | pymavlink 2.4.49、MAVProxy 1.8.74 |
| 显示环境 | 调查终端无 `DISPLAY`；需验证无窗口 GPU 渲染和 LiDAR 输出 |

现有 VRX 构建使用 Garden，因此首轮统一使用 Garden，避免混用两个 Gazebo 版本的插件和消息库。

## 既有 ArduRover

- 路径：`/home/zjy/ardurover-usv`，指向 `/mnt/shared_ssd/zjy/home/ardurover-usv`。
- 当前分支：`Rover-4.4`；调查时工作树干净。
- 当前 HEAD：`5db4ee7de915a98747b05a5ee461c6b28bb2f82a`，包含额外急停继电器改动。
- **要求的提交：`d0356316eb0fa34ed1c9f39f7b059b9ae7ba4664`，本地已有该对象，但当前检出并非该提交。** 该提交的版本文件标识 ArduRover V4.4.0。
- 已有 Pixhawk6C 硬件构建，没有发现 SITL 可执行文件；子模块已初始化。

联调时使用独立 checkout/clone 固定到要求的提交，同时固定其子模块。保留原目录分支和硬件构建，不以版本字符串相同替代提交校验。

## 既有 VRX

- 路径：`/home/zjy/vrx_docking`，指向 `/mnt/shared_ssd/zjy/home/vrx_docking`。
- 分支：`safe-docking-dev`；调查提交：`fd6a6d36f3fa25a3c34b3ee3e89987f24d31b7fe`。
- 存在未跟踪的实验文件；此提交号不能代表全部本地实验资产。复用资产时还需记录文件及校验值。
- 已有 `vrx_gz`、`vrx_gazebo`、`wamv_description` 的构建和安装，CMake 使用 `gz-sim7`。
- 已有 WAM-V 船体、浮力/水动力、风浪、码头、障碍和传感器资产。WAM-V 船体约 5 m，与实船 1.2 m × 0.7 m 不同。
- 既有 docking 场景包含竞赛阶段解锁逻辑；新场景应独立生成并明确船体锁定状态。

## 实施前待补齐

| 项目 | 调查状态 / 拟采用值 |
|---|---|
| MAVROS、MAVROS extras | 未安装；调查时 apt 候选 2.16.0 |
| pointcloud_to_laserscan | 未安装；调查时 apt 候选 2.0.1 |
| GeographicLib 数据 | 尚未发现所需 geoid 数据；安装 MAVROS 时核实 |
| rapidjson-dev | 未安装，Gazebo ArduPilot 插件构建需要 |
| python3-venv | 未安装 |
| ArduPilot Gazebo 插件 | 拟采用官方提交 `80dc9664dda7efd6c5d1a71a79198d1d6b708c0e` 并维护 4.4 协议兼容补丁；尚未构建联调 |

此前对 MAVROS、extras、pointcloud_to_laserscan、python3-venv 的 apt 模拟安装显示新增 13 个包，无升级或移除；不包括后续确认的全部插件构建依赖。本次创建仓库未安装这些依赖。

## 源码导入

`usv_nav2_humble.zip` 共 24 个文件，29,037 字节，CRC 通过。SHA256：

```text
559c826089a07f8a9cf86d7c70ebd7db80cb65b8753741febb8e879af6fb07df
```

源码原样导入 `src/usv_nav2`，交接文档原样保存于 `docs/navigation_handoff.md`。逐文件校验值见 [import_manifest.json](import_manifest.json)。该清单描述导入基线，后续源码修改由 Git 差异追踪。
