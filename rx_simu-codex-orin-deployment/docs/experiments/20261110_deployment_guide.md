# 部署文档与辅助脚本验证：2026-11-10

本次将 [部署指南](../hardware_deployment.md) 改为“代码与算法 → 检查或安装依赖 → 离线检查 → 实测配置 → 数据与规划预览 → 分阶段水试”的流程。没有连接 Orin，没有打开真实雷达或飞控，没有执行模式切换或解锁。以下结果来自 **Ubuntu 22.04.5 / x86_64 server**，不是 ARM64 或实船验收。

## 变更及目的

- 不再默认 `$HOME/robotx_ws` 已存在。MAVROS、Livox 工作空间及独立 SDK 路径由 `.env.boat` 显式选择；apt 安装允许不填写 overlay。
- 导航构建使用系统 Python 3.10、系统 NumPy/SciPy 和仓库 `.venv`，产物放在 `build/hardware`、`install/hardware`；不覆盖原仿真构建。
- `prepare_hardware.py` 检查实测必填值、有效地图路径、凸包、有限数值和低速上限，生成控制关闭的参数文件；保留既有现场配置，不自动猜测船体或安装外参。
- 文档分别解释 Navfn/Dijkstra + DWB 普通导航、Task1 规则搜索与交通预测；明确普通实船路线没有启动比赛规划器或真实 UAV 感知。

## 实际环境和依赖

| 项目 | 本次实际值 |
|---|---|
| 本地仓库 | `/home/zjy/usv_nav`，`codex/orin-deployment` 分支；在 `c8b95b4` 基础上验证本次工作区变更 |
| Python / NumPy / SciPy | 3.10 / 1.21.5 / 1.8.0 |
| pymavlink / setuptools | 2.4.49 / 79.0.1 |
| ROS / Nav2 | Humble / 1.1.20 |
| MAVROS / mavros_msgs | 2.16.0 / 2.16.0，复用 server 既有独立安装 |
| pointcloud_to_laserscan | 2.0.1，复用 server 既有独立安装 |
| Livox SDK2 | `c0796f04c143143899c87a773d9f6b7136453c0b`，本次独立编译 |
| livox_ros_driver2 | `21445540f0d100dc86a7e6df312dd70bbdb4afdf`，包版本 1.0.0，本次独立编译 |
| 飞控约束 | ArduRover 4.4.0 / `d0356316eb0fa34ed1c9f39f7b059b9ae7ba4664`；本次未启动飞控 |

SDK 与驱动版本见 [锁定文件](../../config/hardware_dependencies.lock.json)。本次没有修改系统 apt 包。为在现有 server 上完成检查，使用忽略目录中的环境适配脚本加载已有 MAVROS/点云投影；它不是目标机默认路径。

server 缺少发行版 `python3-venv` 的 ensurepip 和 `python3-rosdep` 安装入口。本次 QA 在独立 `.venv` 内补装 pip/rosdep，并使用用户级 rosdep 缓存。新机指南要求先通过 apt 安装上述依赖。因此下面的构建结果不能描述为“空白操作系统从头安装全部通过”。

## 遇到的问题、修复与回归

**Livox SDK 与驱动不匹配。** 首先尝试 SDK2 `v1.3.1`，SDK 本身构建成功，但固定驱动编译 `pub_handler.cpp` 时缺少 `kLivoxLidarDoubleEchoData` 和 `LivoxLidarDoubleEchoRawPoint`。根因是旧 SDK 头文件没有驱动需要的双回波类型。改为上述固定 SDK 提交，在本次独立目录重建 SDK 和驱动；驱动构建通过，`ldd` 无未解析库。没有通过删代码或关掉相关功能使编译通过。

**server 的 Python 工具准备。** 首次 pip 因继承 SOCKS 代理且隔离了用户目录中的代理支持库而失败；仅对安装子进程去除代理后成功。安装 rosdep 时带入的 setuptools 84.2.1 与现有 colcon 的 `<80` 约束冲突，在 QA venv 中改为 79.0.1 后构建成功。首次 rosdep 更新遇到 SSL 握手超时，保留日志后重试成功。指南使用发行版 rosdep/setuptools，不要求全局升级 Python 库。

## 验证结果

| 检查 | 结果与边界 |
|---|---|
| SDK2 与 Livox ROS 2 驱动独立构建 | 通过；SDK 为独立安装前缀；驱动 `1 package finished` |
| Livox 动态库与包发现 | 通过；节点可执行文件存在，`ldd` 无 `not found`；未启动传感器 |
| `bash scripts/build_hardware.sh` | 通过；rosdep check 通过，三个包构建完成 |
| ROS 消息及 Python 导入 | `MotionPermission`、`RoverBridge`、`LivoxFilter` 导入通过；系统 NumPy/SciPy 数值检查通过 |
| 原导航逻辑测试 | 32 项通过；纯逻辑与消息替身，不代表闭环 |
| 新部署配置测试 | 4 项通过：缺项拒绝、无效值/凸包拒绝、控制关闭且参数一致、已有配置保留 |
| demo 地图生成 | 通过，产生 PGM/YAML；仅离线 demo |
| navigation launch 参数解析 | `--show-args` 通过；没有启动完整 Nav2 生命周期或规划动作 |
| ROS 预览桥真实启动 | 通过，独立 domain 162，未启动 MAVROS或注入传感器数据 |
| 预览输出检查 | 收到 20 条状态、20 条零速度预览；状态为 `PREVIEW: ...`；四个控制/时钟参数为 false |
| 飞控输出隔离 | 桥只有 preview/status、ROS 日志/参数发布者；`/mavros/setpoint_raw/local` 发布者数量为 0 |
| 脚本与文档静态检查 | Bash 语法、ShellCheck、Python 语法、42 个 Bash 文档代码块、JSON、内部章节锚点和相对链接检查通过 |

预览桥使用文档第 6.3 节的 `ros2 run usv_nav2 rover_bridge` 命令启动，检查后仅停止本次创建的进程。没有向 `/usv/cmd_vel`、飞控模式或解锁服务发送控制请求。

可重复运行的基本检查：

```bash
(cd src/usv_nav2 && python3 -m unittest discover -s test -v)
python3 -m unittest discover -s scripts -p 'test_hardware_setup.py' -v
bash -n scripts/build_hardware.sh scripts/hardware_env.sh scripts/hardware_underlay.sh config/real/boat.env.example
shellcheck -x -e SC1091 scripts/build_hardware.sh scripts/hardware_env.sh scripts/hardware_underlay.sh config/real/boat.env.example
git diff --check
```

构建前按部署指南准备依赖和环境。ShellCheck 是文档维护工具，不是运行导航的必要依赖。

## 证据位置和未验证项

本地日志目录：`/home/zjy/usv_nav/runs/deployment_docs_20261110/`；导航构建器记录：`/home/zjy/usv_nav/runs/hardware_build_20261110_155559/`。目录被 Git 忽略，不随 GitHub 克隆下载。

- `livox_driver_build.log`：首次不匹配的失败；`livox_sdk_build_retry.log`、`livox_driver_build_retry.log`：配对后成功。
- `navigation_build.log`、`ros_versions.json`：导航构建和实际依赖版本。
- `navigation_unit_tests.log`、`hardware_setup_tests.log`：逻辑测试结果。
- `preview_smoke.json`、`preview_bridge.log`：真实 ROS 预览启动及图检查。
- `evidence_sha256.json`：本次主要日志校验值。

`navigation_build.log` SHA256：`bc20f22c95670edeae4dbdf978c1338ddb61dbcc1d27e99d30e75f8ed818aceb`；`preview_smoke.json` SHA256：`2dcdbf999a85c2ee512afc0b296d29474fb597b15d51cce8d0106829e46887e9`。

**仍未验证：** 空白 Ubuntu 上完整 apt 安装、Orin ARM64 构建与运行、真实 MID-360 字段/时钟/回波、真实 FCU 固件与定位、测量外参与地图、实船规划/避障/停车、原生 LOITER、水试故障响应。这些必须在目标机器与现场按指南逐层检查，不能由本次 server 离线结果替代。
