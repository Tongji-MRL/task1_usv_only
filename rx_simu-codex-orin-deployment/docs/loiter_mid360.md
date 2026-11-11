# 原生 LOITER 与 MID-360 仿真

日期：2026-11-04。固件仍为 ArduRover V4.4.0 / `d0356316eb0fa34ed1c9f39f7b059b9ae7ba4664`，ROS 2 Humble、Gazebo Garden 7.9、现有 VRX 保持不变。

本文保留首轮实验的范围和结果（对应 `a86bd7b`）。后续低位安装、孤立回波过滤、自动等待交接与异常退出的实测记录见 [仿真补充验证](simulation_verification.md)；下文的“尚未验证”指首轮状态。

本次增加可复现的原生 LOITER 独立实验，以及 MID-360 点云仿真与预处理入口。Task1 的自动等待模式管理尚未启用：原有任务仍使用 GUIDED 零速度等待，避免把独立定点实验误记为任务避障验收。原有 `--lidar vrx` 是默认值，旧实验结论仍只适用于其传感器配置。

## MID-360 模型与依赖

采用 [livox_gz_sim](https://github.com/mathaimp/livox_gz_sim) 的固定提交 `4ca09c3876c2ae60c63f7507578ced57f638e236`，以及它收录、注明 Livox MIT 许可的 `mid360.csv`。原项目面向 Harmonic / Jazzy，本仓库通过 [Garden 补丁](../patches/livox_gz_sim_garden.patch) 链接 `gz-sim7 / transport12 / msgs9`，只构建 PointCloud2 接口，不依赖真实雷达驱动或 SDK。完整第三方源码、CSV、模型和构建结果均在忽略目录 `external/`，运行记录保留其 SHA256。

```bash
cd /home/zjy/usv_nav
bash scripts/build_mid360.sh
source scripts/env.sh
colcon build --symlink-install --base-paths src --packages-select usv_sim
source scripts/env.sh
python3 scripts/sim_session.py start --name mid360_inspection --lidar mid360 --navigation
```

另一个终端执行：

```bash
cd /home/zjy/usv_nav
source scripts/env.sh
python3 scripts/validate_mid360.py --duration 20
python3 scripts/capture_view.py --seconds 10
python3 scripts/sim_session.py stop
```

此启动命令保持控制关闭。`--task1 --course harbor --lidar mid360` 可生成带 2026 浮标的同一传感器场景；不应把生成成功等同于该雷达配置已经完成 Task1 闭环验收。

| 属性 | 当前实现 |
|---|---|
| 名义视场 | 水平 360°，垂直 −7°～52°；底层范围图留有角度边缘余量 |
| 扫描 | 按 Livox CSV 连续推进，20,000 射线/帧、10 Hz；有效返回点数由场景决定 |
| 时间 | 一帧分 5 个 20 ms 物理采样片段；消息头为首片段时间，每点带原始片段时间 |
| 渲染 | 901 × 151 GPU 范围图；扫描方向读取最近像素，存在角度离散近似 |
| 量程与噪声 | 简化为 0.1～40 m；距离 σ=0.02 m、角度 σ=0.15°；固定种子 42 |
| ROS 接口 | `/livox/lidar`，`sensor_msgs/PointCloud2`，height=1，26 字节/点，XYZ/intensity/tag/line/timestamp |
| 无返回 | 不生成伪点，不用无穷远填充未观测方向 |
| 安装 | 现有 WAM-V 光心位置 `[0.7, -0.3, 1.8]` m；替换外观，沿用已有支架和船体惯性 |

算法是传感器近似，不是标定后的光学模型。40 m 对应简化上限，真实探测距离与反射率等有关；这里没有复现水面反射、多径、雨雾、浪花、真实 tag 分类或单束连续运动。内置 MID-360 IMU 尚未接入；飞控继续使用已有专用 SITL IMU，导航定位继续使用飞控 EKF。

安装高度不能直接照搬。当前光心比 `base_link` 高 1.8 m，若观察同一参考系下高 0.25 m 的低障碍，水平姿态时由下视场估算的近端可见距离约为 `(1.8−0.25)/tan(7°)=12.6 m`；实际还受横摇、纵摇、遮挡和稀疏扫描影响。浮标顶部可能更早可见，但不能据此认为船边低物体也可见。后续必须按实船安装高度、倾角和障碍高度重新验证；本次没有修改安装位置来掩盖盲区。

## 点云处理

`livox_filter` 支持无组织点云，读取实际字段偏移和字节序，不再要求 16 条扫描线。通过完整 TF 外参过滤船体点，使用飞控 EKF 导出的 TF 将点云补偿到原消息起始时刻，按最多 5 ms 一组处理点时间。它不订阅 Gazebo 真值。真实雷达必须先完成与 ROS 时钟的同步；不匹配的点时间、过期帧或缺少 TF 时拒绝输出。

原始点云到达时，最后一组点的 TF 可能尚未到达。节点使用最多 8 帧的等待队列，按仿真源时间和单调墙钟双重执行原有 0.5 s 输入期限；等待期间不重打时间戳，也不输出没有完成补偿的帧。

- `/usv/lidar/filtered`：补偿后的有效外部点，保留原始消息时间；交给现有 `pointcloud_to_laserscan`。
- `/usv/lidar/marking`：在重力对齐坐标中按现有 0.25～3 m 高度范围选取的障碍点。
- `/usv/lidar/clearing`：实际测量射线，按原始点测量时刻分组发布，未进行“当前时间”重打戳；不生成无返回射线。
- `/scan`：来自投影器，并将无观测的空角度设为 NaN。空帧不证明前方安全。

真实雷达的噪声、运动补偿、安装盲区和移动障碍清除仍需用实测 bag 校准。对稀疏扫描保持未知会缩小可确认的通行范围；不得为让任务通过而把 NaN 全改成 +Inf。预处理的真实硬件时间分组是近似，仍需单独验证高角速度和动态障碍。

## 原生 LOITER 实验

使用现有 16 线基线雷达，单独检验模式交接和位置保持，避免将新传感器适配混入同一次对照。

```bash
source scripts/env.sh
python3 scripts/sim_session.py start --name loiter_wind --wind 3 --navigation --control
# 另一个终端，同样 source scripts/env.sh：
python3 scripts/validate_loiter.py --duration 180
python3 scripts/sim_session.py stop
```

测试器只连接本次本机 SITL，要求正常解锁；通过 `/mavros/set_mode` 请求 LOITER，并检查飞控 state 确认。定点期间不发速度指令。保持目标取自飞控 `POSITION_TARGET_GLOBAL_INT`，不把 Gazebo 真值作为控制目标。真值仅用于计算保持误差、净空和接触情况。结束后切回 GUIDED，以新指令检查恢复，再单独执行 HOLD/disarm 清理。

默认记录实际 `LOIT_RADIUS`（固件默认 2 m）、`LOIT_TYPE`、`LOIT_SPEED_GAIN`。`--radius` 可配置本次实验的保持半径；`--moving-entry` 检查运动中进入；`--mode zero_velocity` 保持在 GUIDED 并持续发零，作为独立对照。`--error-budget` 默认 3 m，只是本实验的显式判据，不是比赛通道净空承诺。不得以半径内很低的速度、很小的油门或短时漂移代替定点误差测量。

输出 `loiter.jsonl`、`loiter_targets.json`、`loiter_result.json`，独立记录 RMS/P95/最大/末端误差、超出 LOIT_RADIUS 的采样占比、固定目标变化、模式确认、净空和接触。

本次 MID-360 小半径实验的对应命令如下，启动与验证分两个终端执行：

```bash
python3 scripts/sim_session.py start --name mid360_loiter_wind_small --lidar mid360 --wind 3 --navigation --control
python3 scripts/validate_loiter.py --duration 120 --radius 0.3 --error-budget 0.5
# 验证结束后保存误差与实际轨迹图，再停止会话：
python3 scripts/plot_loiter.py "$(cat runs/latest.txt)"
python3 scripts/sim_session.py stop
```

本地基础检查分别运行：

```bash
(cd src/usv_nav2 && python3 -m unittest discover -s test -v)
python3 -m unittest discover -s src/usv_sim/test -v
python3 -m unittest discover -s scripts -p test_sim_session.py -v
git diff --check
```

## 自动任务接入边界

独立 LOITER 测试通过之后，还需要模式管理：选取可容纳全船、漂移和纠偏运动的区域；取消旧路径并确认；申请并确认 LOITER；监测障碍和定位；明确异常退出；恢复 GUIDED 后从当前位置重新规划、只放行新指令。当前桥的 Task1 运动许可仅控制 GUIDED 速度输出，不能直接阻止 LOITER 自行驱动推进器。

因此，本次独立实验不代表已经完成 Task1 自动定点等待、GNSS 降级保护或实船验收。

## 调试记录

- 首轮本机 Gazebo 广播 Wi-Fi 地址，能够发现 topic，但 `/clock`、真值和雷达数据没有到达本机订阅者。测试在解锁前因缺少新鲜输入退出。保持相同场景，将当前进程的 `GZ_IP` 设为 `127.0.0.1` 后数据恢复；不修改系统网络或其他项目。环境脚本保留显式覆盖能力。
- 新 LOITER 测试器曾错误地将帧计数 topic 当作高频发布。插件实际上每累计 250 帧发布一次，约 1 Hz；现在同时检查计数增长与低频遥测更新，保留两次失败记录，不改变物理或传感器门控。
- ROS 环境脚本不兼容提前启用 shell `nounset`；可选构建入口在 source 前只使用 `-e` 与 `pipefail`。
- 直接 CMake 安装没有工作区级 `local_setup.bash`。首次模型加载找不到动态库，修复为显式加入可选插件安装前缀；保留启动失败日志。
- MID-360 原始点云能以 10 Hz 到达，但首次同步处理因末段 TF 尚未到达，20 秒只输出 2 帧扫描。新增上述有时限队列后重新进行实际测试；没有放宽输入期限。
- 会话清理增加运行目录所有权校验，旧监督进程不得停止或删除新会话。两个交错清理用例通过；保留过程中发生的异常退出日志。
- Gazebo 停止后的插件析构仍有段错误日志，栈涉及既有 `WaveVisual` 或 `Task1Scene` 渲染资源；在加入 MID-360 之前的运行目录中也存在同类错误。本次确认受管进程全部退出，未将它记为正常析构通过；未改动既有 VRX 工程。运行期测量与退出阶段分别记录。

实际实验数据与尚未覆盖的项目见 [2026-11-04 实验记录](experiments/20261104_loiter_mid360.md)。
