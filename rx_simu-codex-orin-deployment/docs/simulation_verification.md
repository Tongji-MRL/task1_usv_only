# LOITER / MID-360 仿真补充验证

2026-11-04。固定 ArduRover 4.4.0 提交与现有控制链，对 MID-360 与原生 LOITER 做实际闭环验证。[实验报告](experiments/20261104_simulation_verification.md) 区分通过、失败和未覆盖；[证据清单](experiments/simulation_verification/runs.json) 记录成功和失败的原始运行目录。水面光学、推进器实测曲线、真实雷达时钟和安装尺寸需要实物数据，不能由仿真通过代替。

| 检查组 | 本轮实验与判据 | 状态 |
|---|---|---|
| 障碍可见性 | 三种安装、三种高度、五个距离；最终配置另测八方位 | 已测，有距离/高度限制 |
| 动态清除 | 区分真实背景回波与无返回水域，不强制清图 | 有背景通过；无背景残留 |
| 输入与时序 | 中断、无效扫描、过期时间、缺 TF、暂停恢复与重置 | GUIDED 8/8、暂停通过；reset 自动恢复失败 |
| 原生保持 | 三次重复、零速度对照、运动中进入、风浪组合 | 原生保持通过；零速度对照漂移约 4.05 m |
| 模式所有权 | GUIDED→LOITER→GUIDED；取消旧动作、确认模式、新命令；手动接管不抢回控制 | 桥合同 15/15 + 预处理退出通过；自主横穿内两次恢复通过 |
| 保持异常 | 输入失效、桥/MAVROS 退出、GNSS Fixed/Float/3D/失锁/恢复 | 已测；Fixed 偏差可漏检，失锁 HOLD 后仍会漂移 |
| Task1 | MID-360 配置下港口全流程；比赛只切一次灯；静态、横穿、封路，独立碰撞/规则裁判及视频 | 静态通过；横穿和封路均因残留占据未完成 |
| 时间戳与 TF | 原云、逐点清除射线、扫描、飞控 odom；实际 DDS 发布者 | 源时间戳匹配，7 条 TF 边各一个发布者 |
| 仿真退出 | 渲染资源在 RenderTeardown 释放，MID 插件不接管宿主信号 | Gazebo 多次正常退出；ROS bridge 包装进程仍需 SIGTERM |

自动 LOITER 等待已作为显式可选配置实现和验证。进入前必须取消导航动作，确认输入与可容纳船体转向及纠偏的区域；外部改变模式、输入失效和任务取消需要明确退出行为。不能把 GUIDED 的零速度门控当作能够约束 LOITER。

任何失败都保留原始运行目录和原因；修复后另开实验，不覆盖失败数据。最终报告区分接口检查、实际闭环和实船未验证事项。

## 构建与独立传感器检查

前置依赖见 [requirements](../requirements/README.md)。新增消息字段后必须重建接口及消费者，旧消息定义的 bag 应使用对应历史代码读取。

```bash
cd /home/zjy/usv_nav
source scripts/env.sh
bash scripts/build_mid360.sh
bash scripts/build_render_fix.sh
colcon build --symlink-install --base-paths src --packages-select robotx_task1_interfaces usv_nav2 usv_sim robotx_task1 robotx_task1_nav
cmake --build build/task1_gazebo -j4
source scripts/env.sh
# 控制默认关闭；先查看低位安装的传感器。
python3 scripts/sim_session.py start --name mid360_inspection --navigation --task1 --course harbor --lidar mid360 --mid360-mount low_bow
```

另一个终端，同样 `source scripts/env.sh`：

```bash
python3 scripts/validate_mid360.py --duration 60
python3 scripts/validate_lidar_geometry.py --azimuths
python3 scripts/sim_session.py stop
```

`validate_lidar_geometry` 保持 disarm；实体箱体位置和离开时间写入日志。启动参数、源码快照、库哈希与 `shutdown_audit.json` 均在当前运行目录。

## 独立保持、故障与 GNSS 批测

以下批测会明确启用本机 SITL 控制，逐例冷启动、保存日志并结束会话；不得与已有仿真并行运行。验证器核实固定固件、运行目录及本机连接，不连接实船。

```bash
source scripts/env.sh
python3 scripts/run_simulation_checks.py sensor
python3 scripts/run_simulation_checks.py hold
python3 scripts/run_simulation_checks.py gnss
python3 scripts/run_simulation_checks.py station
python3 scripts/run_simulation_checks.py timing
# 独立复现单项：
python3 scripts/run_simulation_checks.py hold --case mid360_hold_waves
```

- `hold`：三次 120 s 原生保持、GUIDED 零速度对照、运动中切入、风浪组合。零速度对照的宽位移预算只允许记录漂移，不是定点通过标准。
- `gnss`：GPS_INPUT 10 Hz，分开控制 fix、报告精度、随机误差、缓慢偏置和延迟；不自动新增 RTK 门控或故障后恢复模式。
- `station`：替换任务许可和传感器转发的专用桥合同试验，检查 15 种交接/故障，并另开冷启动测试实际预处理进程退出。它不运行自主 Task1。
- `timing`：GUIDED 输入故障、暂停和 reset。失败项返回非零并保留证据；直接 Gazebo reset 不应作为常规重启方式。

保持图：`python3 scripts/plot_loiter.py /home/zjy/usv_nav/runs/<运行目录>`。停止实验必须使用 `python3 scripts/sim_session.py stop`；下一次启动重新建立物理、FCU、MAVROS、TF 和导航，不能只重置 Gazebo 时钟。

## 自动等待的 Task1 实验

自动 LOITER **默认关闭**，仅 `--task1-loiter` 开启。它同时配置模式管理器和专用 GCS 心跳，不能只手工改 bridge 参数而遗漏 FCU 失联配置。

```bash
source scripts/env.sh
python3 scripts/run_task1_suite.py --group dynamic --seed 1 --changes 1 --course harbor --lidar mid360 --mid360-mount low_bow --task1-loiter --record-maps --record-sensors --stop-on-failure
```

每个动态运行保留独立规则/接触裁判、任务状态、原始回波、地图和视频。查看是否实际进入 LOITER、是否保持同一目标、是否确认 GUIDED 后重新规划；仅开启开关不能算交接通过。`--changes 1` 对应用户确认的比赛只切一次灯。

```bash
python3 scripts/analyze_task1_stationkeeping.py /home/zjy/usv_nav/runs/<运行目录>
python3 scripts/analyze_task1_maps.py /home/zjy/usv_nav/runs/<运行目录> --buoys
python3 scripts/annotate_task1.py /home/zjy/usv_nav/runs/<运行目录>
python3 scripts/export_task1_record.py /home/zjy/usv_nav/runs/<运行目录>
python3 scripts/collect_simulation_evidence.py docs/experiments/simulation_verification/runs.json
python3 scripts/plot_simulation_evidence.py docs/experiments/simulation_verification/evidence.json
```

导出应在所有进程和分析结束后执行。视频和 bag 在忽略目录，仓库跟踪紧凑报告、图和校验记录；不上传凭据或完整第三方源码。
