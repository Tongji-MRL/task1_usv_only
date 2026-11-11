# Task 1 复现流程

工作目录 `/home/zjy/usv_nav`。先读 `implementation.md` 的规则边界和 `validation.md` 的实际通过/失败记录。首次环境安装、固定固件及原始导航流程沿用仓库主 README。

Task1 规划还链接服务器已有的 `ros-humble-ompl`（`1.7.0-4jammy.20260226.011457`）；移植环境需要该 ROS 开发包。Dubins 距离只用于前向穿越搜索的排序，碰撞和赛事规则仍逐边硬检查。

## 构建与检查

```bash
source scripts/env.sh
colcon build --symlink-install --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release
cmake -S tools/task1_gazebo -B build/task1_gazebo -DCMAKE_BUILD_TYPE=Release
cmake --build build/task1_gazebo -j4
(cd src/usv_nav2 && python3 -m unittest discover -s test -v)
python3 -m unittest discover -s src/robotx_task1/test -v
ROS_DOMAIN_ID=43 python3 -m unittest discover -s src/robotx_task1/test_ros -v
build/robotx_task1_nav/test_rules
build/robotx_task1_nav/test_hull
python3 -m unittest discover -s src/usv_sim/test -v
git diff --check
```

运行过程中不要覆盖已加载的共享库。运行文件由 `sim_session.py` 创建在忽略目录 runs 中；每次保存本仓库源码快照、固件/插件哈希、生成世界与参数，不覆盖既有工程。

随时用 `python3 scripts/task1_status.py` 查看当前进程是否存活、最近的物理帧及记录年龄、任务阶段和已结束的批测结果。`managed_session: false` 表示当前没有本仓库管理的仿真；旧的 COMPLETE 记录不会被解释为仍在运行。

## 场景及任务

只看模型，控制保持关闭：

```bash
source scripts/env.sh
python3 scripts/sim_session.py start --task1 --navigation --name task1_preview
```

另一个已 source 的终端运行 `python3 scripts/check_task1_model.py` 检查真实相机五种灯态、1 s 亮/1 s 暗、OFF 实体和侧视遮光；`python3 scripts/check_task1_lidar.py` 记录原始点云、过滤点云和扫描回波。模型检查要求 SITL 未解锁，会改变测试浮标的灯态。结束用 `python3 scripts/sim_session.py stop`。

完整任务由以下命令执行正常解锁、任务开始、证据记录、10 s 末端漂移记录和清理：

```bash
python3 scripts/run_task1_suite.py --seed 1 --changes 1 --record-maps
```

根据用户 2026-11-01 确认，正式比赛仅一次语义灯态切换，闪烁亮灭不计次数。`--group all --record-maps` 先运行 9 项比赛主验收：3 种布局 × 一次切换 × 无动态障碍/横穿/临时阻挡；再运行对应的 9 项零切换对照。`--group all --changes 1 --record-maps` 只运行这 9 项主验收。seed 3 含一个 OFF 实体。`--group dynamic --seed 1 --changes 1 --dynamic cross` 可单独选择一次切换加横穿。

两次切换仅在显式 `--stress` 下运行，例如 `--group dynamic --seed 1 --changes 2 --dynamic cross --stress`。它的结果列为额外压力测试，不进入比赛主验收计数。`--skip-case s1_c0_none` 可在续跑时跳过已归档的用例；跳过不代表通过，报告仍读取该例的实际记录。无匹配或无效筛选时脚本报错，失败运行保留，结果按独立裁判判定，不以进程退出码代替任务成功。

排查感知问题时加 `--record-sensors`，在 `sensors/` 保存原始/过滤/标记/清除点云、扫描、TF、时钟、EKF 和指令。测试脚本检测到实际接触后请求任务取消并继续记录 10 s；这是测试端中止，不能算作导航本身主动避碰。

额外观测距离候选未能解决恢复问题，默认关闭。仅复现这个候选时添加 `run_task1_suite.py --visibility-guard`，或手动启动时加 `sim_session.py --task1-visibility-guard`；不要将它当作已经通过的动态避障策略。

人工观察时，可先 `sim_session.py start --task1 --navigation --control ...`，再运行 `validate_task1.py`；控制开启只面向脚本核验过的本次固定版本 SITL。不要同时运行其他速度控制器。

## 港区工程场景与标注视频

`--course harbor` 选择 [harbor.json](../../config/task1/harbor.json) 中的独立布局：88 m × 47 m 边界、错位红绿浮标、OFF 实体、南侧码头和北侧浮台。三个固定布局改变通道弯曲和一次灯变的时机（进入 TRANSIT 后 45 / 70 / 100 s）。它们用于增加工程测试的约束，不是官方场地测绘复刻；结果和原 baseline 用例分开归档。

交通目标使用独立工作船外形、2 m × 3 m × 0.9 m 保守碰撞体，按预设航线匀速横穿，`block` 模式在中段暂停再继续。它是连续运动的实体障碍，尚无独立船舶控制器、水动力和碰撞避让决策。交通真值只供裁判与视频使用，导航仍靠 LiDAR 感知；两个固定码头作为已测绘禁航区进入静态地图。所有十个浮标继续使用 2026 RoboBuoy 模型。

以下命令逐次冷启动；必须等前一条结束并完成进程清理再执行下一条：

```bash
source scripts/env.sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 scripts/run_task1_suite.py --course harbor --group base --seed 1 --changes 1 --record-maps
python3 scripts/run_task1_suite.py --course harbor --group dynamic --seed 1 --changes 1 --dynamic cross --record-maps --record-sensors
python3 scripts/run_task1_suite.py --course harbor --group dynamic --seed 1 --changes 1 --dynamic block --record-maps --record-sensors
python3 scripts/run_task1_suite.py --course harbor --group base --seed 1 --changes 1 --wind 2 --wave-gain 0.08 --record-maps
```

`wave-gain` 是 VRX 波浪插件的增益，不能直接称作波高；实际摇摆与漂移从该次真值日志计算。未实测的布局、风浪和交通组合不能继承其他用例的通过结论。低层遮挡、停止后的占据残留和动态相遇仍是需要验证的现有能力缺口。

新运行同时记录每帧相机源时间和任务实际接受的 `/task1/reference_path`。完整任务结束后生成 1920 × 1080 标注视频及轨迹、末端、地图图片：

```bash
python3 scripts/annotate_task1.py runs/实际运行目录
# 单帧审核，秒数相对任务开始：
python3 scripts/annotate_task1.py runs/实际运行目录 --preview 260
```

等待/绕行改进的同场景对照与沿途 ETA 图，可在完整运行后生成；不要使用未完成的任务作完整耗时对照：

```bash
python3 scripts/analyze_route_efficiency.py runs/旧完整运行 runs/新完整运行 --output runs/新完整运行/route_efficiency.json
python3 scripts/plot_route_decisions.py runs/旧完整横穿运行 runs/新完整横穿运行 --output runs/新完整横穿运行/visualization
```

`route_efficiency.json` 单列策略等待、未知释放时间的观察等待、前移观察、几何换路和恢复过的规划/控制失败；限速命令与船体实际速度分别核验。只有完成并停录后才导出实验包，脚本拒绝覆盖已有归档：

```bash
python3 scripts/export_task1_record.py runs/实际运行目录
```

输出 `visualization/task1_annotated.mp4`：左侧是真实 Gazebo 相机加数据标注，右侧为同次发布的局部代价地图和状态。紫色虚线为当前执行的规划路径，青色实线为实际 VRX 航迹，黄色星号为当前阶段末端目标；停车/重规划时旧路径改为灰色。红绿保留给浮标，ID、灯态文字和闪烁亮暗相位分别标清。相机标注按相机源时间投影，地图按回放时间取最近的历史记录；显示源时间差，不使用未来消息填补空缺。

标注来自已知灯态接口，不能视为视觉识别结果；VRX 真值用于离线航迹评估，导航仍使用飞控 EKF。回放按 0.5 仿真秒采样、5 FPS 编码，为 2.5 倍速；原相机 MP4 保留不变，来源和校验值见 `task1_annotated.json`。若旧运行缺少相机标定或逐帧时标，脚本拒绝猜测同步关系。

异常场景分别冷启动：

```bash
python3 scripts/run_task1_suite.py --fault observation_gap
python3 scripts/run_task1_suite.py --fault reordered_observation
python3 scripts/run_task1_suite.py --fault mission_exit
python3 scripts/run_task1_suite.py --fault cancel
python3 scripts/run_task1_suite.py --fault role_change
python3 scripts/run_task1_suite.py --fault no_path
python3 scripts/run_task1_suite.py --fault late_change
```

异常测试的“通过”表示按预期停车/拒绝任务，和正常 Task1 完成分开存储。重放观测保留原时间戳。任务进程退出时，飞控、MAVROS 和物理仿真继续运行。

## 数据和图像

一轮已完成的一次灯态切换实验示例见 [20261101_181046 完整记录](experiments/20261101_181046.md)，包含 MP4、轨迹/地图、参数、事件时间和原始证据索引。这是历史版本、无动态障碍的通过记录，不替代当前版本回归。

每次目录包含 task1_mission.jsonl（事件）、task1_judge.jsonl（独立真值、场景、接触）、task1.jsonl（指令、EKF、推力反馈、真值）、task1_result.json（独立判定）、task1_execution.json（动作执行）、两个实际相机 MP4。推进器力由 Gazebo Thruster 反馈角速度和固定配置反算，详见 thrust_observation.json，不是实体测力计或轴动力学模型。

启动时选择 `--record-maps` 才能生成地图回放：

```bash
python3 scripts/plot_task1.py runs/实际运行目录 --animate
```

visualization 中提供参考路径/真值/EKF 航迹、期望末端与误差、实际静态/全局/局部地图、MP4 和 GIF。参考是空间路径，不是带时序的速度轨迹；GIF 是全视频节选。所有浮标实体持续绘制；空心灯标为闪烁暗相位，深灰为语义 OFF。

`--animate` 同时将真实 Gazebo 总览相机转为便于播放的 H.264 `task1_vrx.mp4`；`task1_replay.mp4` 是按记录数据绘制的 2.5 倍速地图回放。真实相机按收到的帧顺序编码为 5 FPS，接收帧率可能低于配置值，因此不能用视频播放时长测量停车或灯变延迟；以画面内仿真时标、较新运行的 `task1_video.json` 源时标和原始日志为准。图像与视频的来源及校验值写入 `task1_manifest.json`。

`python3 scripts/analyze_task1_maps.py runs/实际运行目录 --buoys` 检查每张全局地图持续保留全部已知浮标；不加 `--buoys` 检查动态障碍的标记和离开后清除。已知浮标占据层来自本轮允许的已知观测，不能代替 `check_task1_lidar.py` 的真实传感器回波检查。

## 已知观察接口与规则规划

`/task1/observations` 为 BuoyArray：map 坐标、源时间戳、完整十个 ID、碰撞半径、语义状态、revision、survey_complete。已知 UAV 替身从 Gazebo 场景取得这些信息；没有读取船体真值定位。将来接视觉时保持此契约和来源时间，不把旧观测改成新时间戳。

任务机先检查全部调查记录，再依次接近入口、顺时针完整绕圈、合法穿越、逆时针完整绕出口。全局规划含位置、航向和红/绿通过标志；使用权重 1.4 的 A*、0.4 m 状态格、36 航向、0.6 m 前向运动边、逐 0.1 m 碰撞/规则检查、10 s 搜索预算。穿越段仅生成可前向跟踪的曲线，不生成原地转向段；错误侧正横边界留 1 m 纵向余量，判断更近的遮挡浮标时留 0.3 m 余量。独立裁判仍按原始正横规则计量。局部 DWB 增加硬性规则 critic 和可恢复的末端转向 critic。动态障碍由物理 LiDAR 感知，未向规划器发布障碍物真值轨迹。

语义变化时版本作废并取消动作；运动许可为 0.5 s 租约。EKF 平动低于 0.04 m/s、角速低于 0.04 rad/s，持续 1.5 s 后才重新规划。该余量来自实测 EKF 与真值速度的短时差异。独立裁判另外用真值低于 0.05 m/s / 0.05 rad/s、持续 1 s 检查停车；发零、推进器反馈、停车距离和仿真帧推进分别报告。闪烁暗相位不增加版本，也不触发重规划。

浮标观测在任务机中的源时标和墙钟期限均为 0.35 s，为 0.1 s 任务检查周期和桥转发留出余量；较早的 0.5 s 配置在实际观测中断测试中未满足 0.6 s 发零目标。EKF 数据有效期仍为 0.5 s，插件内部的观测/上下文检查仍为 0.5 s；任务机使用更严格的观测门槛。

入口/出口身份中途重分配按不支持场景中止；无合法路径保持停车重试，整场 1200 仿真秒超时。没有新增 RTK/EKF 质量策略或持续定点保持。新模型用固定系泊和保守碰撞外廓，适合本轮船端规则测试，尚不代表波浪中的浮标运动或真实相机识别精度。

## 本轮实际参数与诊断

每次运行的 `generated/nav2_wamv.yaml`、`generated/task1_scenario.json` 和 `rover.parm` 保存该次参数。Nav2 启动文件将 `use_sim_time` 和 `enable_control` 作为显式启动覆盖项；因此不能只根据生成参数模板中的默认 `false` 判断运行状态。可用 `ros2 param get /controller_server use_sim_time` 或只读的 `python3 scripts/audit_graph.py` 检查实际值，后者同时记录 QoS、源时间戳和 TF 边。

| 项目 | 本轮数值 |
|---|---|
| DWB 频率、预测窗口 | 10 Hz、5 s；11 月 1 日较早运行使用 3 s |
| 前进、转向上限 | 0.3 m/s、0.4 rad/s；不规划侧向速度 |
| 平动、角加速度 | 0.2 m/s²、0.4 rad/s² |
| 平动、角减速度绝对值 | 0.08 m/s²、0.2 rad/s²；按实测较慢制动收紧 |
| 输入速度归零阈值 | x/y/角速度均 0.005，保留低速漂移供 DWB 预测 |
| DWB 速度采样 | 前向 15、侧向 1、角速度 25 |
| 导航到点窗口 | 0.4 m、0.2 rad；独立真值验收 0.5 m、0.25 rad |
| WAM-V 碰撞外廓 | 约 5.371 m × 2.537 m，使用包含推进器的实际凸包；局部 padding 0.5 m，全局 1.0 m |
| 膨胀半径、衰减系数 | 4.5 m、1.0 |
| 全局地图 | 100 m × 100 m 水域，0.2 m 栅格，2 Hz 更新 |
| 局部地图 | 30 m × 30 m 滚动窗口，0.05 m 栅格，5 Hz 更新 |
| LaserScan 投影 | `pointcloud_to_laserscan`，`base_stabilized`，高度 0.25–3 m，角分辨率 0.5°，距离 0.3–20 m |
| Task1 障碍层候选 | VoxelLayer：16 层 × 0.2 m，起点 z = −0.2 m；真实返回标记、三维射线清除，不清空 padded footprint |
| 额外观测距离 | 默认 0（关闭）；实验开关启用时约 5.98 m，按 `(1.8−0.25)/tan(15°)+0.2` 计算；已知浮标只豁免此额外距离，仍做实体碰撞检查 |
| 障碍标记 / 射线清除 | 15 m / 20 m；已知浮标层持续保留 OFF 实体 |
| 桥输入超时 | 指令、odom、scan 各 0.5 s，飞控 state 2 s；桥发布 20 Hz |

任务机的停车、绕圈和搜索设置见上一节。`config/task1/defaults.json` 同时记录模型依据和当前任务默认值；现阶段仅边界与圈半径由场景生成器读取，任务机其他值以其 ROS 参数声明为准，搜索离散参数在 `rules.hpp` 中。修改这些值时同时检查场景、任务机和独立裁判的一致性，不能只改记录文件便认为运行配置已更新。

规划耗时按每次事件的单调墙钟记录在 `results.json` 的 `execution_metrics`，灯变到重新行驶的暂停则按仿真时钟记录。Task1 当前搜索预算为 10 s，预算耗尽时保持停车后重试；最终到达并不表示每次搜索都在预算内成功。

复现记录地图上的搜索：`cmake -S tools/task1_search -B build/task1_search -DCMAKE_BUILD_TYPE=Release && cmake --build build/task1_search`，再执行 `python3 scripts/replay_task1_search.py runs/运行目录 仿真时刻`。它比较欧氏与 Dubins 估计，使用飞控 EKF 起点和当时语义，输出路径及耗时。OccupancyGrid 已量化非致命栅格代价，因此这是离线诊断，不能代替实时闭环。

增加 `--reference-stamp 路径源时刻 --no-search` 可单独核查已接受参考路径在该张地图上的船体碰撞、规则余量和正横事件，输出 JSON 与逐采样点 CSV。此检查只读记录，不连接控制；即使采样可行，也不能代替起点连接、重新规划、动态交通评估和实际闭环。

交通碰撞修复的新增节点、预测层、复测与保护进程故障命令见 [交通让行说明](traffic_avoidance.md)。完整 Task1 驱动现在在解锁前检查两张 costmap 中的浮标占据，并保留 `startup_costmap_check.json`；新层不能覆盖原来的浮标、静态边界和传感器占据。`traffic_avoidance.json` 与 `visualization/traffic_avoidance.png` 用于查看预警、零指令、实际推力、停止和恢复。规划耗时以 `path_candidate` 为结束事件，候选交通检查的等待不再计入搜索耗时。
