# Task 1 第 1 步：录 bag，测两个距离

对应 [usv_only_plan.md](usv_only_plan.md) 第 8 节第 1 步。目标是两个数：**雷达多远能稳定打到浮标**、**相机白天多远能分清灯色**。全程只订阅和记录，不解锁、不切模式、不发控制指令；船由遥控器手动驾驶。

**验证状态**：分析逻辑、检查判据和无 ROS 的 bag 读取在开发电脑上通过离线测试（合成数据）。`rosbag2_py` 读取、`survey_check` 节点、录制脚本、构建脚本改动**尚未在 Orin 或任何 ROS 环境运行过**，更没有真实传感器数据。下面 A 段就是补这一层。

## 新增内容

| 文件 | 作用 |
|---|---|
| `src/robotx_task1/robotx_task1/survey_core.py` | 纯算法：位姿插值、逐点去畸变、聚类、路标关联、距离统计、投影与识色 |
| `src/robotx_task1/robotx_task1/survey_bag.py` | `survey_analyze`：读 bag 出报告 |
| `src/robotx_task1/robotx_task1/survey_check.py` | `survey_check`：录制前检查实时话题 |
| `scripts/record_task1_survey.sh` | 检查通过后录制 |
| `scripts/build_hardware.sh` | 构建与 rosdep 检查加入 `robotx_task1`（跳过仅仿真用的 `robotx_task1_nav`、`ros_gz_interfaces`） |

## A. Orin 上先做无硬件验证

```bash
cd "$HOME/rx_simu"
source scripts/hardware_underlay.sh
sudo apt install --no-upgrade ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins ros-humble-tf2-msgs
bash scripts/build_hardware.sh
source scripts/hardware_env.sh
(cd src/robotx_task1 && python3 -m unittest discover -s test -p 'test_survey_*.py' -v)
export ROS_DOMAIN_ID=162 ROS_LOCALHOST_ONLY=1
python3 -m unittest discover -s src/robotx_task1/test_ros -p 'test_survey_ros.py' -v
```

通过标志：

1. colcon 显示 **4 packages finished**，`ros2 pkg executables robotx_task1` 列出 `survey_check` 和 `survey_analyze`。
2. 第一组测试全部 OK（`test_survey_rosbags` 因未装 `rosbags` 显示 skipped 属正常）。这同时验证了 Orin 的 NumPy / SciPy 版本，开发电脑上用的是更新的版本。
3. 第二组 2 项 OK：用 `rosbag2_py` 写入并分析合成 bag，结果为雷达 12.0 m、相机 8.0 m；检查节点在没有传感器时返回失败。

验证后新开终端再做实船步骤，恢复 domain 62。任何一步失败，把完整输出带回来排查，不要跳过。

## B. 现场准备

1. 按 [hardware_deployment.md](../hardware_deployment.md) 第 8 节启动 A–G（MAVROS、map_origin、雷达安装 TF、MID-360 驱动、导航预览 `enable_control:=false`、livox_filter、cloud_to_scan）。
2. 启动 RealSense 驱动，只开彩色流。参数名随驱动版本不同，先用 `ros2 launch realsense2_camera rs_launch.py --show-args` 核对。建议 640×480、15 fps、`rgb8`；1280×720 30 fps 的原始图约 80 MB/s。按方案 5.3 固定低曝光和白平衡，把数值记在 `notes.md`。
3. 发布相机安装 TF（相机机体系：x 前、y 左、z 上；填 `.env.boat` 的 `USV_CAMERA_*`）：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x "${USV_CAMERA_X:?}" --y "${USV_CAMERA_Y:?}" --z "${USV_CAMERA_Z:?}" \
  --roll "${USV_CAMERA_ROLL:?}" --pitch "${USV_CAMERA_PITCH:?}" --yaw "${USV_CAMERA_YAW:?}" \
  --frame-id base_link --child-frame-id "${USV_CAMERA_FRAME:?}"
```

4. 单独运行检查，逐条处理 `[FAIL]`：

```bash
ros2 run robotx_task1 survey_check
```

`[WARN]` 不阻止录制，但要看懂。RTK 一行需要 mavros_extras 的 `gps_status` 插件；当前 `config/real/mavros.yaml` 的 allowlist 没有它，未加载时请在地面站读定位类型并写进 `notes.md`。

## C. 录制

```bash
bash scripts/record_task1_survey.sh approach1
```

脚本先跑检查，失败则不录。数据在 `runs/task1_approach1_<时间>/`，其中 `notes.md` 现场填写。

驾驶方法（遥控器 MANUAL，尽量慢，约 0.2–0.3 m/s）：

- 从 **15–20 m 外**船头对着浮标直线靠近到约 2 m，再原路退回或掉头驶离；重复 2–3 趟。慢速是为了让每 1 m 距离段有足够样本（雷达至少 3 s，相机至少 2 s）。
- 相机固定朝前，只有船头对着的浮标才有颜色数据。
- 每种灯态（红、绿、闪蓝、常蓝、灭）尽量各覆盖一个浮标；至少要有红和绿。
- 记下每个浮标的真实灯态和大致位置，后面标注要用。
- 结束按 Ctrl-C，执行 `ros2 bag info runs/task1_approach1_<时间>/bag` 确认各话题条数不为零。

## D. 分析

```bash
ros2 run robotx_task1 survey_analyze runs/task1_approach1_<时间>/bag --crops
```

结果在 `<bag>_analysis/`：`report.json`、`landmarks.png`（俯视图）、`labels_template.json`、`crops/`（被判色的图像区域，黄框为判色范围）。

1. 对照 `landmarks.png` 和表中的 `ahead` / `left`（相对第一帧船位的前、左距离），在 `labels_template.json` 的 `truth` 里填 `red`、`green`、`blue_flash`、`blue_steady`、`off`；岸壁碎片、其他船填 `ignore`。
2. 带标注重跑：

```bash
ros2 run robotx_task1 survey_analyze runs/task1_approach1_<时间>/bag --crops \
  --labels runs/task1_approach1_<时间>/bag_analysis/labels_template.json
```

3. 读最后两行：`LiDAR stable range` 和 `Camera color range` 的 `min`，就是要的两个数。

**判据（均可用参数调整）**

- 雷达：0.5 s 窗口内在路标 0.5 m 内有不少于 5 点、尺寸不超过 1 m 的聚类，算检出。某 1 m 距离段检出率不低于 90 % 且至少 6 个窗口才算稳定。`lidar_band_m` 是连续稳定的距离段，近端即近距离盲区（对应方案 5.6 的估算）。
- 相机：黄框内高亮高饱和像素按色相计数。闪烁灯占空比 50 %，要求该距离段至少 30 % 的帧判对且判成其他颜色的帧不超过 5 %；常蓝要求 80 %；灭灯要求误判不超过 5 %。
- 样本不足的距离段不外推，稳定段到此为止。

**必须人工核对**

- 打开 `crops/`，确认黄框套住了灯。偏了说明相机外参或灯高不对，用 `--lamp-height`（相对 base_link）或 `--lamp-offset` 调整；外参细调属于第 3 步。
- `z_bot` / `z_top` 是浮标点云相对 base_link 的高度范围，用来确定 `USV_MARK_MIN_Z/MAX_Z`。
- `report.json` 每个距离段的 `rms_error` 若随距离明显增大，说明航向有误差（方案 5.7）；同时核对飞控参数 `EK3_SRC1_YAW`。
- 开头的 `PROBLEM:` 行不能忽略。雷达时钟未同步时默认不出结果；`--lidar-stamp receive` 只能给出近似值，实时链路仍然不可用。

## 没有 ROS 的电脑上分析

把 bag 目录拷到电脑后：

```bash
python -m pip install rosbags numpy scipy
cd src/robotx_task1
python -m robotx_task1.survey_bag <bag 目录> --crops
```

这条路径读不了 `mavros_msgs` 中除 State 以外的消息（如 GPS 定位类型），其余结果相同。
