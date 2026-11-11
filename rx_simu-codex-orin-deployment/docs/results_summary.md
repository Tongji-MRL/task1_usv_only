# 最新导航测量结果

由 `scripts/summarize_results.py` 从保留的运行记录生成。仅展示每个名称最近一次尝试；历史失败见 `results.json` 的 `attempts`。
位置、航向和净空均使用 VRX 真值独立评估。取消和不可达用例的终点误差不作为到点验收。

| 用例 | 判定 / action 状态 | 位置误差 m | 航向误差 rad | 最小净空 m | 10 s 漂移 m | 日志 |
|---|---|---:|---:|---:|---:|---|
| single_1 | 通过 / 4 | 0.368 | 0.046 | 1.558 | 0.199 | [20261029_054518_single_1](../runs/20261029_054518_single_1/single_1_result.json) |
| single_2 | 通过 / 4 | 0.352 | 0.046 | 1.559 | 0.186 | [20261029_054642_single_2](../runs/20261029_054642_single_2/single_2_result.json) |
| single_3 | 通过 / 4 | 0.363 | 0.047 | 1.550 | 0.197 | [20261029_054806_single_3](../runs/20261029_054806_single_3/single_3_result.json) |
| multi_1 | 通过 / 4 | 0.318 | 0.161 | 1.915 | 0.043 | [20261029_054930_multi_1](../runs/20261029_054930_multi_1/multi_1_result.json) |
| multi_2 | 通过 / 4 | 0.348 | 0.157 | 1.878 | 0.037 | [20261029_055107_multi_2](../runs/20261029_055107_multi_2/multi_2_result.json) |
| multi_3 | 通过 / 4 | 0.319 | 0.157 | 1.895 | 0.045 | [20261029_055244_multi_3](../runs/20261029_055244_multi_3/multi_3_result.json) |
| static_obstacle | 通过 / 4 | 0.264 | 0.161 | 3.355 | 0.095 | [20261029_055421_static_obstacle](../runs/20261029_055421_static_obstacle/static_obstacle_result.json) |
| cancel | 通过 / 5 | 3.638 | 0.028 | 4.479 | 0.575 | [20261029_062236_cancel](../runs/20261029_062236_cancel/cancel_result.json) |
| unreachable | 通过 / 6 | 59.999 | 0.032 | 6.406 | 0.004 | [20261029_055759_unreachable](../runs/20261029_055759_unreachable/unreachable_result.json) |
| unreachable_obstacle | 通过 / 6 | 9.999 | 0.035 | 6.402 | 0.005 | [20261029_055907_unreachable_obstacle](../runs/20261029_055907_unreachable_obstacle/unreachable_obstacle_result.json) |
| wind | 通过 / 4 | 0.228 | 0.209 | 1.472 | 0.252 | [20261029_060015_wind](../runs/20261029_060015_wind/wind_result.json) |
| waves | 通过 / 4 | 0.368 | 0.044 | 1.571 | 0.181 | [20261029_060144_waves](../runs/20261029_060144_waves/waves_result.json) |
| wind_waves | 通过 / 4 | 0.291 | 0.213 | 1.520 | 0.140 | [20261029_060308_wind_waves](../runs/20261029_060308_wind_waves/wind_waves_result.json) |
| gps_input_fixed | 通过 / 4 | 0.377 | 0.116 | 1.489 | 0.205 | [20261029_060434_gps_input_fixed](../runs/20261029_060434_gps_input_fixed/gps_input_fixed_result.json) |
| dynamic_navigation | 通过 / 4 | 0.387 | 0.167 | 1.847 | 0.018 | [20261029_105554_dynamic_navigation](../runs/20261029_105554_dynamic_navigation/dynamic_navigation_result.json) |
| feedback_goal | 通过 / 4 | 0.371 | 0.038 | 1.570 | 0.205 | [20261029_062349_feedback_goal](../runs/20261029_062349_feedback_goal/feedback_goal_result.json) |

action 4 = 成功，5 = 取消，6 = 中止。验收条件不只检查 action 状态，还检查真值误差、碰撞、途经点和零指令延迟。
推力归零和物理停止观测见 `results.json` 的 `metrics.post_goal`；`thrust_measurement` 区分输入推力命令与物理插件反馈，`null` 表示观察窗口内未达到该条件。
