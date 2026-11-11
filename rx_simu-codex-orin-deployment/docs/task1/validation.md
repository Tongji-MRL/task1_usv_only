# Task 1 实际验证记录

由 `scripts/report_task1.py` 根据保留的独立裁判结果生成。完整历史（包括调试失败）见 [results.json](results.json)。
每项展示最近一次已完成的尝试，可能来自较早提交；运行目录的 provenance.json 与源码快照标识当时版本。新修改不能继承旧运行的通过结论。
根据用户 2026-11-01 确认，正式比赛仅一次语义灯态切换；闪烁亮灭不计次数。一次切换为主验收，零切换为对照，两次切换仅作额外压力测试。
比赛主验收最近记录：2 项通过，4 项接触碰撞，3 项启动失败、任务未开始；启动失败不能算作已完成场景验证。
独立港区工程场景最近记录：4 项通过，0 项接触碰撞；不替代上面的 baseline 矩阵。
正常任务与故障测试分开判定；程序报告 COMPLETE 不等于独立验收通过。尚未运行的组合明确列出。

| 范围 | 用例 | 判定 | 行为源码版本 | 位置 / 航向误差 | 最小净空 | 证据 |
|---|---|---|---|---|---|---|
| 比赛主验收 | s1_c1_none | 通过 | 较早；需回归 | 0.397 m / 0.029 rad | 1.360 m | [20261101_221130_task1_s1_c1_none](../../runs/20261101_221130_task1_s1_c1_none/task1_result.json) |
| 比赛主验收 | s1_c1_cross | 失败：接触碰撞 | 较早；需回归 | 不适用／未完成 | 0.000 m | [20261101_222214_task1_s1_c1_cross](../../runs/20261101_222214_task1_s1_c1_cross/task1_result.json) |
| 比赛主验收 | s1_c1_block | 失败：接触碰撞 | 较早；需回归 | 不适用／未完成 | 0.000 m | [20261101_222809_task1_s1_c1_block](../../runs/20261101_222809_task1_s1_c1_block/task1_result.json) |
| 比赛主验收 | s2_c1_none | 通过 | 较早；需回归 | 0.383 m / 0.021 rad | 1.249 m | [20261101_223511_task1_s2_c1_none](../../runs/20261101_223511_task1_s2_c1_none/task1_result.json) |
| 比赛主验收 | s2_c1_cross | 失败：接触碰撞 | 较早；需回归 | 不适用／未完成 | 0.000 m | [20261101_224555_task1_s2_c1_cross](../../runs/20261101_224555_task1_s2_c1_cross/task1_result.json) |
| 比赛主验收 | s2_c1_block | 失败：接触碰撞 | 较早；需回归 | 不适用／未完成 | 0.000 m | [20261101_225150_task1_s2_c1_block](../../runs/20261101_225150_task1_s2_c1_block/task1_result.json) |
| 比赛主验收 | s3_c1_none | 启动失败：任务未开始 | 较早；需回归 | 不适用／未完成 | — | [20261101_225903_task1_s3_c1_none](../../runs/20261101_225903_task1_s3_c1_none/task1_result.json) |
| 比赛主验收 | s3_c1_cross | 启动失败：任务未开始 | 较早；需回归 | 不适用／未完成 | — | [20261101_230050_task1_s3_c1_cross](../../runs/20261101_230050_task1_s3_c1_cross/task1_result.json) |
| 比赛主验收 | s3_c1_block | 启动失败：任务未开始 | 较早；需回归 | 不适用／未完成 | — | [20261101_230238_task1_s3_c1_block](../../runs/20261101_230238_task1_s3_c1_block/task1_result.json) |
| 零切换对照 | s1_c0_none | 通过 | 较早；需回归 | 0.396 m / 0.034 rad | 1.236 m | [20261101_220059_task1_s1_c0_none](../../runs/20261101_220059_task1_s1_c0_none/task1_result.json) |
| 零切换对照 | s1_c0_cross | 启动失败：任务未开始 | 较早；需回归 | 不适用／未完成 | — | [20261101_230425_task1_s1_c0_cross](../../runs/20261101_230425_task1_s1_c0_cross/task1_result.json) |
| 零切换对照 | s1_c0_block | 启动失败：任务未开始 | 较早；需回归 | 不适用／未完成 | — | [20261101_230613_task1_s1_c0_block](../../runs/20261101_230613_task1_s1_c0_block/task1_result.json) |
| 零切换对照 | s2_c0_none | 启动失败：任务未开始 | 较早；需回归 | 不适用／未完成 | — | [20261101_230800_task1_s2_c0_none](../../runs/20261101_230800_task1_s2_c0_none/task1_result.json) |
| 零切换对照 | s2_c0_cross | 启动失败：任务未开始 | 较早；需回归 | 不适用／未完成 | — | [20261101_230947_task1_s2_c0_cross](../../runs/20261101_230947_task1_s2_c0_cross/task1_result.json) |
| 零切换对照 | s2_c0_block | 启动失败：任务未开始 | 较早；需回归 | 不适用／未完成 | — | [20261101_231134_task1_s2_c0_block](../../runs/20261101_231134_task1_s2_c0_block/task1_result.json) |
| 零切换对照 | s3_c0_none | 启动失败：任务未开始 | 较早；需回归 | 不适用／未完成 | — | [20261101_231321_task1_s3_c0_none](../../runs/20261101_231321_task1_s3_c0_none/task1_result.json) |
| 零切换对照 | s3_c0_cross | 启动失败：任务未开始 | 较早；需回归 | 不适用／未完成 | — | [20261101_231507_task1_s3_c0_cross](../../runs/20261101_231507_task1_s3_c0_cross/task1_result.json) |
| 零切换对照 | s3_c0_block | 启动失败：任务未开始 | 较早；需回归 | 不适用／未完成 | — | [20261101_231654_task1_s3_c0_block](../../runs/20261101_231654_task1_s3_c0_block/task1_result.json) |
| 异常响应 | fault_observation_gap | 通过 | 较早；需回归 | 不适用／未完成 | 6.364 m | [20261101_202759_task1_s1_c0_none_observation_gap](../../runs/20261101_202759_task1_s1_c0_none_observation_gap/task1_fault_result.json) |
| 异常响应 | fault_reordered_observation | 通过 | 较早；需回归 | 不适用／未完成 | 6.309 m | [20261101_202914_task1_s1_c0_none_reordered_observation](../../runs/20261101_202914_task1_s1_c0_none_reordered_observation/task1_fault_result.json) |
| 异常响应 | fault_mission_exit | 通过 | 较早；需回归 | 不适用／未完成 | 6.341 m | [20261101_203029_task1_s1_c0_none_mission_exit](../../runs/20261101_203029_task1_s1_c0_none_mission_exit/task1_fault_result.json) |
| 异常响应 | fault_cancel | 通过 | 较早；需回归 | 不适用／未完成 | 6.570 m | [20261101_203143_task1_s1_c0_none_cancel](../../runs/20261101_203143_task1_s1_c0_none_cancel/task1_fault_result.json) |
| 异常响应 | fault_role_change | 通过 | 较早；需回归 | 不适用／未完成 | 6.440 m | [20261101_203255_task1_s1_c0_none_role_change](../../runs/20261101_203255_task1_s1_c0_none_role_change/task1_fault_result.json) |
| 异常响应 | fault_no_path | 通过 | 较早；需回归 | 不适用／未完成 | 7.164 m | [20261101_203408_task1_s1_c0_none_no_path](../../runs/20261101_203408_task1_s1_c0_none_no_path/task1_fault_result.json) |
| 异常响应 | fault_late_change | 通过 | 较早；需回归 | 不适用／未完成 | 1.153 m | [20261101_203521_task1_s1_c0_none_late_change](../../runs/20261101_203521_task1_s1_c0_none_late_change/task1_fault_result.json) |
| 异常响应 | fault_traffic_gap | 通过 | 当前 | 不适用／未完成 | 6.121 m | [20261102_173517_task1_harbor_s1_c0_none_traffic_gap](../../runs/20261102_173517_task1_harbor_s1_c0_none_traffic_gap/task1_fault_result.json) |
| 异常响应 | fault_traffic_exit | 通过 | 当前 | 不适用／未完成 | 6.070 m | [20261102_173704_task1_harbor_s1_c0_none_traffic_exit](../../runs/20261102_173704_task1_harbor_s1_c0_none_traffic_exit/task1_fault_result.json) |
| 额外压力测试 | s1_c2_none | 通过 | 较早；需回归 | 0.378 m / 0.049 rad | 1.021 m | [20261101_182043_task1_s1_c2_none](../../runs/20261101_182043_task1_s1_c2_none/task1_result.json) |
| 额外压力测试 | s1_c2_cross | 失败 | 较早；需回归 | 不适用／未完成 | 0.720 m | [20261101_213330_task1_s1_c2_cross](../../runs/20261101_213330_task1_s1_c2_cross/task1_result.json) |
| 额外压力测试 | s2_c2_none | 通过 | 较早；需回归 | 0.373 m / 0.051 rad | 1.216 m | [20261101_185028_task1_s2_c2_none](../../runs/20261101_185028_task1_s2_c2_none/task1_result.json) |
| 额外压力测试 | s3_c2_none | 通过 | 较早；需回归 | 0.378 m / 0.041 rad | 1.105 m | [20261101_192014_task1_s3_c2_none](../../runs/20261101_192014_task1_s3_c2_none/task1_result.json) |
| 港区工程测试 | harbor_s1_c1_none | 通过 | 较早；需回归 | 0.369 m / 0.086 rad | 1.225 m | [20261102_001502_task1_harbor_s1_c1_none](../../runs/20261102_001502_task1_harbor_s1_c1_none/task1_result.json) |
| 港区工程测试 | harbor_s1_c1_cross | 通过 | 当前 | 0.245 m / 0.155 rad | 1.307 m | [20261102_171813_task1_harbor_s1_c1_cross](../../runs/20261102_171813_task1_harbor_s1_c1_cross/task1_result.json) |
| 港区工程测试 | harbor_s1_c1_none_wind2_wave0.08 | 通过 | 较早；需回归 | 0.375 m / 0.216 rad | 1.282 m | [20261102_003413_task1_harbor_s1_c1_none_wind2_wave0.08](../../runs/20261102_003413_task1_harbor_s1_c1_none_wind2_wave0.08/task1_result.json) |
| 港区工程测试 | harbor_s1_c1_block | 通过 | 当前 | 0.371 m / 0.044 rad | 1.272 m | [20261102_170326_task1_harbor_s1_c1_block](../../runs/20261102_170326_task1_harbor_s1_c1_block/task1_result.json) |

额外观测距离为非默认实验，不替代上面的默认配置验收：

- `s1_c2_cross_visibility_guard`：失败；[独立结果](../../runs/20261101_214702_task1_s1_c2_cross/task1_result.json)。

临近浮标灯变测试只测注入后 10 s 的停止和规则响应；记录末端运动状态为 `PLANNING`。若为 STOPPING / PLANNING / WAIT_CLEAR，只能证明安全停等，不能声称已恢复航行或完成整场。

模型和感知检查：

- 相机灯态/实体：通过；[最近证据](../../runs/20261101_204127_task1_model_final_recheck/model_validation.json)。

- LiDAR 三层回波：通过；[最近证据](../../runs/20261101_190020_task1_s3_c0_none/lidar_validation.json)。

固定固件、规则及范围见 [implementation.md](implementation.md)，启动命令见 [runbook.md](runbook.md)，根因与修复见 [debug_log.md](debug_log.md)。
WAM-V、固定系泊模型和本地报告不能代表正式 UAV 协同、RoboCommand 接入或实船比赛得分。
