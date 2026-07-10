# Agent 协作状态

更新日期：2026-07-10

本文档用于 Codex 与 DeepSeek 之间的简短交接沟通。详细方案仍以 `planner/optitrack_mocap_bridge_plan.md` 为准。

## 当前任务

将 `predict_node_0627.py` 使用的上游动捕数据源替换为 HOPE-PKU 的 OptiTrack 方案。

任务边界：

- 修改上游 mocap data source / bridge。
- 暂不改 `predict_node_0627.py` 的 planner 具体实现逻辑，除非后续证明存在 bug。
- 保持 `predict_node_0627.py` 当前 ZMQ 输入 schema 不变。
- 保持下游 `/wbc_racket_command` 行为不变。

当前 planner 输入 schema：

```text
robot: {"type": "robot", "pos": [x_mm, y_mm, z_mm], "quat": [qx, qy, qz, qw]}
ball:  {"type": "ball", "t": seconds, "pos": [x_mm, y_mm, z_mm], "vel": [vx_mm_s, vy_mm_s, vz_mm_s]}
```

## 当前结论

HOPE-PKU 仓库中确实包含 OptiTrack 数据源路径。

推荐链路：

```text
OptiTrack Cameras
-> Motive
-> NatNet
-> HOPE-PKU NatNet bridge / HOPE mocap relay
-> HOPE table-frame data
-> thin schema adapter
-> current ZMQ tcp://*:5556
-> predict_node_0627.py
```

因此本项目应直接复用 HOPE-PKU 的 OptiTrack / Motive / NatNet 方案，不应在本项目中从零重写 NatNet receiver。

## 当前进度

- Codex 已检查 `predict_node_0627.py`，确认其上游输入是 ZMQ `recv_pyobj()`。
- Codex 已检查 `XuWuLingYu/HOPE-PKU`，发现两类可用 HOPE 数据接口：
  - HOPE ROS 2 mocap topics：`/poses`、`/ball/point`、`/P1/pose`、`/P2/pose`。
  - HOPE NatNet Ball Viewer：C++ NatNet bridge 输出 UDP JSON，适合复用球的位置、速度和诊断字段。
- Codex 已更新 `planner/optitrack_mocap_bridge_plan.md`，将主方案改为复用 HOPE-PKU，并只新增薄适配层。
- 当前推荐：
  - ball：优先使用 HOPE UDP JSON。
  - robot pose：使用 HOPE ROS `/P1/pose`，或扩展 HOPE UDP bridge 输出完整 P1/P2 pose。

## 关键假设

- HOPE table frame 与当前 planner 隐含坐标系不同，adapter 必须做坐标和单位转换。
- 当前 planner 的 ZMQ 输入使用 mm 和 mm/s；HOPE 参考数据使用 m 和 m/s。
- HOPE UDP JSON 对 ball state 较完整，但当前 `rigid_bodies` 字段可能只有 id/name/tracked，不包含完整机器人位姿。
- 实验室真实 Motive session 仍需确认 object name、streaming id、streaming mode、timestamp、marker 来源和坐标轴方向。

## 给 DeepSeek 的待确认问题

- robot pose 应从 HOPE ROS `/P1/pose` 读取，还是扩展 HOPE UDP bridge 以保持 adapter 全 UDP？
- 第一版 adapter 应直接使用 HOPE 输出的 ball velocity，还是为了与当前 planner 稳定性一致而重新用位置历史估计速度？
- 真实实验环境中，HOPE table frame 到当前 planner frame 的最终坐标变换应如何固定？

## 后续规划

1. 在真实 OptiTrack 环境中跑通 HOPE-PKU NatNet bridge 或 HOPE ROS relay。
2. 记录一小段 ball 和 robot 数据样本。
3. 确认单位、时间戳、object name、marker 来源和坐标轴方向。
4. 实现 `mocap/hope_optitrack_adapter/`：
   - 接收 HOPE UDP JSON 和/或 HOPE ROS topics；
   - 将 HOPE table frame 转换到当前 planner frame；
   - 将 m/m/s 转换为 mm/mm/s；
   - 发布当前 ZMQ `send_pyobj()` schema。
5. 先用 `predict_node_0627.py --render` 验证，再连接 WBC。

## 协作说明

- Codex 负责维护本地计划文档和后续 adapter 实现。
- DeepSeek 重点审查数据源选择、坐标变换、假设条件和失败场景。
- 若 Codex 与 DeepSeek 对方案有分歧，应以真实 OptiTrack/HOPE 记录样本为准，不凭文档猜测。
