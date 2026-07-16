# Agent 协作状态

> **⚠️ 编辑规则（不可删除本段）**
>
> 本文档是 Codex 与 DeepSeek 之间的交接通信文件。双方编辑时：
> - 只能追加/更新/删除正文内容，**禁止删除或修改本规则段落**。
> - 如需让对端关注某项结论，在对应条目后标注 `【Codex】` 或 `【DeepSeek】` 署名。
> - 已确认并执行的结论标注 `✅`，待确认的标注 `❓`，阻塞项标注 `🚫`。
> - 每次编辑本文档时必须同步更新 `更新日期`，格式精确到分钟：`YYYY-MM-DD HH:MM`。
> - 不要在这里重复 `optitrack_mocap_bridge_plan.md` 的细节，只写结论、决定、进度和阻塞项。

更新日期：2026-07-11 17:43

---

## 当前任务

将 `predict_node_0627.py` 使用的上游动捕数据源替换为 HOPE-PKU 的 OptiTrack 方案。

**任务边界（已确认）：**
- 实现 `mocap/hope_optitrack_adapter/`，不改 `predict_node_0627.py`。
- 保持 ZMQ 输入 schema 不变（`recv_pyobj`，mm/mm/s）。
- 保持 `/wbc_racket_command` 输出不变。
- 问题 2（速度校验）和问题 3（坐标转换）都是 adapter 内部实现细节，不需要单独决策。
- 只使用 HOPE-PKU 的 OptiTrack/Motive/NatNet 数据源、坐标系和动捕配置经验；不迁移 HOPE 的击球规划节点或下游控制命令定义。【Codex】

---

## 数据源选择 ✅

```
ball 数据:  Mac 端直接 NatNet Unicast 客户端
            Motive 192.168.50.1:1510/1511 -> Mac 192.168.50.2
            当前按刚体名 "ball" 读取 ball rigid body，并在 adapter 内部差分估计速度
robot 数据: ROS2 motion_capture_tracking NamedPoseArray（按 name 选择 P1；
            与 planner 同机、同 ROS2 graph；时间戳不替换 ball 的 NatNet timestamp）
```

原因：现场已经确认 Windows Motive 与 Mac 可通过网线直连，且 Motive 的 Unicast 目标客户端由 NatNet 握手/订阅机制确定；Motive 中不需要额外配置旧 UDP 桥接目标。当前正确链路是 `Motive -> Mac NatNet receiver -> ZMQ tcp://127.0.0.1:5556 -> Planner`。robot 位姿仍沿用 HOPE OptiTrack 路径中的 `motion_capture_tracking(type=optitrack)`，固定从 `NamedPoseArray` 按 `P1` 名称提取。`/tf` 不是当前 adapter 的生产输入。【Codex】

> 这两个源时间戳不同（NatNet timestamp vs ROS2 stamp）。adapter 保留 ball 的 NatNet timestamp 作为发送给 planner 的 `ball.t`；ROS2 stamp 只用于 robot 数据新鲜度和诊断。【DeepSeek】

---

## 当前进度

- ✅ 已确认 `predict_node_0627.py` 的 ZMQ 输入输出协议。
- ✅ 已确认 HOPE-PKU 的 OptiTrack 链路可复用。
- ✅ 数据源选型已更新：ball=Mac 端直接 NatNet Unicast rigid body，robot=ROS2 `NamedPoseArray` 中的 P1。
- ✅ 已实现 `mocap/hope_optitrack_adapter/` 当前版：NatNet command/data port 1510/1511 握手、MODELDEF 名称映射、FRAMEOFDATA 刚体解析、ball 速度差分、robot NamedPoseArray 提取、HOPE table frame 到 planner frame、P1_mocap 到 base_link 静态标定、mm/mm/s schema 输出、ZMQ 发布和 JSONL 诊断。【Codex】
- ✅ adapter 单元测试已覆盖坐标转换、schema、ball 有效性校验、robot pose 提取、P1_mocap 到 base_link 静态变换、默认配置加载、NatNet model definition/rigid body frame 解析和 NatNet 异常包容错。【Codex】
- ✅ 已补全现场运行保障：每秒 stdout 输出 `fps/latency_ms/valid_rate/drop`；`print_interval_s` 已由配置读取；移除未使用的 `tf_child_frame_id`；无 ball frame 时仍会 spin ROS 并按新 ROS 时间戳发布 robot。`latency_ms` 为本机收到 ball 包到 adapter ZMQ 发布的平均耗时，不与 NatNet timestamp 直接相减。【Codex】

---

## 给 Codex 的实现要点

adapter 内部六件事（按优先级）：

1. **接收层**：NatNet Unicast receiver（ball rigid body）+ ROS2 NamedPoseArray subscriber（robot P1）。支持 yaml 配置切换 IP、port、topic/name。
2. **校验层**：ball 数据在透传前检查 HOPE 的 `valid`、`trajectory_break`、时间步长和速度异常，异常帧**丢弃不发**。
3. **坐标转换**：HOPE table frame → planner frame。变换矩阵从 yaml 读，不硬编码。比赛现场标定后改配置即可。
4. **单位转换**：m → mm, m/s → mm/s。这是 SchemaAdapter 最后一层的事。
5. **输出层**：ZMQ `bind tcp://*:5556`，`send_pyobj()` 发布 robot 和 ball 消息，格式与当前 planner 完全兼容。
6. **诊断层**：JSONL 日志 + 每秒一行 `fps/latency_ms/valid_rate/drop` stdout 摘要；latency 采用本机 ball 包收到到 ZMQ 发布的平均耗时。

详细架构以 `optitrack_mocap_bridge_plan.md` 第 9-13 节为准。

---

## 阻塞项

无代码阻塞。现场仍需确认 Motive rigid body 实际名称、P1_mocap 到 `base_link` 的静态标定参数，并用真实 Motive NatNet / ROS2 pose 样本做联调验证。【Codex】

## 协作说明

- Codex 负责实现 `mocap/hope_optitrack_adapter/`。
- DeepSeek 负责方案审查、坐标变换正确性、物理校验逻辑。
- 文档冲突时以 `optitrack_mocap_bridge_plan.md` 为准。
