# OptiTrack Mocap Bridge 适配说明

## 1. 文档目的

本文用于给 Codex 说明当前 `predict_node_0627.py` 的架构、主要职责、输入输出协议，以及本次将动捕数据源替换为 OptiTrack / Motive / NatNet 的实现目标、推荐方案、注意事项和成功标准。

本次任务的核心不是重写乒乓球击球 planner，也不是修改 policy / WBC / controller，而是新增或替换上游 **mocap bridge**，让 OptiTrack 数据源输出与当前系统兼容的数据格式。

推荐原则：

```text
不改 planner 逻辑
不改 WBC / policy 接口
新增 OptiTrack -> ZMQ bridge
保持现有 ZMQ schema 兼容
先验证数据等价，再接入机器人执行
```

---

## 2. 当前 `predict_node_0627.py` 的定位

当前 `predict_node_0627.py` 可以理解为：

```text
基于动捕输入和物理模型的实时击球目标规划节点
Real-time physics-based strike planner
```

它不是 task planner，也不是 policy，也不是底层 controller。

### 2.1 它是 planner 的哪一层

当前系统没有完整的 task planner，因为系统目前：

- 不能根据战术选择打点；
- 默认打到对方半台中心附近；
- 不能判断或动态适应球旋转；
- 不能根据对手位置选择直线、斜线、快攻、挡球等策略。

因此当前代码更准确地说是：

```text
固定目标条件下的 strike planner
```

它负责根据来球状态计算：

```text
什么时候击球
在哪里击球
球拍击球瞬间应该具有怎样的位置和速度
```

下游的 WBC / policy / controller 再负责把这个高层目标转成实际机器人关节运动。

---

## 3. 当前文件整体架构

`predict_node_0627.py` 可以分为四个主要部分。

```text
0. 球拍指令反解模块
1. 球轨迹预测器
2. 可视化与误差诊断模块
3. 主程序：ZMQ 输入 -> 预测/规划 -> ROS2 指令发布
```

---

## 4. 主要模块说明

### 4.1 `_solve_v_outgoing(...)`

职责：

```text
在给定击球点、目标落点、期望飞行时间的条件下，求解出球速度 v_outgoing。
```

特点：

- 使用 Newton-like iteration；
- 先用抛物线近似初始化；
- 如果传入 predictor，则使用包含阻力、固定 spin_az 和重力的完整预测模型修正；
- 输出的是球被击出后的目标速度，不是球拍速度。

---

### 4.2 `calculate_racket_command(...)`

职责：

```text
根据来球速度、期望出球速度、恢复系数 Cr，反解球拍需要的速度。
```

输入核心参数：

```text
p_racket      击球点位置
v_incoming    击球瞬间来球速度
p_landing     目标落点
dt_flight     击球后到目标落点的期望飞行时间
Cr            球拍-球碰撞恢复系数
predictor     轨迹预测器，可选
```

输出：

```text
v_racket_cmd  世界坐标系下的球拍目标速度
```

该函数属于 strike planner 的核心逻辑。

---

### 4.3 `FastTrajectoryPredictor`

职责：

```text
根据当前球位置和速度预测未来轨迹、反弹和穿越击球平面的状态。
```

包含的物理模型：

- 质量 `m`；
- 重力 `g`；
- 固定的上旋等效竖直加速度 `spin_az`；
- x/y/z 方向阻力系数；
- 球桌高度；
- 桌面反弹参数 `k` 和 `b`。

关键方法：

#### `predict_state_at_time(p0, v0, t)`

根据初始位置和速度，预测 t 秒后的球位置和速度。

#### `predict_hitting_point(p_init, v_init, target_x, max_time, dt)`

预测球何时穿越固定击球平面 `target_x`。

输出：

```text
hit_pos   球穿越 target_x 平面时的位置
hit_vel   球穿越 target_x 平面时的速度
hit_t     距离穿越击球平面的剩余时间
```

这是当前 planner 判断击球时机和击球点的核心。

#### `compute_full_trajectory(...)`

用于可视化，生成预测轨迹点序列。

---

### 4.4 `TrajectoryVisualizer`

职责：

```text
可选的实时 3D 轨迹显示与预测误差分析。
```

主要功能：

- 显示预测轨迹；
- 显示真实轨迹；
- 显示预测击球点；
- 记录每帧预测结果；
- 在球穿越击球平面后，根据真实穿越点计算位置误差和时间误差；
- 绘制 time-to-strike 与预测误差曲线。

该模块用于调试和验证，不应在 OptiTrack 适配中改动核心逻辑。

---

### 4.5 `main(...)`

主程序职责：

```text
连接上游 ZMQ 动捕数据源
创建 ROS2 publisher
持续接收 robot / ball 数据
根据 ball 数据预测击球点和击球时刻
计算球拍目标位置和目标速度
持续发布 14 维 /wbc_racket_command
```

当前 ZMQ 输入：

```text
socket.connect("tcp://127.0.0.1:5556")
```

当前 ROS2 输出：

```text
/wbc_racket_command
std_msgs.msg.Float64MultiArray
Size = 14
```

---

## 5. 当前 planner 的输入协议

当前 `predict_node_0627.py` 只消费已经标准化后的 ZMQ 数据。它并不直接调用任何青瞳、OptiTrack、NatNet、Vicon 或其他动捕 SDK。

### 5.1 robot 消息

当前 planner 期望收到类似结构：

```python
{
    "type": "robot",
    "pos": [x_mm, y_mm, z_mm],
    "quat": [qx, qy, qz, qw]
}
```

语义：

```text
pos   机器人底座在世界坐标系下的位置，单位应为 mm
quat  机器人底座姿态四元数，当前约定应保持 [qx, qy, qz, qw]
```

注意：当前 planner 中 robot pos 会被除以 1000 转成米。

---

### 5.2 ball 消息

当前 planner 期望收到类似结构：

```python
{
    "type": "ball",
    "t": mocap_time_s,
    "pos": [x_mm, y_mm, z_mm],
    "vel": [vx_mm_s, vy_mm_s, vz_mm_s]
}
```

语义：

```text
t    动捕时间戳，单位秒，必须单调递增
pos  球在世界坐标系下的位置，单位 mm
vel  球在世界坐标系下的速度，单位 mm/s
```

注意：当前 planner 中 ball pos 和 vel 都会被除以 1000 转成 m 和 m/s。

---

## 6. 当前 planner 的输出协议

当前节点持续发布 14 维 `Float64MultiArray` 到：

```text
/wbc_racket_command
```

数据顺序：

```text
0-2    robot base position: x, y, z
3-6    robot base quaternion: qx, qy, qz, qw
7      t_strike
8-10   racket target position relative to robot/base reference
11-13  racket target velocity relative to robot/base reference
```

可以表示为：

```python
[
    base_x, base_y, base_z,
    qx, qy, qz, qw,
    t_strike,
    p_racket_x, p_racket_y, p_racket_z,
    v_racket_x, v_racket_y, v_racket_z,
]
```

当前存在两种发布语义：

```text
default 模式：首次有效预测前，发布默认球拍目标，t_strike = -0.9
pred 模式：首次有效预测后，持续发布预测目标，t_strike 随时间递减，最低夹到 -0.9
```

---

## 7. 当前硬编码策略参数

当前代码中存在一些固定策略参数，说明它不是完整 task planner：

```python
TARGET_HIT_X = -1.47
LOCK_TIME_THRESHOLD = 0.62
ROBOT_BASE_ORIGIN_WORLD = np.array([-1.87, 0.0, -0.76])
p_landing_target = np.array([0.685, 0.0, 0.02])
desired_flight_time = 0.6
```

含义：

```text
TARGET_HIT_X         固定击球平面
LOCK_TIME_THRESHOLD  当预测剩余时间小于阈值时开始锁定/更新击球目标
p_landing_target     固定回球落点，近似对方半台中心附近
desired_flight_time  击球后到落点的期望飞行时间
```

本次 OptiTrack 适配不应修改这些 planner 参数，除非后续单独确认需要校准坐标系或策略。

---

## 8. OptiTrack 适配目标

本次目标：

```text
将当前旧动捕数据源替换为 OptiTrack / Motive / NatNet，
但保持 predict_node_0627.py 的输入 schema 不变。
```

推荐目标链路：

```text
OptiTrack Cameras
        ↓
Motive
        ↓ NatNet
HOPE-PKU NatNet bridge / HOPE mocap relay
        ↓ HOPE table-frame m/m/s data
thin_schema_adapter
        ↓ current ZMQ PUB tcp://*:5556 (send_pyobj, mm/mm/s)
predict_node_0627.py
        ↓ ROS2 /wbc_racket_command
WBC / Policy / Controller
```

也就是说，数据源层直接采用 HOPE-PKU 的 OptiTrack / Motive / NatNet 方案，不再重新设计一套独立的 OptiTrack 数据源。当前项目只需要补一层薄适配：

```text
HOPE format -> current predict_node_0627.py ZMQ schema
```

也就是说，新建或替换的是：

```text
schema adapter / relay 层
```

而不是：

```text
planner
policy
WBC
controller
```

---

## 8.1 当前是否已有 OptiTrack / HOPE 具体数据格式

已检查 `XuWuLingYu/HOPE-PKU`。该仓库里有两类可以直接参考的具体格式：

```text
1. HOPE 正式 ROS 2 mocap 接口
   位置/姿态以 ROS 2 topic 形式进入 planner / WBC。

2. HOPE NatNet Ball Viewer 调试接口
   Windows 端 C++ NatNet bridge 将 ball / rigid body 诊断数据以 UDP JSON 发给 Python viewer。
```

但仍需注意：这些是 **HOPE 参考实现格式**，不是本实验室 OptiTrack 现场录制出的真实 session。实施前仍应记录一段本地 NatNet callback / bridge JSONL，确认 Motive 版本、object name、marker 来源、坐标轴和时间戳。

因此本项目应明确区分三层数据边界：

```text
1. NatNet 原始层
   Motive / NatNet frame
   包含 frame id、source timestamp、rigid body 数据、labeled / unlabeled marker 数据等。
   这一层依赖实际 Motive / NatNet 版本和 Streaming 配置，必须现场确认。

2. bridge 内部标准层
   项目内建议统一成 MocapFrame / JSONL 记录：
   - position_m / velocity_m_s 使用米制单位
   - quaternion_xyzw 使用 [qx, qy, qz, qw]
   - source_timestamp_s 用于速度估计和状态推进
   - received_monotonic_s 只用于延迟诊断
   - ball.semantics 明确标注 unlabeled_marker / labeled_marker / rigid_body

3. planner 兼容层
   为了不改 predict_node_0627.py，最终 ZMQ 仍需输出旧 schema：
   - robot.pos: [x_mm, y_mm, z_mm]
   - robot.quat: [qx, qy, qz, qw]
   - ball.pos: [x_mm, y_mm, z_mm]
   - ball.vel: [vx_mm_s, vy_mm_s, vz_mm_s]
   - ball.t: seconds
```

也就是说，本计划已有的是**项目建议的标准化格式**，不是已经验证过的**实验室 OptiTrack 原始格式**。实施前必须先用官方 NatNet sample 或 bridge 日志记录一段真实 session，再确认：

```text
Motive version / NatNet version
streaming mode: unicast or multicast
frame id 是否连续
source timestamp 单位和单调性
rigid body 名称、streaming id、tracked flag
球的数据来源：unlabeled marker / labeled marker / rigid body
球 marker 是否等价于球心
Motive world -> planner/table frame 的坐标变换
```

---

### 8.1.1 HOPE 正式 ROS 2 mocap 接口

HOPE-PKU 的正式 mocap 接口不是 ZMQ，而是 ROS 2 topic。关键文件：

```text
mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md
docs/interfaces/ros_topics.md
docs/interfaces/frames.md
hope_ws/src/hope_bringup/scripts/avatar_pro_vrpn_relay
hope_ws/src/hope_planner/hope_planner/node.py
hope_ws/src/hope_planner/config/hope_planner.yaml
```

HOPE 标准话题：

```text
/poses       geometry_msgs/PoseArray
/ball/point  geometry_msgs/PointStamped
/table/pose  geometry_msgs/PoseStamped
/P1/pose     geometry_msgs/PoseStamped
/P2/pose     geometry_msgs/PoseStamped
/tf          tf2_msgs/TFMessage
```

`/poses` 的默认顺序由 `pose_array_order` 指定：

```yaml
pose_array_order: ["ball", "PPT", "P1", "P2"]
```

因此 HOPE planner 默认：

```yaml
ball_pose_index: 0
```

HOPE planner 实际消费的是：

```python
PoseArray /poses
pose = msg.poses[ball_pose_index]
p_ball = [pose.position.x, pose.position.y, pose.position.z]
t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
```

单位和时间语义：

```text
position: m
velocity: 由 planner 内部 BallStateEstimator 根据最近位置和时间戳拟合得到，单位 m/s
timestamp: ROS header stamp，秒
frame_id: world
```

HOPE 的 `RacketCommand` 输出消息为：

```text
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 normal
float64 strike_time
float64 time_to_strike
geometry_msgs/Vector3 ball_velocity_outgoing
bool valid
bool clears_net
bool bypasses_net_posts
int32 predicted_bounces
```

这与本项目当前 `/wbc_racket_command` 的 14 维 `Float64MultiArray` 不同。本次数据源替换仍应保持当前 14 维输出不变。

---

### 8.1.2 HOPE 坐标系与当前 planner 坐标系差异

HOPE canonical frame：

```text
origin: P1 近端左侧桌面角点
X: 从 P1 指向 P2，沿球台长轴
Y: 从 P1 视角沿球台宽度，左角为 0，右侧为 -1.525
Z: 向上，桌面 z = 0
table: x in [0, 2.74], y in [-1.525, 0]
x_hit: 0.0
target_land: [2.055, -0.7625, 0.0]
```

当前 `predict_node_0627.py` 的隐含 frame 更接近“球台中心 frame”：

```text
table x: [-1.37, 1.37]
table y: [-0.7625, 0.7625]
TARGET_HIT_X = -1.47
p_landing_target = [0.685, 0.0, 0.02]
来球时 vx < -1.0
```

如果直接复用 HOPE frame 输入当前 planner，击球平面和落点都会错。若不改 planner 逻辑，应在 bridge / relay 中做 HOPE table frame -> 当前 planner frame 的转换：

```python
x_planner = x_hope - 1.37
y_planner = y_hope + 0.7625
z_planner = z_hope + z_offset

vx_planner = vx_hope
vy_planner = vy_hope
vz_planner = vz_hope
```

其中 `z_offset` 需要按当前 planner 的桌面高度约定确认。当前代码使用 `table_height = 0.02`，因此若 HOPE 的桌面为 `z = 0` 且不改 planner 参数，初版可用：

```text
z_offset = 0.02
```

但这应通过静态球/桌面验证确认。

---

### 8.1.3 HOPE NatNet Ball Viewer UDP JSON 调试格式

HOPE-PKU 中 `mocap/tools/natnet_ball_viewer/src/bridge.cpp` 实现了一个 Windows 侧 C++ NatNet -> UDP JSON bridge。它不是正式 planner 输入，但字段非常适合作为本项目 OptiTrack bridge 的诊断格式参考。

默认网络配置：

```ini
server=192.168.43.228
local=192.168.43.128
connection=multicast
multicast=239.255.42.99
paddle_name=qiupai
paddle_id=-1
marker_index=-1
udp_host=127.0.0.1
udp_port=38999
```

UDP JSON 顶层字段：

```json
{
  "frame": 123,
  "timestamp": 12.345678,
  "dt": 0.002778,
  "valid": true,
  "trajectory_break": false,
  "smoothing_window": 5,
  "selected": 0,
  "missed_frames": 0,
  "candidate_count": 1,
  "source": "other",
  "table_valid": true,
  "ppt_marker_count": 4,
  "position": [0.5, -0.7, 0.3],
  "velocity": [-3.0, 0.1, 0.2],
  "acceleration": [0.0, 0.0, -9.8],
  "world_position": [1.2, 0.4, 0.9],
  "smoothed_position": [0.5, -0.7, 0.3],
  "world_smoothed_position": [1.2, 0.4, 0.9],
  "world_velocity": [-3.0, 0.1, 0.2],
  "world_acceleration": [0.0, 0.0, -9.8],
  "rigid_bodies": [
    {"id": 1, "name": "PPT", "tracked": true}
  ],
  "qiupai": {
    "present": true,
    "tracked": true,
    "id": 2,
    "mean_error": 0.001,
    "position": [0.0, 0.0, 0.0],
    "world_position": [0.0, 0.0, 0.0],
    "x_axis": [0.25, 0.0, 0.0],
    "y_axis": [0.0, 0.25, 0.0],
    "z_axis": [0.0, 0.0, 0.25],
    "markers": []
  },
  "score": -120.0,
  "confidence": 120.0,
  "suppressed": false,
  "filter_reason": "",
  "nearest_keypoint_distance": 0.3,
  "quality": 10,
  "residual": 0.001,
  "marker_id": 42
}
```

字段语义：

```text
frame       NatNet frame id
timestamp   NatNet fTimestamp
dt          优先由连续 NatNet timestamp 相减得到
position    已转换到 HOPE table frame 的 ball position，单位 m
velocity    已转换到 HOPE table frame 的 ball velocity，单位 m/s
acceleration 已转换到 HOPE table frame 的 ball acceleration，单位 m/s^2
world_*     Motive / NatNet 原始 world frame 下的值
source      "labeled" 或 "other"；"other" 对应 NatNet OtherMarkers / unlabeled 点云
selected    当前选中的候选 index；-1 表示无有效候选
```

注意：`qiupai` 是 HOPE viewer 调试用 paddle rigid body。HOPE 正式比赛设计要求 racket/paddle 不由 mocap 闭环追踪，因此本项目可以借鉴其“排除 paddle marker 泄漏”的逻辑，但不应把 `qiupai` 作为生产控制输入。

---

### 8.1.4 HOPE 球 marker 语义与筛选逻辑

HOPE 推荐球作为 **single unlabeled marker**，而不是刚体：

```text
Ball: single unlabeled marker
位置: [x, y, z] 3-DOF
姿态: 无意义，忽略
速度: 由位置序列估计
```

球的物理准备：

```text
推荐：全反光球 / 全表面 retroreflective coating
不推荐：多个离散反光贴片
```

原因：

```text
全反光球通常重建成接近球心的单点，spin-invariant。
多贴片会产生多个表面点，离球心约 20 mm，且旋转时会跳点。
```

HOPE NatNet bridge 的筛选逻辑：

```text
1. 优先从 NatNet LabeledMarkers 中提取带 unlabeled 标志的 marker；
2. 若没有，则使用 OtherMarkers；
3. 候选点转换到 table frame 后检查是否在合理 play volume；
4. 抑制距离 PPT table keypoints < 0.045 m 的点；
5. 抑制距离 qiupai marker keypoints < 0.045 m 的点；
6. 选择 confidence 最高的未抑制候选；
7. 5 帧 moving window 平滑 position；
8. velocity = smoothed position 差分 / dt；
9. velocity 再用 5 帧 moving window 平滑；
10. 若 frame-to-frame jump > 0.30 m，标记 trajectory_break 并重置窗口。
```

这些规则比当前文档原先的“最近 3-5 帧线性拟合”更贴近 HOPE-PKU 的实际实现。对本项目而言，可以直接采用“候选筛选 + 诊断字段”的思路；速度估计方法可在 HOPE moving-window 差分和当前 planner 更稳的多项式拟合之间选择。

---

## 9. 推荐适配架构

既然 HOPE-PKU 已经提供 OptiTrack / Motive / NatNet 参考链路，本项目不应从零实现完整 NatNet bridge。推荐新增独立薄适配模块：

```text
mocap/hope_optitrack_adapter/
```

内部模块建议如下：

```text
HopeUdpJsonReceiver 或 HopeRosReceiver
CoordinateAdapter
SchemaAdapter
ZmqPublisher
DiagnosticsLogger
```

数据流：

```text
HOPE-PKU NatNet bridge / HOPE ROS relay
        ↓
HOPE table-frame ball + robot data
        ↓
CoordinateAdapter
        ↓
SchemaAdapter
        ↓
ZmqPublisher
        ↓
tcp://*:5556
```

可选输入源：

```text
方案 A：HOPE NatNet Ball Viewer UDP JSON
  C++ NatNet bridge -> UDP JSON 127.0.0.1:38999
  适合最快复用 OptiTrack/NatNet 数据和诊断字段。
  注意：当前 HOPE UDP JSON 对 ball 很完整，但 rigid_bodies 只含 id/name/tracked；
       若当前 ZMQ schema 需要 robot base pos/quat，需同时读取 HOPE ROS /P1/pose，
       或扩展 HOPE C++ bridge 输出 P1/P2 pose。

方案 B：HOPE ROS 2 topic
  /poses, /ball/point, /P1/pose, /P2/pose
  适合后续整体迁移到 ROS 2 mocap graph。
```

当前项目若目标是最小改动接上 `predict_node_0627.py`，优先：

```text
ball:  HOPE UDP JSON
robot: HOPE ROS /P1/pose 或扩展后的 HOPE UDP JSON
```

---

## 10. Adapter 模块职责

### 10.1 `HopeDataReceiver`

职责：

```text
接收 HOPE-PKU 已经标准化/诊断化后的动捕数据
支持 HOPE UDP JSON 或 HOPE ROS 2 topic
提取 ball、robot、timestamp、valid/diagnostic 字段
```

优先支持 HOPE UDP JSON：

```text
host: 127.0.0.1
port: 38999
source fields:
  frame
  timestamp
  valid
  position
  velocity
  rigid_bodies
```

可选支持 HOPE ROS 2 topic：

```text
/poses
/ball/point
/P1/pose
/P2/pose
```

NatNet / Motive 连接本身由 HOPE-PKU bridge 负责，本项目 adapter 不直接调用 NatNet SDK。

HOPE-PKU 参考配置使用：

```text
Motive v3.4
NatNet SDK v4.4
Command Port: 1510
Data Port: 1511
Multicast Address: 239.255.42.99
Rigid Bodies: ON
Unlabeled Markers: ON
Labeled Markers: OFF unless needed
Up Axis: Z Axis
```

其中 `Up Axis = Z Axis` 很关键；否则需要在 bridge 中做 Y-up -> Z-up 转换。

---

### 10.2 `SourceMapper`

职责：

```text
把 HOPE 数据中的对象语义映射到当前 planner schema。
```

例如：

```yaml
table:
  type: rigid_body
  name: PPT

robot:
  type: rigid_body
  name: P1

ball:
  source: unlabeled_marker
  selection: nearest_to_previous
```

或：

```yaml
robot:
  type: rigid_body
  id: 3

ball:
  type: rigid_body
  id: 7
```

不要在 adapter 里硬编码 Motive 的 rigid body array index；应通过 HOPE JSON/ROS 中的 name、topic 或配置选择 `P1` / `P2`。

HOPE-PKU 参考命名：

```text
PPT       ping-pong table rigid body / world anchor
P1, P2    humanoid base_link mocap rigid body
Ball      only when the ball is configured as a named rigid body
qiupai    paddle rigid body used by NatNet viewer diagnostics only
```

当前 `predict_node_0627.py` 只需要一个 `robot` pose。若采用 HOPE 命名，应在 bridge 配置中选择 `P1` 或 `P2` 映射到当前 schema 的 `robot`。

注意：乒乓球在 NatNet 中通常不是一个稳定命名的 `pingpong_ball` marker。现场更常见的形式是：

```text
unlabeled marker 点云
labeled marker
小球被定义成 rigid body
```

如果采用 HOPE UDP JSON，球候选选择已经由 HOPE C++ bridge 完成；adapter 只需检查 `valid`、`selected`、`candidate_count`、`trajectory_break`、`source` 等诊断字段。若改用 HOPE ROS topic，则需要额外确认 `/poses` 中 ball 的 index 或 `/ball/point` 的来源。

---

### 10.3 `CoordinateAdapter`

职责：

```text
把 HOPE table frame 转换为当前 planner 期望的世界坐标系。
```

当前 planner 隐含坐标语义：

```text
x：球台长轴方向
y：左右方向
z：高度方向
来球时 vx < -1.0
target_x = -1.47
```

坐标转换建议写成：

```python
p_planner = R_planner_from_hope @ p_hope + t_planner_from_hope
v_planner = R_planner_from_hope @ v_hope
```

姿态转换建议写成：

```python
q_planner_body = q_planner_from_hope * q_hope_body
```

如果机器人刚体自身 body frame 和旧系统 body frame 不一致，再增加：

```python
q_planner_body = q_planner_from_hope * q_hope_body * q_body_correction
```

注意：四元数顺序必须明确，不得凭感觉假设。

---

### 10.4 `BallStateAdapter`

职责：

```text
获取 HOPE 输出的球位置/速度，并决定是否直接转发给当前 planner。
```

如果采用 HOPE UDP JSON，`position` 和 `velocity` 已由 HOPE bridge 输出在 HOPE table frame 中，单位分别为 m 和 m/s。adapter 可直接做坐标/单位转换后发布。

如果采用 HOPE ROS topic，`/ball/point` 和 `/poses` 只提供位置；速度需要在 adapter 或 planner 中估计。

可选初版方法 A（简单、低延迟，接近 HOPE NatNet viewer 实现）：

```text
5 帧位置 moving average
velocity = smoothed_position 差分 / dt
5 帧速度 moving average
```

可选初版方法 B（更适合 planner 输入，抗噪更稳）：

```text
最近 3-5 帧位置 + 时间戳
线性拟合 p(t) = v * t + b
输出 v
```

如果直接使用 HOPE NatNet viewer 的 UDP JSON，初版不必重新估计速度，只需要记录 HOPE velocity 并在诊断中监控异常。

若 adapter 自己估计速度，必须使用 HOPE/NatNet source timestamp，不要使用本机 receive wall time。以下情况应重置或暂停速度估计：

```text
valid=False
球候选 source 改变
selected candidate id / index 改变
frame id 跳号
dt <= 0 或 dt 过小
dt 过大
位置跳变超过阈值
```

不建议第一版直接上复杂滤波。原因：

```text
复杂滤波会引入额外延迟和调参变量，
早期会干扰坐标系、单位和时间戳问题的定位。
```

无效数据处理：

```text
球丢失、跳变过大、dt 异常、速度量级异常时，不发布 ball 消息。
robot 消息可以继续发布。
```

不要发布假的零球速或零位置，否则会污染 planner 预测。

---

### 10.5 `SchemaAdapter`

职责：

```text
把 bridge 内部标准状态转换成当前 planner 兼容的 ZMQ schema。
```

建议 bridge 内部先保留米制标准状态，再在 SchemaAdapter 最后一层转换成 planner 兼容单位：

```text
position_m   -> pos mm
velocity_m_s -> vel mm/s
```

bridge 内部标准状态建议记录为 JSONL，便于离线回放和质量分析：

```json
{
  "source": "optitrack",
  "schema_version": 1,
  "frame_id": 12345,
  "source_timestamp_s": 12.345678,
  "received_monotonic_s": 9876.54321,
  "coordinate_frame": "motive_world_or_table",
  "ball": {
    "position_m": [0.12, -0.34, 0.78],
    "tracked": true,
    "semantics": "unlabeled_marker",
    "validated_as_center": false
  },
  "rigid_bodies": {
    "P1": {
      "position_m": [1.0, 0.0, 0.0],
      "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
      "tracked": true,
      "streaming_id": 2
    }
  }
}
```

这不是 planner 直接消费的格式。planner 直接消费的仍是下面的 ZMQ 兼容消息。

robot 输出：

```python
{
    "type": "robot",
    "t": mocap_time_s,
    "pos": [x_mm, y_mm, z_mm],
    "quat": [qx, qy, qz, qw],
    "valid": True,
}
```

ball 输出：

```python
{
    "type": "ball",
    "t": mocap_time_s,
    "pos": [x_mm, y_mm, z_mm],
    "vel": [vx_mm_s, vy_mm_s, vz_mm_s],
    "valid": True,
}
```

当前 planner 只读取必要字段，额外字段应不影响兼容性。

---

### 10.6 `ZmqPublisher`

职责：

```text
绑定 tcp://*:5556
发布 robot 和 ball 消息
```

非常重要：当前 `predict_node_0627.py` 使用的是：

```python
socket.recv_pyobj()
```

因此保持 planner 不变时，ZMQ wire format 必须是 Python pickle / `send_pyobj()` 兼容对象。HOPE UDP JSON 不能直接被当前 planner 读取，必须经过 Python adapter 转换。

可选方案：

```text
方案 A：HOPE UDP JSON -> Python adapter -> pyzmq send_pyobj()
方案 B：HOPE ROS topic -> Python adapter -> pyzmq send_pyobj()
方案 C：最小修改 planner 接收层，支持 JSON/msgpack；但这属于 planner 输入层改动
```

若目标是“不改 planner 逻辑”，推荐先采用方案 A：

```text
HOPE-PKU C++ NatNet bridge
        ↓ UDP JSON 127.0.0.1:38999
Python HOPE -> ZMQ adapter
        ↓ pyzmq send_pyobj()
predict_node_0627.py recv_pyobj()
```

当前 planner 是：

```text
connect tcp://127.0.0.1:5556
```

最小改动方案：

```text
bridge 和 planner 跑在同一台机器
bridge bind tcp://*:5556
planner connect tcp://127.0.0.1:5556
```

如果 bridge 跑在 Motive Windows 电脑，而 planner 跑在另一台代码机器，则需要：

```text
方案 A：把 planner 的 connect 地址改成 Motive 电脑 IP
方案 B：做端口转发，把远端 5556 映射到本地 127.0.0.1:5556
```

优先方案 A，便于排查。

---

### 10.7 `DiagnosticsLogger`

职责：

```text
记录 bridge 输入、输出、延迟、帧率和有效性。
```

建议记录为 JSONL：

```text
logs/mocap_bridge_YYYYMMDD_HHMMSS.jsonl
```

每帧记录：

```text
frame_id
mocap_time
receive_wall_time
publish_wall_time
robot_pos
robot_quat
ball_pos
ball_vel
valid flags
latency
drop count
fps
```

每秒打印一次：

```text
fps
robot_valid_rate
ball_valid_rate
latency_p50 / p95 / p99
drop_count
latest_ball_speed
```

---

## 11. 推荐目录结构

本项目推荐只新增 HOPE -> current ZMQ 的适配层：

```text
mocap/
  hope_optitrack_adapter/
    __init__.py
    main.py
    config.yaml
    hope_udp_json_receiver.py
    hope_ros_receiver.py
    coordinate_adapter.py
    schema_adapter.py
    zmq_publisher.py
    diagnostics.py

  tests/
    test_coordinate_adapter.py
    test_schema_adapter.py
```

不建议在本项目重复维护 C++ NatNet SDK 接收层。C++ NatNet 接收优先复用 HOPE-PKU：

```text
HOPE-PKU/mocap/tools/natnet_ball_viewer/src/bridge.cpp
HOPE-PKU/mocap/tools/natnet_ball_viewer/config/natnet_viewer.conf
HOPE-PKU/mocap/tools/natnet_ball_viewer/run_python_natnet_viewer.ps1
```

除非 HOPE bridge 无法满足现场需求，否则不要另写 `NatNetReceiver.cpp`。

---

## 12. 推荐配置文件

```yaml
motive:
  server_ip: "192.168.1.100"
  local_ip: "192.168.1.101"
  transmission_type: "unicast"

stream:
  publish_zmq: "tcp://*:5556"
  output_pos_unit: "mm"
  output_vel_unit: "mm/s"
  quat_order_to_planner: "xyzw"

assets:
  table:
    type: "rigid_body"
    name: "PPT"
    id: null

  robot:
    type: "rigid_body"
    name: "P1"
    id: null

  ball:
    source: "unlabeled_marker"
    selection: "nearest_to_previous"
    id: null
    max_jump_m: 0.8
    validated_as_center: false

coordinate:
  # If using HOPE table frame as source:
  # x_planner = x_hope - 1.37
  # y_planner = y_hope + 0.7625
  # z_planner = z_hope + z_offset
  source_frame: "hope_table"
  R_planner_from_source:
    - [1, 0, 0]
    - [0, 1, 0]
    - [0, 0, 1]
  t_planner_from_source_m: [-1.37, 0.7625, 0.02]

velocity:
  method: "moving_average_difference"  # or "linear_fit"
  window_size: 5
  max_dt_ms: 30
  max_speed_m_s: 20.0
  max_accel_m_s2: 300.0

diagnostics:
  log_jsonl: true
  log_dir: "logs"
  print_interval_s: 1.0
```

---

## 13. 主循环建议

乒乓球任务是实时系统，旧帧价值很低。建议使用：

```text
latest-frame wins
```

不要无限积压队列。

伪代码：

```python
def on_hope_udp_json(frame):
    latest_frame_buffer.write(frame)

while running:
    frame = latest_frame_buffer.read_latest()
    if frame is None:
        continue

    # HOPE UDP JSON path:
    # frame["position"] / frame["velocity"] are in HOPE table frame, m and m/s.
    # frame["timestamp"] is NatNet fTimestamp.
    # frame["rigid_bodies"] contains names/tracked flags; robot pose may need
    # the HOPE ROS topic path if the UDP JSON does not include P1/P2 full pose.
    if not frame.get("valid", False):
        continue

    ball_pos_hope = frame["position"]
    ball_vel_hope = frame["velocity"]
    timestamp = frame["timestamp"]

    ball_pos_planner = coordinate_adapter.hope_to_planner_position(ball_pos_hope)
    ball_vel_planner = coordinate_adapter.hope_to_planner_vector(ball_vel_hope)

    robot_state = robot_source.latest_robot_pose()  # e.g. P1/P2 from HOPE ROS or separate bridge field
    if robot_state is not None and robot_state.valid:
        zmq_pub.send(robot_schema(robot_state))

    zmq_pub.send(ball_schema(
        t=timestamp,
        pos_mm=ball_pos_planner * 1000.0,
        vel_mm_s=ball_vel_planner * 1000.0,
    ))

    diagnostics.update(...)
```

---

## 14. 实现步骤建议

### Step 1：定位旧 bridge

先在项目里找到当前是谁在向 `tcp://*:5556` 发布数据：

```bash
rg -n "5556|bind\(|send_pyobj|send_json|send_multipart" .
rg -n "ChingMu|青瞳|CM|xingying|形影|Tracker|RigidBody" .
rg -n "NatNet|Motive|OptiTrack" .
```

目标：

```text
确认旧 bridge 的输出 schema、单位、坐标系、四元数顺序和时间戳来源。
```

---

### Step 2：先跑通 HOPE-PKU NatNet Ball Viewer

目标：

```text
确认 Motive streaming 配置正确
确认 HOPE C++ NatNet bridge 能连接 Motive
确认 UDP JSON 中 frame / timestamp 连续
确认 position / velocity / valid 正常
确认 PPT rigid body 能被看到
确认 P1 或 P2 robot rigid body 能被看到
确认 ball 来源是 unlabeled marker / labeled marker / rigid body
记录一段 HOPE UDP JSON，作为本项目 adapter 的输入样例
```

仍建议先跑官方 NatNet sample 一次，用来分离 SDK / 防火墙 / Motive 配置问题；但正式接入路径直接采用 HOPE bridge。

---

### Step 3：实现最小 HOPE -> ZMQ adapter

第一版只做：

```text
读取 HOPE UDP JSON 或 HOPE ROS topic
HOPE table frame -> 当前 planner frame 坐标转换
m / m/s -> mm / mm/s
P1 或 P2 pose -> current robot schema
pyzmq send_pyobj() 发布当前 ZMQ schema
JSONL 诊断日志
```

暂时不要重新实现 NatNet 接收、复杂滤波、复杂异常恢复、复杂 planner 逻辑。

---

### Step 4：静态和手动移动验证

验证：

```text
机器人静止时 pos / quat 是否稳定
手动移动球时 x/y/z 方向是否符合 planner 约定
单位是否为 mm
速度方向和量级是否合理
四元数模长是否接近 1
```

---

### Step 5：接入 planner --render

运行：

```bash
python predict_node_0627.py --render
```

目标：

```text
可视化中能看到真实轨迹
预测轨迹方向合理
击球点位置合理
时间误差和位置误差曲线能正常输出
```

---

### Step 6：再接入 WBC / policy

只有在 `bridge -> planner --render` 验证通过后，才允许接入机器人执行。

---

## 15. 关键注意事项

### 15.1 单位

当前 planner 假设：

```text
输入 pos 单位：mm
输入 vel 单位：mm/s
内部除以 1000 变成 m 和 m/s
```

如果 bridge 输出已经是 m，planner 会再次除以 1000，导致坐标缩小 1000 倍。

HOPE-PKU 的正式 ROS 2 接口使用米：

```text
/poses PoseArray           position: m
/ball/point PointStamped   point: m
/table/pose PoseStamped    pose.position: m
/P1/pose PoseStamped       pose.position: m
/P2/pose PoseStamped       pose.position: m
```

HOPE NatNet viewer UDP JSON 也使用米和 m/s：

```text
position / world_position: m
velocity / world_velocity: m/s
acceleration / world_acceleration: m/s^2
```

因此若从 HOPE ROS topic 或 HOPE UDP JSON 接入当前 `predict_node_0627.py`，SchemaAdapter 必须在最后一步转换：

```text
m    -> mm
m/s  -> mm/s
```

验证方法：

```text
在 Motive 中移动刚体 10 cm
bridge 输出应变化约 100，而不是 0.1
```

---

### 15.2 坐标轴方向

当前 planner 强依赖：

```text
来球时 vx < -1.0
击球平面 target_x = -1.47
z 是高度
```

如果 x 轴方向错，planner 不会进入追踪逻辑。

HOPE frame 和当前 planner frame 的 x 方向可以保持一致，但原点不同：

```text
HOPE x=0      P1 近端边
HOPE x=1.37   球台中心/网线
HOPE x=2.74   P2 远端边

当前 planner x=-1.37  P1 近端边
当前 planner x=0      球台中心/网线
当前 planner x=1.37   P2 远端边
```

所以采用 HOPE 数据时，不做坐标平移会导致 `TARGET_HIT_X=-1.47` 永远不在正确位置。

验证方法：

```text
沿球台长轴向机器人移动球，检查 vx 是否为负
左右移动球，检查 y 是否变化
上下移动球，检查 z 是否增加
```

---

### 15.3 四元数顺序

当前输出到下游 WBC 的顺序是：

```text
qx, qy, qz, qw
```

必须确认 NatNet wrapper 输出顺序。如果 NatNet 或 wrapper 给的是：

```text
qw, qx, qy, qz
```

则必须在 bridge 中转换。

验证方法：

```text
quat norm ≈ 1
绕 z 轴旋转机器人刚体，观察 yaw 是否符合预期
```

---

### 15.4 时间戳

当前 planner 用 `ball.t` 计算：

```text
locked_absolute_hit_time = current_time + flight_time
t_strike = locked_absolute_hit_time - current_time
```

因此 `ball.t` 必须：

```text
单位为秒
单调递增
与 ball.pos / ball.vel 同源
不能混用 wall time 和 mocap time
```

---

### 15.5 球速度估计

速度比位置更敏感。若速度噪声大，会直接影响：

```text
击球点预测
击球时间预测
球拍目标速度反解
```

初版建议：

```text
3-5 帧线性拟合
异常点剔除
不要过度平滑
```

过度平滑会引入延迟，高速乒乓球任务中几十毫秒误差就可能导致击球失败。

---

### 15.6 不要污染 planner

本次不应随意改动：

```text
FastTrajectoryPredictor
calculate_racket_command
predict_hitting_point
/wbc_racket_command 输出格式
默认/pred 发布语义
```

除非明确发现旧数据源与 OptiTrack 坐标系差异导致必须将某些硬编码参数配置化。

---

## 16. 成功标准

### 16.1 Bridge 层成功标准

```text
1. Motive streaming 开启后，bridge 能稳定接收 robot 和 ball 数据
2. bridge 能持续发布兼容旧 schema 的 ZMQ 消息
3. robot.pos 静态稳定，位置无明显跳变
4. robot.quat 模长接近 1
5. ball.pos 坐标轴方向与 planner 约定一致
6. ball.vel 方向和量级合理
7. 时间戳单调递增，单位为秒
8. ball.semantics 已确认并记录，不把未验证的表面 marker 当球心
9. ZMQ wire format 与 planner 的 recv_pyobj() 兼容，或已明确存在 Python relay
10. 240 Hz 或目标帧率下无明显堆积延迟
11. 日志中能统计 fps、valid rate、latency、drop count
```

---

### 16.2 Planner 接入成功标准

```text
1. 不改或极少改 predict_node_0627.py 即可连接新 bridge
2. python predict_node_0627.py --render 能显示真实轨迹和预测轨迹
3. 来球时能进入 tracking / locked 状态
4. 能正常输出击球点 Y/Z、t_flight
5. 球穿越 target_x 后能输出位置误差和时间误差
6. /wbc_racket_command 持续发布 14 维数组
```

---

### 16.3 系统级成功标准

```text
1. bridge -> planner -> /wbc_racket_command 链路稳定
2. 默认模式和预测模式切换正常
3. 无 ball 消息时 planner 不崩溃
4. ball 短时丢失时不发布伪造零数据
5. 静态 robot pose 不抖动到影响 WBC
6. 延迟和帧率满足后续机器人控制需求
```

---

## 17. 推荐给 Codex 的实现边界

Codex 应优先做：

```text
新增 hope_optitrack_adapter
复用 HOPE-PKU NatNet Ball Viewer / HOPE ROS mocap relay
新增配置文件
新增日志
新增 schema / coordinate 的最小测试
保持 predict_node_0627.py 主体不变
```

Codex 不应主动做：

```text
重写 NatNet SDK 接收层
重构 predict_node_0627.py
修改物理预测模型
修改击球策略参数
修改 /wbc_racket_command 格式
修改 policy / WBC
加入复杂滤波或学习模型
删除旧 bridge 或旧方案
```

如果必须改 `predict_node_0627.py`，建议只做最小配置化，例如：

```text
把 ZMQ 地址从硬编码 tcp://127.0.0.1:5556 改为 CLI 参数或 config
不改变数据语义
不改变 planner 逻辑
```

---

## 18. 最小可执行版本定义

第一版 MVP bridge 只需满足：

```text
HOPE-PKU NatNet bridge / HOPE ROS mocap
-> HOPE table-frame robot + ball
-> 坐标/单位转换
-> current ZMQ schema
-> predict_node_0627.py --render 可用
```

不要求：

```text
战术打点选择
旋转估计
复杂球身份追踪
多球处理
完整异常恢复
自动标定
真机击球成功
```

---

## 19. 一句话总结

本次任务的正确抽象是：

```text
实现一个 OptiTrack/NatNet 到现有 ZMQ mocap schema 的适配层，
使当前 physics-based strike planner 能无感替换数据源。
```

最终目标不是让 planner 变复杂，而是让数据源替换后，下游 `predict_node_0627.py`、`/wbc_racket_command`、WBC / policy 仍然按原接口工作。
