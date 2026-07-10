# OptiTrack NatNet 与 HOPE 动捕链路：迁移可行性与实施方案

> **目标**：明确 HOPE 仓库哪些内容可迁移到 OptiTrack，哪些部分必须替换；给出面向当前乒乓球机器人项目的最小可用接入方案。
>
> **结论先行**：HOPE 的**对象语义、坐标系约定、ROS 2 输出接口、球状态估计/规划上层**可以参考或复用；但 HOPE 公开仓库**不包含直接调用 OptiTrack NatNet SDK 的收包代码**。OptiTrack 的实时数据接收应由 `motion_capture_tracking` 或自建 NatNet adapter 完成。

---

## 1. 问题边界

本次需要回答的不是“HOPE 是否支持动捕”，而是：

1. OptiTrack 的 NatNet SDK 能否实时提供球位置和刚体位姿？
2. HOPE 仓库中是否已经实现了该 SDK 接收层？
3. 若没有，HOPE 的哪些模块仍可迁移到 OptiTrack？
4. 对当前 SMASH 风格的本体视觉复现，最小、可靠的接入路径是什么？

---

## 2. NatNet SDK：OptiTrack 实时数据能力

### 2.1 SDK 的职责

NatNet 是 OptiTrack/Motive 对外发送实时或回放动捕数据的网络 SDK：

```text
Motive（Windows / Tracking Server）
  └─ UDP（Unicast 或 Multicast）
       └─ NatNet Client / packet parser
            └─ 实时 frame data
                 ├─ rigid bodies
                 ├─ labeled markers
                 ├─ unlabeled markers
                 ├─ frame id
                 └─ timestamps / latency data
```

官方 SDK 中，`NatNetClient` 是原生 C++ 客户端接收数据包的核心对象；`NatNetTypes.h` 定义协议中的数据类型。SDK 也提供可改造的样例工程。

### 2.2 版本事实

用户提供的 URL 路径包含 `natnet-4.0`，但当前页面标题为 **NatNet 4.1**。官方列出：

| NatNet bitstream 版本 | 对应 Motive 版本 |
|---|---|
| NatNet 4.1 | Motive 3.1 |
| NatNet 4.0 | Motive 3.0 |

因此，实施前应从实际 Motive 服务端确认其 NatNet / bitstream 版本，不能假设本机 SDK、Motive 和第三方 parser 一定匹配。

### 2.3 能拿到的数据

| 目标对象 | Motive / NatNet 数据来源 | 可获得内容 |
|---|---|---|
| 球台 | `RigidBodyData` | 位置 + 四元数姿态（6DoF） |
| 机器人 `base_link` | `RigidBodyData` | 位置 + 四元数姿态（6DoF） |
| 头部/相机 rigid body | `RigidBodyData` | 位置 + 四元数姿态（6DoF） |
| 球 | unlabeled marker 或 labeled marker | 通常为 3D 点位置（3DoF） |
| 球（若做成 rigid body） | `RigidBodyData` | 位置 + 姿态；但小球 marker 几何和自旋下可靠性是另一问题 |

刚体姿态以四元数 `(qx, qy, qz, qw)` 提供。不要在接收层过早转换成 Euler 角；Euler 转换还依赖旋转顺序、左右手系和局部/全局轴约定。

### 2.4 官方 SDK 与 direct depacketization 的边界

NatNet 官方明确不建议长期依赖 direct depacketization：UDP bitstream 语法可能随版本变化，应用需要随 SDK 更新 parser。官方建议优先使用 NatNet library；只有官方库不可用的平台才应使用 `PacketClient` / `PythonSample` 直接解析数据包。

**工程结论**：

- **生产主链路优先**：官方 NatNet SDK 或使用其官方 SDK backend 的 bridge。
- **快速原型/跨平台验证**：可尝试 direct parser，但必须绑定 Motive/NatNet 版本并做回归测试。

---

## 3. HOPE 仓库：实际提供了什么

### 3.1 HOPE 的定位

HOPE 是乒乓球人形机器人开放平台仓库。其公开内容包含：

- 参考设计文档；
- A3 + Isaac Lab starter；
- ROS 2 / mocap / planner 的接口与背景资料；
- 规划、消息和 bring-up 相关结构。

其 README 把 `hope_ws/`、`mocap/`、`HOPE_*_Reference_Setup.md` 标记为 optional/background material，而非完整可直接部署的 OptiTrack SDK 客户端。

### 3.2 HOPE 的目标动捕架构

HOPE 文档明确给出的 OptiTrack 路线是：

```text
OptiTrack Cameras
  → Motive（Windows）
  → NatNet UDP（LAN）
  → Linux ROS 2 Host
  → motion_capture_tracking
  → /poses + /tf
  → Planner / WBC
```

关键点：**NatNet → ROS 2 的协议适配由外部 `motion_capture_tracking` 承担，而不是 HOPE 仓库内置的 `NatNetClient` 实现。**

### 3.3 HOPE 定义的对象语义

HOPE 将外部动捕对象划分为三类：

| 对象 | HOPE 参考设计 | 动捕输出 |
|---|---|---|
| `PPT` 球台 | vendor-tracked rigid body | 6DoF pose |
| `P1/P2` 机器人 `base_link` | vendor-tracked rigid body | 6DoF pose |
| 乒乓球 | 单个 unlabeled marker | 3D position |

HOPE 明确不让动捕追踪球拍；球拍位姿应由：

```text
world → base_link（mocap） → joints（encoders） → forward kinematics → paddle pose
```

得到。这一原则与“本体感知/本体运动学为主、外部动捕不绕过末端控制”的架构一致。

---

## 4. HOPE 仓库中是否有 NatNet 实际接收代码？

### 4.1 结论

**没有证据表明 HOPE 公开仓库内包含直接调用 NatNet SDK 的收包实现。**

也就是说，公开仓库中不是下面这种链路：

```text
HOPE 内置 NatNetClient
  → Motive UDP
  → sFrameOfMocapData
  → RigidBodyData / marker arrays
```

而是文档级架构：

```text
Motive / NatNet
  → 外部 motion_capture_tracking
  → ROS 2 /poses + /tf
  → HOPE planner / WBC
```

### 4.2 可迁移与不可直接迁移的模块

| 模块 | 是否可直接复用/参考 | 原因 |
|---|---:|---|
| HOPE 的球台、机器人 `base_link`、球的对象语义 | 可以 | 与 OptiTrack/NatNet 数据模型一致 |
| HOPE 的 ROS 2 坐标与 topic 约定 | 可以参考 | 需要按本项目统一 frame 后使用 |
| HOPE 的球状态估计、轨迹预测、击球规划上层 | 可以参考 | 输入可抽象为球位置/时间戳，与协议无关 |
| HOPE 文档中的 Motive 配置原则 | 可以参考 | 需按实际设备、版本和项目坐标系验证 |
| `avatar_pro_vrpn_relay` 类 VRPN 路径 | 不可用于 OptiTrack NatNet | VRPN 与 NatNet 是不同协议 |
| NatNet UDP 收包、frame parser、Data Description 映射 | 必须另行实现/部署 | HOPE 不提供该层源码 |
| `PoseArray` 中按索引区分对象的做法 | 不建议作为生产接口 | `PoseArray` 不携带对象名/ID/质量信息 |

---

## 5. `motion_capture_tracking`：HOPE 推荐的 OptiTrack bridge

### 5.1 能力

`IMRCLab/motion_capture_tracking` 是 ROS 2 package，可接收多类动捕系统，包括 OptiTrack。其支持：

1. 使用厂商软件追踪刚体 6DoF pose；
2. 用原始点云做自定义刚体逐帧追踪；
3. 用原始无标签点云做单 marker 逐帧追踪；
4. 将结果发布为 `tf2` 和 `/poses`。

这刚好覆盖 HOPE 的三类对象：

```text
PPT / RobotBase / EgoCamera
  → Motive vendor rigid bodies
  → 6DoF poses

Ball
  → unlabeled marker point cloud
  → frame-to-frame tracking
  → 3D point
```

### 5.2 OptiTrack 后端

该项目列出两条 OptiTrack 后端：

| backend | 原理 | 平台 | 风险/建议 |
|---|---|---|---|
| `optitrack` | direct depacketization | 跨平台 | 对未测试 Motive 版本可能有兼容问题，不覆盖所有功能 |
| `optitrack_closed_source` | 官方 NatNet SDK 4.1.0 | 仅 x64 Linux | 更接近官方 SDK；需要 Linux x64 环境 |

仓库建议先尝试 `optitrack`，出现问题时切换 `optitrack_closed_source`。但从 NatNet 官方文档的长期稳定性原则看，生产系统应优先评估官方 SDK backend。

### 5.3 文档给出的 Motive 设置

推荐配置为：

```text
Enable NatNet:          ON
Transmission Type:      Multicast（或按项目改为 Unicast）
Up Axis:                Z Axis
Rigid Bodies:           ON
Unlabeled Markers:      ON
Labeled Markers:        OFF（若不需要）
Marker Sets:            OFF（若不需要）
Skeletons:              OFF（若不需要）
Command Port:           1510
Data Port:              1511
```

这些是 HOPE/reference bridge 的目标设置；上线前仍须在实际 Motive 界面、网络策略和接收端日志中逐项验证。

---

## 6. 球的语义：最容易被写错的部分

### 6.1 不要把表面 marker 自动命名为球心

如果球上只有一个离散反光点，且该点会随自旋绕球心运动，则其位置不是严格的球心轨迹。此时接口必须保守命名为：

```python
ball_marker_position_world
```

而不是：

```python
ball_center_position_world
```

### 6.2 HOPE 的推荐球方案

HOPE 的单点方案是：

- 使用一个无标签 marker，或将球完全反光包覆；
- 通过逐帧关联追踪一个 3D 点；
- 不估计旋转；
- 不建模 Magnus force。

其文档认为，全反光包覆球可在每个相机中形成单一亮斑，重建为接近球几何中心的单点；多离散反光贴片会在点云中产生多个表面点，并随自旋/遮挡导致 identity switching。

### 6.3 本项目的命名建议

接入初期，建议显式保留测量语义：

```python
ball_position_mocap: Vec3 | None
ball_position_semantics: Literal[
    "surface_marker",
    "reflective_blob_center",
    "rigid_body_pivot",
]
ball_position_validated_as_center: bool
```

只有完成静态、旋转与高速飞行验证后，才将其作为 `ball_center_position` 输入最终滤波/规划链路。

---

## 7. 坐标系与机器人相机：不可省略的转换层

### 7.1 建议统一 frame

建议项目层面明确：

```text
M : Motive world frame
T : table frame
R : robot base frame
C : ego camera frame
```

所有下游模块最终读取的应是 table frame 下的数据：

```text
ball_position_table
robot_base_pose_table
ego_camera_pose_table
```

而不是直接混用 Motive world 数据。

### 7.2 HOPE 的坐标约定

HOPE 参考设计采用 ROS 2 REP-103 的 Z-up 坐标系，并以球台近端左侧桌角作为原点；X 沿桌长朝向对方，Y 沿桌宽，Z 向上。该约定可以参考，但不要未经确认直接复制到本项目。

**项目必须唯一确定**：

- 原点：球台中心还是近端角点；
- X/Y/Z 的方向；
- 右手系约定；
- 球台 rigid body pivot 的位置和方向；
- Motive `Up Axis`；
- `M → T` 的固定/动态变换。

### 7.3 头部相机不能只靠机器人 base pose 推断

若相机安装在会运动的头部，通常：

```text
T_world_camera
= T_world_base
× T_base_head(q_joint)
× T_head_camera
```

因此仅追踪 `base_link` 不足以获得准确相机世界位姿。过渡阶段可以在相机支架或头部设置一个 OptiTrack rigid body，直接获得 `T_world_camera`，以验证 ego 视觉坐标转换。

---

## 8. 推荐的最小架构（不把 OptiTrack 耦合进 planner）

```text
Motive / OptiTrack
  → NatNet
  → OptiTrack adapter
      ├─ Motive rigid body name/id resolution
      ├─ ball marker association
      ├─ source timestamp extraction
      └─ Motive → table frame transform
  → MocapFrame（项目内部统一数据契约）
      ├─ online latest-frame buffer
      │    └─ AEKF / trajectory predictor / strike planner
      └─ bounded recording queue
           └─ raw + normalized logs
```

推荐内部数据契约：

```python
from dataclasses import dataclass
from typing import Literal

Vec3 = tuple[float, float, float]
QuatXYWZ = tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class PointTrack:
    position_m: Vec3
    tracked: bool
    residual: float | None = None


@dataclass(frozen=True, slots=True)
class PoseTrack:
    position_m: Vec3
    quaternion_xyzw: QuatXYWZ
    tracked: bool


@dataclass(frozen=True, slots=True)
class MocapFrame:
    source: Literal["optitrack"]
    frame_id: int
    source_timestamp_s: float
    received_monotonic_s: float

    ball: PointTrack | None
    table: PoseTrack | None
    robot_base: PoseTrack | None
    ego_camera: PoseTrack | None
```

### 核心原则

1. **协议隔离**：NatNet 类型不得泄露到 AEKF / planner。
2. **身份显式**：禁止用 `poses[0]` 约定球。
3. **丢旧保新**：控制链路读取 latest frame，不使用无界队列积压旧数据。
4. **回调轻量**：NatNet 回调只做解析、字段复制、发布；不做 UI、磁盘写入、复杂预测。
5. **时间戳分离**：源时间戳用于 `dt`；本机收到时间只用于延迟诊断。
6. **坐标集中**：`M → T` 变换只在一个模块维护。

---

## 9. 实施路径

### P0：验证 OptiTrack/Motive 基础连通性

```text
Motive
→ 官方 NatNet sample / MinimalClient
→ 打印 server version、frame rate、rigid body IDs、marker data
```

验收：

- 能连续收到帧；
- 能看到球台、机器人、相机的 rigid body；
- 能确认球数据来源是 unlabeled marker / labeled marker / rigid body；
- 记录 Motive/NatNet 的实际版本。

### P1：先打通刚体 pose

```text
RigidBodyData
→ table pose
→ robot base pose
→ ego camera pose（若装了相机 rigid body）
```

验收：

- 启动时解析 `name → streaming ID` 映射，不硬编码 ID；
- 输出位置 + 四元数；
- 静止球台下，`table` 在 table frame 中近似恒定；
- 验证轴方向没有交换、镜像或单位错误。

### P2：再打通球位置

```text
unlabeled/labeled marker
→ ball association
→ ball_position_mocap
→ validity + semantics metadata
```

验收：

- 静态球位置稳定；
- 缓慢移动时轨迹连续；
- 自旋、遮挡、发球下无明显 marker identity switching；
- 对球中心语义给出实验结论，而非代码假设。

### P3：接入状态估计与规划

```text
ball_position_table + source_timestamp
→ AEKF / polynomial baseline
→ trajectory prediction
→ strike planner
```

验收：

- 使用 NatNet source timestamp 推进状态；
- 正确处理帧跳号和 tracking lost；
- 输出球位置、速度、预测击球点与击球时间；
- OptiTrack 轨迹可与 ego vision trajectory 对齐评估。

---

## 10. 推荐方案

### 10.1 当前最小可用方案

```text
Windows Motive
  → NatNet UDP
  → Linux ROS 2 host / Python bridge
  → 具名 MocapFrame
  → table frame
  → AEKF + strike planner
```

两条落地路径：

| 环境 | 优先方案 |
|---|---|
| Linux x64 + ROS 2 | 先评估 `motion_capture_tracking`；有兼容性问题时使用其官方 SDK backend 或自建官方 SDK bridge |
| Windows + Python 主程序 | 直接基于官方 NatNet SDK sample / PythonSample 做小型 adapter，输出项目内 `MocapFrame` |
| macOS 作为开发机 | 不建议把它作为官方 SDK 生产接收端；可用于上层开发或通过 Windows/Linux bridge 消费数据 |

### 10.2 不推荐的方案

```text
不要：直接修改 VRPN relay 以“兼容 NatNet”
不要：将 NatNet 接收逻辑嵌入 planner
不要：以 PoseArray 的数组下标识别球
不要：把单个球表面 marker 无验证地命名为球心
不要：在收包回调中进行 UI、磁盘写入、AEKF 和 planner
不要：无界排队保存实时帧
```

---

## 11. 需要现场确认的信息

以下信息决定 adapter 的具体实现，必须在设备到位后记录：

1. Motive 的确切版本、NatNet version、bitstream version；
2. 使用单播还是组播、服务端与客户端 IP、网卡和端口；
3. 当前球的反光方案：单点、全反光、多个贴片，还是 rigid body；
4. 球台、机器人 base、头部/相机是否已建立 vendor rigid body；
5. 各 rigid body 的名称、streaming ID、pivot 与轴方向；
6. 相机是否随头部/颈部运动，是否需要相机 rigid body；
7. 是否计划采用 ROS 2；
8. SMASH 当前 AEKF / strike planner 的球状态输入位置及时间戳接口。

---

## 12. 最终结论

> **HOPE 可以迁移到 OptiTrack，但迁移的是上层数据语义和 ROS/规划接口，不是复制一个现成的 HOPE NatNet 客户端。**

准确的系统边界是：

```text
OptiTrack Motive
  → NatNet SDK / NatNet bridge
  → motion_capture_tracking 或自建 OptiTrackAdapter
  → ball_position + table_pose + robot_base_pose + ego_camera_pose
  → table frame
  → AEKF / SMASH trajectory prediction / strike planner
```

对本项目而言，OptiTrack 的定位应是：

```text
外部动捕真值系统
+ 过渡阶段的球状态输入
+ 本体视觉定位与预测误差的评估基准
```

而不是长期替代机器人本体视觉。

---

## 参考资料

1. [OptiTrack NatNet 4.1 SDK Overview](https://docs.optitrack.com/v3.1/developer-tools/natnet-sdk/natnet-4.0)
2. [HOPE Repository](https://github.com/hitchopen/HOPE)
3. [HOPE Motion Capture System and Coordinates Reference Setup](https://github.com/hitchopen/HOPE/blob/main/mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md)
4. [IMRCLab motion_capture_tracking](https://github.com/IMRCLab/motion_capture_tracking)
