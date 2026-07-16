# OptiTrack / NatNet 到当前 Planner 的接入方案

## 1. 当前结论

本项目只替换 `predict_node_0627.py` 的上游动捕数据源，不修改击球、预测球、WBC 指令发布和 planner 内部逻辑。

当前有效链路：

```text
OptiTrack cameras
    -> Motive on Windows 192.168.50.1
    -> NatNet Unicast command/data 1510/1511
    -> Mac NatNet receiver 192.168.50.2
    -> Python adapter
    -> ZMQ tcp://127.0.0.1:5556, send_pyobj(), mm/mm/s
    -> planner/predict_node_0627.py
```

明确不要再配置旧的 UDP bridge。Motive 中没有目标客户端 IP/port 输入框，Unicast 目标由 Mac 侧 NatNet client 的握手和订阅机制确定。

## 2. 任务边界

必须保持不变：

- `planner/predict_node_0627.py` 的预测与击球逻辑。
- planner 现有 ZMQ 输入 schema：Python `recv_pyobj()`。
- ball/robot 消息单位：位置 `mm`，速度 `mm/s`。
- `/wbc_racket_command` 输出。

允许修改或新增：

- `mocap/hope_optitrack_adapter/` 下的数据源 adapter。
- adapter 配置、诊断、测试和文档。

如果未来必须改 `predict_node_0627.py`，先复制一份诊断版再改，不直接覆盖原文件。

## 3. HOPE 文档中可复用的信息

参考文档：

```text
https://github.com/hitchopen/HOPE/blob/main/mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup_ZH.md
```

对当前任务有价值的结论：

- HOPE 使用 OptiTrack Motive 和 NatNet 作为动捕数据源，目标帧率可到 `360 Hz`。
- Motive 坐标系采用 `Z Axis` 作为 Up Axis。
- HOPE 对刚体对象通常使用 `PPT`、`P1`、`P2` 等 rigid body 名称。
- HOPE 对乒乓球的推荐做法是单个 unlabeled marker，而不是多 marker rigid body；这是为了避免球旋转时刚体 pivot 误差影响球心。
- HOPE 的 Motive streaming 检查项包括：NatNet enabled、Unicast 或 Multicast、Command Port `1510`、Data Port `1511`、Rigid Bodies on、Skeletons off、Up Axis `Z Axis`。

当前现场已经把 `ball` 做成 rigid body，Mac adapter 先按刚体名 `ball` 接入。这是为了最快验证 `OptiTrack -> Mac -> Planner` 链路。如果现场发现旋转球时球心明显画圈，应回到 HOPE 推荐的单 marker ball 方案，再扩展 adapter 读取 marker stream。

## 4. Windows / Motive 设置

现场 Windows 电脑：

```text
IP: 192.168.50.1
Subnet: 255.255.255.0
Motive: connected to OptiTrack cameras
```

Motive Data Streaming：

```text
NatNet: Enabled
Local Interface: 192.168.50.1
Transmission Type: Unicast
Command Port: 1510
Data Port: 1511
Up Axis: Z Axis
Rigid Bodies: On
Skeletons: Off
Marker Sets: Off
```

如果之后采用 HOPE 推荐的单 marker ball：

```text
Unlabeled Markers: On
Labeled Markers: Off
```

当前 rigid body ball 路径只要求 `Rigid Bodies: On`，adapter 暂不消费 marker stream。

## 5. Mac 网络设置

Mac 有线网口：

```text
IP: 192.168.50.2
Subnet: 255.255.255.0
Gateway: empty or 192.168.50.1
DNS: not required
```

到现场后先验证：

```bash
ifconfig
ping 192.168.50.1
```

如果 adapter 收不到数据，用 tcpdump 判断是否有来自 Motive 的 UDP：

```bash
sudo tcpdump -ni <mac_ethernet_interface> host 192.168.50.1 and udp
```

判断方式：

```text
能看到 UDP，但 adapter 无 frame：
    优先查 NatNet bitstream/version、packet parser、Unicast 绑定端口。

看不到 UDP：
    优先查 Motive streaming、Windows 防火墙、Windows Local Interface、网线/网卡。
```

## 6. 当前配置

当前默认配置文件：

```text
mocap/hope_optitrack_adapter/config.yaml
```

关键配置：

```yaml
natnet:
  server_ip: "192.168.50.1"
  local_ip: "192.168.50.2"
  command_port: 1510
  data_port: 1511
  connection_type: "unicast"
  bitstream_version: [4, 1]

ball:
  source: natnet_rigid_body
  rigid_body_name: "ball"

table:
  rigid_body_name: "table"

robot:
  source: motion_capture_tracking_named_pose
  topic: "/motion_capture_tracking/poses"
  rigid_body_name: "P1"

stream:
  publish_zmq: "tcp://*:5556"
```

`bitstream_version: [4, 1]` 是为了请求 Motive 输出 adapter 当前解析器覆盖的 NatNet 4.x frame layout。若现场 Motive 强制使用更高版本且解析失败，再按实际 packet 调整。

## 7. Adapter 实现结构

当前实现目录：

```text
mocap/hope_optitrack_adapter/
```

主要模块：

```text
natnet_receiver.py
    NatNet Unicast transport
    NAT_CONNECT / REQUEST_MODELDEF
    MODELDEF rigid body name -> id
    FRAMEOFDATA rigid body pose
    ball position and velocity estimate

ros_receiver.py
    robot NamedPoseArray receiver
    按 rigid_body_name 选 P1

coordinate_adapter.py
    source/table frame -> planner frame

validation.py
    ball valid、dt、速度上限、trajectory break 检查

schema_adapter.py
    m / m/s -> mm / mm/s
    生成当前 planner 兼容 dict

diagnostics.py
    JSONL 日志
    每秒 stdout 一行 fps/latency_ms/valid_rate/drop

main.py
    组装 receiver、validator、transform、ZMQ publisher
```

## 8. Planner 输入 schema

当前 planner 通过：

```python
socket.connect("tcp://127.0.0.1:5556")
data = socket.recv_pyobj()
```

adapter 发布 ball：

```python
{
    "type": "ball",
    "t": natnet_timestamp_seconds,
    "pos": [x_mm, y_mm, z_mm],
    "vel": [vx_mm_s, vy_mm_s, vz_mm_s],
    "valid": True,
}
```

adapter 发布 robot：

```python
{
    "type": "robot",
    "t": ros_timestamp_seconds,
    "pos": [x_mm, y_mm, z_mm],
    "quat": [qx, qy, qz, qw],
    "valid": True,
}
```

内部计算统一使用 SI 单位：`m`、`m/s`、`s`。只在 `schema_adapter.py` 发送前转换成 planner 需要的 `mm`、`mm/s`。

## 9. 坐标与标定

当前配置中的默认转换：

```yaml
coordinate:
  source_frame: "hope_table"
  R_planner_from_source:
    - [1, 0, 0]
    - [0, 1, 0]
    - [0, 0, 1]
  t_planner_from_source_m: [-1.37, 0.7625, 0.02]
```

这表示先按 HOPE/table frame 数据接入，再平移到当前 planner 默认球桌坐标。现场必须检查：

- 球桌中心附近是否接近 planner 期望原点。
- `z` 是否向上。
- 来球方向的速度符号是否符合 planner 触发条件。
- 静止球位置是否稳定。

P1 动捕刚体到机器人 `base_link` 的静态标定参数暂时保留为 identity：

```yaml
robot:
  mocap_to_base_link_translation_m: [0.0, 0.0, 0.0]
  mocap_to_base_link_quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]
```

真实机器人联调前再标定。

## 10. 现场运行步骤

第一步，只验证 Mac 能收到 Motive NatNet：

```bash
python3 -m mocap.hope_optitrack_adapter.main \
  --config mocap/hope_optitrack_adapter/config.yaml \
  --no-ros
```

期待每秒输出类似：

```text
mocap fps=360.0 latency_ms=... valid_rate=... drop=0
```

第二步，如果 Mac ROS2 和 `motion_capture_tracking` 可用，运行 full adapter：

```bash
python3 -m mocap.hope_optitrack_adapter.main \
  --config mocap/hope_optitrack_adapter/config.yaml
```

第三步，启动 planner：

```bash
python3 planner/predict_node_0627.py
```

启动顺序建议：

```text
1. Motive streaming on
2. adapter running and diagnostics stable
3. planner start
```

ZMQ PUB/SUB 有 slow joiner 现象，前几帧丢失是正常的；稳定后应持续收到 ball。

## 11. 无机器人本体时如何验证

没有 policy 和机器人本体时，仍可验证数据源链路：

- adapter 能启动并持续输出 fps/latency/valid/drop。
- 移动 `ball` 刚体时，ZMQ ball 消息位置连续变化。
- 静止球速度接近 0。
- 手动让球沿来球方向运动时，planner 能出现“发现新来球”和预测输出。

这只能证明 `OptiTrack -> Mac -> Planner prediction` 链路，不证明真实机器人坐标、WBC 指令或击球动作正确。

## 12. 现场失败排查

`ping 192.168.50.1` 不通：

```text
检查 Mac/Windows 静态 IP、网线、拓展坞、Windows 防火墙。
```

`tcpdump` 无 UDP：

```text
检查 Motive Streaming 是否 Enabled；
检查 Local Interface 是否为 192.168.50.1；
检查 Transmission Type 是否 Unicast；
不要把 Broadcast Port 或其他端口改成旧 UDP bridge 端口。
```

有 UDP 但无 frame：

```text
检查 NatNet bitstream version；
检查 Command/Data port 1510/1511；
检查 adapter local_ip 是否为 192.168.50.2；
检查 Motive 是否开启 Rigid Bodies。
```

能收到 frame 但没有 ball：

```text
检查 Motive rigid body 名称是否真为 "ball"；
检查 config.yaml 的 ball.rigid_body_name；
检查 ball tracking_valid。
```

ball 静止时位置抖动或旋转时画圈：

```text
优先在 Motive 中检查 rigid body pivot；
如果 rigid body ball 本身不适合，切到 HOPE 推荐的 single unlabeled marker ball 方案。
```

## 13. 成功标准

NatNet 层：

```text
fps 接近 360
frame id 持续增长
drop 接近 0
timestamp 单调递增
```

刚体层：

```text
table 和 ball 名称正确
ball tracking_valid 稳定
静止球位置稳定
```

坐标层：

```text
z 向上
单位正确
来球方向速度符号正确
球桌中心/边界与 planner 坐标预期一致
```

planner 层：

```text
ZMQ recv_pyobj 正常
planner 能收到 ball
planner 能触发预测
不修改击球和预测逻辑
```
