# PingPongRobot

PingPongRobot 是一个由动捕驱动的乒乓球机器人项目。当前代码库主要围绕 HOPE / OptiTrack / NatNet 数据链路展开，把球和机器人位姿按 planner 现有的 ZMQ 协议送入下游。

## 仓库内容

- `mocap/hope_optitrack_adapter/`：Python 适配器，负责读取 NatNet 球数据和 ROS2 机器人位姿，转换到 planner 坐标系，并以 mm / mm/s 发布 ZMQ 消息。
- `planner/`：动捕桥接相关的规划与协作记录。
- `Optitrack/windows_natnet_adapter/`：Windows 侧 NatNet 适配器骨架和交接文档。
- `docs/`：设计说明和项目级文档。
- `logs/`：本地日志输出和会话产物。

## 快速开始

这个仓库当前主要围绕 `mocap/hope_optitrack_adapter/` 下的 Python 适配器。

### 1. 运行测试

在仓库根目录执行：

```bash
python3 -m unittest discover -s mocap/tests -v
```

### 2. 运行适配器

使用默认配置：

```bash
python3 -m mocap.hope_optitrack_adapter.main --config mocap/hope_optitrack_adapter/config.yaml
```

只运行球数据，不启用 ROS2 机器人接收器：

```bash
python3 -m mocap.hope_optitrack_adapter.main --config mocap/hope_optitrack_adapter/config.yaml --no-ros
```

## 适配器行为

当前适配器会：

- 从 NatNet 刚体读取球位姿数据；
- 从 ROS2 `motion_capture_tracking` 的命名位姿中读取机器人位姿；
- 将两者转换到 planner 坐标系；
- 将输出单位转换为 mm 和 mm/s；
- 通过 ZMQ `tcp://*:5556` 发布 planner 可直接消费的字典；
- 把 JSONL 诊断写入 `logs/`。

默认连接参数位于 `mocap/hope_optitrack_adapter/config.yaml`。

## 配置说明

适配器配置分为以下几部分：

- `natnet`：Motive 的服务端和客户端 IP、端口、连接类型以及 NatNet bitstream 版本。
- `ball`：球数据来源和刚体名称。
- `table`：球桌刚体名称。
- `robot`：机器人 topic、刚体名称，以及 mocap 到 `base_link` 的静态变换。
- `stream`：ZMQ 发布地址和输出单位。
- `coordinate`：源坐标系到 planner 坐标系的变换。
- `diagnostics`：JSONL 日志和 stdout 汇总设置。

## Windows / Motive 说明

如果你要验证真实的 OptiTrack 数据，请参考 Windows 侧交接文档：

- `Optitrack/windows_natnet_adapter/README.md`
- `Optitrack/windows_natnet_adapter/WINDOWS_SETUP.md`

这两份文档说明了 Motive / NatNet 的配置、SDK 预期和 Windows 机器上的 smoke test 流程。

## 开发说明

- 需要 Python 3.10+。
- 在 macOS 上，系统自带的 `python3` 可能仍然是 3.9，无法运行这些测试；请使用 Python 3.10+ 环境。
- 适配器支持可选的 YAML 路径；如果没有安装 `PyYAML`，代码会回退到一个只覆盖当前配置结构的轻量内置 YAML 解析器。
- 只要不同时更新下游消费端，就不要随意改 planner 输入的单位和 schema。

## 仓库结构

```text
docs/
logs/
mocap/
Optitrack/
planner/
```

完整的桥接设计见 `planner/optitrack_mocap_bridge_plan.md`。