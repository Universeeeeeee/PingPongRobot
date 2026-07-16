from __future__ import annotations

import json
import ast
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .coordinate_adapter import CoordinateAdapter, RobotStaticTransform


@dataclass(frozen=True)
class AdapterConfig:
    ball_source: str = "natnet_rigid_body"
    udp_host: str | None = None
    udp_port: int | None = None
    natnet_server_ip: str = "192.168.50.1"
    natnet_local_ip: str = "192.168.50.2"
    natnet_command_port: int = 1510
    natnet_data_port: int = 1511
    natnet_connection_type: str = "unicast"
    natnet_bitstream_version: tuple[int, int] = (4, 1)
    ball_rigid_body_name: str = "ball"
    table_rigid_body_name: str = "table"
    robot_topic: str = "/motion_capture_tracking/poses"
    robot_rigid_body_name: str = "P1"
    publish_zmq: str = "tcp://*:5556"
    coordinate_adapter: CoordinateAdapter = CoordinateAdapter.default_hope_to_planner()
    robot_static_transform: RobotStaticTransform = RobotStaticTransform.identity()
    diagnostics_log_dir: str = "logs"
    diagnostics_enabled: bool = True
    diagnostics_print_interval_s: float = 1.0


def load_config(path: str | Path | None) -> AdapterConfig:
    if path is None:
        return AdapterConfig()
    config_path = Path(path)
    raw = _load_mapping(config_path)
    coordinate = raw.get("coordinate", {})
    stream = raw.get("stream", {})
    ball = raw.get("ball", {})
    natnet = raw.get("natnet", {})
    table = raw.get("table", {})
    robot = raw.get("robot", {})
    diagnostics = raw.get("diagnostics", {})
    coord_adapter = CoordinateAdapter(
        r_planner_from_source=coordinate.get(
            "R_planner_from_source",
            CoordinateAdapter.default_hope_to_planner().r_planner_from_source,
        ),
        t_planner_from_source_m=coordinate.get(
            "t_planner_from_source_m",
            CoordinateAdapter.default_hope_to_planner().t_planner_from_source_m,
        ),
    )
    robot_static_transform = RobotStaticTransform(
        translation_m=robot.get("mocap_to_base_link_translation_m", [0.0, 0.0, 0.0]),
        quaternion_xyzw=robot.get("mocap_to_base_link_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0]),
    )
    bitstream_version = natnet.get("bitstream_version", [4, 1])
    return AdapterConfig(
        ball_source=ball.get("source", "natnet_rigid_body"),
        udp_host=ball.get("host"),
        udp_port=int(ball["port"]) if "port" in ball else None,
        natnet_server_ip=natnet.get("server_ip", "192.168.50.1"),
        natnet_local_ip=natnet.get("local_ip", "192.168.50.2"),
        natnet_command_port=int(natnet.get("command_port", 1510)),
        natnet_data_port=int(natnet.get("data_port", 1511)),
        natnet_connection_type=natnet.get("connection_type", "unicast"),
        natnet_bitstream_version=(int(bitstream_version[0]), int(bitstream_version[1])),
        ball_rigid_body_name=ball.get("rigid_body_name", "ball"),
        table_rigid_body_name=table.get("rigid_body_name", "table"),
        robot_topic=robot.get("topic", "/motion_capture_tracking/poses"),
        robot_rigid_body_name=robot.get("rigid_body_name", "P1"),
        publish_zmq=stream.get("publish_zmq", "tcp://*:5556"),
        coordinate_adapter=coord_adapter,
        robot_static_transform=robot_static_transform,
        diagnostics_log_dir=diagnostics.get("log_dir", "logs"),
        diagnostics_enabled=bool(diagnostics.get("log_jsonl", True)),
        diagnostics_print_interval_s=float(diagnostics.get("print_interval_s", 1.0)),
    )


def _load_mapping(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        return json.loads(text)
    try:
        import yaml
    except ImportError:
        return _load_basic_yaml(text)
    loaded = yaml.safe_load(text)
    return loaded or {}


def _load_basic_yaml(text: str) -> dict[str, Any]:
    root: dict[str, Any] = {}
    current_section: str | None = None
    current_list_key: str | None = None

    for raw_line in text.splitlines():
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        stripped = raw_line.strip()
        if indent == 0 and stripped.endswith(":"):
            current_section = stripped[:-1]
            root[current_section] = {}
            current_list_key = None
            continue
        if current_section is None:
            raise ValueError("YAML value found before a top-level section")

        section = root[current_section]
        if indent == 2 and stripped.endswith(":"):
            current_list_key = stripped[:-1]
            section[current_list_key] = []
            continue
        if indent == 2 and ":" in stripped:
            key, value = stripped.split(":", 1)
            section[key.strip()] = _parse_scalar(value.strip())
            current_list_key = None
            continue
        if indent == 4 and stripped.startswith("- ") and current_list_key is not None:
            section[current_list_key].append(_parse_scalar(stripped[2:].strip()))
            continue
        raise ValueError(f"unsupported YAML line: {raw_line}")

    return root


def _parse_scalar(value: str) -> Any:
    if value == "":
        return None
    lower = value.lower()
    if lower == "true":
        return True
    if lower == "false":
        return False
    if lower == "null":
        return None
    if value.startswith("[") or value.startswith(("'", '"')):
        return ast.literal_eval(value)
    try:
        if any(char in value for char in ".eE"):
            return float(value)
        return int(value)
    except ValueError:
        return value
