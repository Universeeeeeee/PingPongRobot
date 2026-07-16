from __future__ import annotations

import select
import socket
import struct
import time
from dataclasses import dataclass
from typing import Any


NAT_CONNECT = 0
NAT_SERVERINFO = 1
NAT_REQUEST_MODELDEF = 4
NAT_MODELDEF = 5
NAT_FRAMEOFDATA = 7
NAT_KEEPALIVE = 10


@dataclass(frozen=True)
class NatNetRigidBody:
    rigid_body_id: int
    name: str
    position_m: list[float]
    quaternion_xyzw: list[float]
    tracking_valid: bool
    mean_error: float | None = None


@dataclass(frozen=True)
class NatNetFrame:
    frame_id: int
    timestamp_s: float
    rigid_bodies: dict[str, NatNetRigidBody]


class NatNetPacketParser:
    def __init__(self, bitstream_version: tuple[int, int] = (4, 1)) -> None:
        self.bitstream_version = bitstream_version
        self.rigid_body_names_by_id: dict[int, str] = {}
        self.rigid_body_ids_by_name: dict[str, int] = {}

    def parse_packet(self, packet: bytes) -> NatNetFrame | None:
        if len(packet) < 4:
            raise ValueError("NatNet packet is shorter than header")
        message_id, packet_size = struct.unpack_from("<HH", packet, 0)
        payload = packet[4 : 4 + packet_size]
        if len(payload) != packet_size:
            raise ValueError("NatNet packet payload is truncated")

        if message_id == NAT_MODELDEF:
            self._parse_modeldef(payload)
            return None
        if message_id == NAT_FRAMEOFDATA:
            return self._parse_frame(payload)
        return None

    def _parse_modeldef(self, payload: bytes) -> None:
        try:
            self._parse_modeldef_without_dataset_sizes(payload)
        except ValueError as original_error:
            try:
                self._parse_modeldef_with_dataset_sizes(payload)
            except ValueError:
                raise original_error

    def _parse_modeldef_without_dataset_sizes(self, payload: bytes) -> None:
        reader = _PacketReader(payload)
        rigid_body_names_by_id: dict[int, str] = {}
        rigid_body_ids_by_name: dict[str, int] = {}
        dataset_count = reader.read_uint32()
        for _index in range(dataset_count):
            dataset_type = reader.read_uint32()
            if dataset_type == 0:
                self._skip_marker_set_description(reader)
            elif dataset_type == 1:
                name, rigid_body_id = self._read_rigid_body_description(reader)
                rigid_body_names_by_id[rigid_body_id] = name
                rigid_body_ids_by_name[name] = rigid_body_id
            elif dataset_type == 2:
                self._skip_skeleton_description(reader)
            elif dataset_type == 5:
                self._skip_camera_description(reader)
            else:
                raise ValueError(f"unsupported NatNet model definition dataset type {dataset_type}")
        self.rigid_body_names_by_id = rigid_body_names_by_id
        self.rigid_body_ids_by_name = rigid_body_ids_by_name

    def _parse_modeldef_with_dataset_sizes(self, payload: bytes) -> None:
        reader = _PacketReader(payload)
        rigid_body_names_by_id: dict[int, str] = {}
        rigid_body_ids_by_name: dict[str, int] = {}
        dataset_count = reader.read_uint32()
        for _index in range(dataset_count):
            dataset_type = reader.read_uint32()
            block_size = reader.read_uint32()
            block = reader.read_bytes(block_size)
            if dataset_type == 1:
                block_reader = _PacketReader(block)
                name = block_reader.read_c_string()
                rigid_body_id = block_reader.read_uint32()
                rigid_body_names_by_id[rigid_body_id] = name
                rigid_body_ids_by_name[name] = rigid_body_id
        self.rigid_body_names_by_id = rigid_body_names_by_id
        self.rigid_body_ids_by_name = rigid_body_ids_by_name

    def _parse_frame(self, payload: bytes) -> NatNetFrame:
        major, minor = self.bitstream_version
        reader = _PacketReader(payload)
        frame_id = reader.read_uint32()
        self._skip_marker_set_data(reader, major, minor)
        self._skip_legacy_marker_data(reader, major, minor)

        rigid_body_count = reader.read_uint32()
        self._read_data_size_if_present(reader, major, minor)
        rigid_bodies: dict[str, NatNetRigidBody] = {}
        for _index in range(rigid_body_count):
            rigid_body = self._read_rigid_body(reader, major, minor)
            rigid_bodies[rigid_body.name] = rigid_body

        self._skip_skeleton_data(reader, major, minor)
        if self._has_data_size_fields(major, minor):
            self._skip_counted_block(reader)
        self._skip_labeled_markers(reader, major, minor)
        self._skip_counted_block(reader)
        self._skip_counted_block(reader)
        timestamp_s = self._read_frame_suffix(reader, major, minor)
        return NatNetFrame(frame_id=frame_id, timestamp_s=timestamp_s, rigid_bodies=rigid_bodies)

    def _read_rigid_body(self, reader: "_PacketReader", major: int, minor: int) -> NatNetRigidBody:
        rigid_body_id = reader.read_uint32()
        position = reader.read_float32_list(3)
        quaternion = reader.read_float32_list(4)
        if major < 3 and major != 0:
            marker_count = reader.read_uint32()
            reader.skip(12 * marker_count)
            if major >= 2:
                reader.skip(8 * marker_count)
        mean_error: float | None = None
        if major >= 2:
            mean_error = reader.read_float32()
        tracking_valid = True
        if (major == 2 and minor >= 6) or major > 2:
            tracking_valid = (reader.read_int16() & 0x01) != 0
        name = self.rigid_body_names_by_id.get(rigid_body_id, str(rigid_body_id))
        return NatNetRigidBody(
            rigid_body_id=rigid_body_id,
            name=name,
            position_m=position,
            quaternion_xyzw=quaternion,
            tracking_valid=tracking_valid,
            mean_error=mean_error,
        )

    def _read_rigid_body_description(self, reader: "_PacketReader") -> tuple[str, int]:
        major, _minor = self.bitstream_version
        name = reader.read_c_string()
        rigid_body_id = reader.read_uint32()
        reader.skip(4)  # parent ID
        reader.skip(12)  # pivot/offset
        if major >= 3:
            marker_count = reader.read_uint32()
            reader.skip(12 * marker_count)
            reader.skip(4 * marker_count)
            if major >= 4:
                for _index in range(marker_count):
                    reader.read_c_string()
        return name, rigid_body_id

    def _skip_marker_set_description(self, reader: "_PacketReader") -> None:
        reader.read_c_string()
        marker_count = reader.read_uint32()
        for _index in range(marker_count):
            reader.read_c_string()

    def _skip_skeleton_description(self, reader: "_PacketReader") -> None:
        reader.read_c_string()
        reader.skip(4)
        rigid_body_count = reader.read_uint32()
        for _index in range(rigid_body_count):
            self._read_rigid_body_description(reader)

    def _skip_camera_description(self, reader: "_PacketReader") -> None:
        reader.read_c_string()
        reader.skip(12 + 16)

    def _skip_marker_set_data(self, reader: "_PacketReader", major: int, minor: int) -> None:
        marker_set_count = reader.read_uint32()
        self._read_data_size_if_present(reader, major, minor)
        for _index in range(marker_set_count):
            reader.read_c_string()
            marker_count = reader.read_uint32()
            reader.skip(12 * marker_count)

    def _skip_legacy_marker_data(self, reader: "_PacketReader", major: int, minor: int) -> None:
        marker_count = reader.read_uint32()
        self._read_data_size_if_present(reader, major, minor)
        reader.skip(12 * marker_count)

    def _skip_skeleton_data(self, reader: "_PacketReader", major: int, minor: int) -> None:
        if (major == 2 and minor > 0) or major > 2:
            skeleton_count = reader.read_uint32()
            self._read_data_size_if_present(reader, major, minor)
            for _index in range(skeleton_count):
                reader.skip(4)
                rigid_body_count = reader.read_uint32()
                for rb_index in range(rigid_body_count):
                    self._read_rigid_body(reader, major, minor)

    def _skip_labeled_markers(self, reader: "_PacketReader", major: int, minor: int) -> None:
        if (major == 2 and minor > 3) or major > 2:
            marker_count = reader.read_uint32()
            self._read_data_size_if_present(reader, major, minor)
            for _index in range(marker_count):
                reader.skip(4 + 12 + 4)
                if (major == 2 and minor >= 6) or major > 2:
                    reader.skip(2)
                if major >= 3:
                    reader.skip(4)

    def _skip_counted_block(self, reader: "_PacketReader") -> None:
        item_count = reader.read_uint32()
        byte_count = reader.read_uint32()
        if item_count > 0 and byte_count <= 0:
            raise ValueError("NatNet counted block has items but no byte count; parser cannot skip safely")
        reader.skip(byte_count)

    def _read_frame_suffix(self, reader: "_PacketReader", major: int, minor: int) -> float:
        reader.skip(8)  # timecode and subframe
        if (major == 2 and minor >= 7) or major > 2:
            timestamp_s = reader.read_float64()
        else:
            timestamp_s = reader.read_float32()
        if major >= 3:
            reader.skip(24)
        if major >= 4:
            reader.skip(8)
        reader.skip(2)
        return timestamp_s

    def _read_data_size_if_present(self, reader: "_PacketReader", major: int, minor: int) -> int:
        if self._has_data_size_fields(major, minor):
            return reader.read_uint32()
        return 0

    def _has_data_size_fields(self, major: int, minor: int) -> bool:
        return (major == 4 and minor > 0) or major > 4


class NatNetRigidBodyBallReceiver:
    def __init__(
        self,
        server_ip: str = "192.168.50.1",
        local_ip: str = "192.168.50.2",
        command_port: int = 1510,
        data_port: int = 1511,
        connection_type: str = "unicast",
        ball_name: str = "ball",
        table_name: str = "table",
        bitstream_version: tuple[int, int] = (4, 1),
        transport: "NatNetUdpTransport | None" = None,
        poll_timeout_s: float = 0.001,
    ) -> None:
        if connection_type != "unicast":
            raise ValueError("direct Mac NatNet receiver currently supports unicast only")
        self.ball_name = ball_name
        self.table_name = table_name
        self._poll_timeout_s = poll_timeout_s
        self._transport = transport or NatNetUdpTransport(
            server_ip=server_ip,
            local_ip=local_ip,
            command_port=command_port,
            data_port=data_port,
            bitstream_version=bitstream_version,
        )
        self._parser = NatNetPacketParser(bitstream_version=bitstream_version)
        self._started = False
        self._previous_frame_id: int | None = None
        self._previous_timestamp_s: float | None = None
        self._previous_position_m: list[float] | None = None
        self.last_received_monotonic_s: float | None = None
        self.last_error: str | None = None
        self._packet_count = 0
        self._frame_packet_count = 0
        self._modeldef_packet_count = 0
        self._frames_without_ball_count = 0
        self._last_message_id: int | None = None

    def recv_frame(self) -> dict[str, Any] | None:
        if not self._started:
            self._transport.start()
            self._started = True

        deadline_s = time.monotonic() + self._poll_timeout_s
        while True:
            timeout_s = max(0.0, deadline_s - time.monotonic())
            packet = self._transport.recv_packet(timeout_s)
            if packet is None:
                return None
            self.last_received_monotonic_s = time.monotonic()
            self._record_packet_header(packet)
            try:
                frame = self._parser.parse_packet(packet)
            except ValueError as exc:
                self.last_error = str(exc)
                continue
            if frame is None:
                continue
            ball_frame = self._frame_to_ball_dict(frame)
            if ball_frame is not None:
                return ball_frame
            self._frames_without_ball_count += 1
            if time.monotonic() >= deadline_s:
                return None

    def close(self) -> None:
        self._transport.close()

    def diagnostic_status(self) -> str:
        last_msg = self._last_message_id if self._last_message_id is not None else "n/a"
        rigid_body_names = ",".join(sorted(self._parser.rigid_body_ids_by_name.keys()))
        rigid_body_names = rigid_body_names or ""
        last_error = self.last_error or "n/a"
        return (
            "source=natnet "
            f"packets={self._packet_count} "
            f"last_msg={last_msg} "
            f"modeldef={self._modeldef_packet_count} "
            f"frames={self._frame_packet_count} "
            f"rb=[{rigid_body_names}] "
            f"no_ball={self._frames_without_ball_count} "
            f"err={last_error}"
        )

    def _record_packet_header(self, packet: bytes) -> None:
        self._packet_count += 1
        if len(packet) < 2:
            self.last_error = "NatNet packet is shorter than message id"
            return
        self._last_message_id = struct.unpack_from("<H", packet, 0)[0]
        if self._last_message_id == NAT_MODELDEF:
            self._modeldef_packet_count += 1
        elif self._last_message_id == NAT_FRAMEOFDATA:
            self._frame_packet_count += 1

    def _frame_to_ball_dict(self, frame: NatNetFrame) -> dict[str, Any] | None:
        ball = frame.rigid_bodies.get(self.ball_name)
        if ball is None:
            return None
        if not ball.tracking_valid:
            self._reset_velocity()
            return {
                "frame": frame.frame_id,
                "timestamp": frame.timestamp_s,
                "valid": False,
                "trajectory_break": True,
                "selected": ball.rigid_body_id,
                "source": "natnet_rigid_body",
                "position": ball.position_m,
                "velocity": [0.0, 0.0, 0.0],
            }

        velocity = [0.0, 0.0, 0.0]
        dt_s = None
        if self._previous_timestamp_s is not None and self._previous_position_m is not None:
            dt_s = frame.timestamp_s - self._previous_timestamp_s
            if dt_s > 0:
                velocity = [
                    (position_value - previous_value) / dt_s
                    for position_value, previous_value in zip(ball.position_m, self._previous_position_m)
                ]

        self._previous_frame_id = frame.frame_id
        self._previous_timestamp_s = frame.timestamp_s
        self._previous_position_m = ball.position_m
        payload: dict[str, Any] = {
            "frame": frame.frame_id,
            "timestamp": frame.timestamp_s,
            "valid": True,
            "trajectory_break": False,
            "selected": ball.rigid_body_id,
            "source": "natnet_rigid_body",
            "position": ball.position_m,
            "velocity": velocity,
        }
        if dt_s is not None:
            payload["dt"] = dt_s
        return payload

    def _reset_velocity(self) -> None:
        self._previous_frame_id = None
        self._previous_timestamp_s = None
        self._previous_position_m = None


class NatNetUdpTransport:
    def __init__(
        self,
        server_ip: str,
        local_ip: str,
        command_port: int = 1510,
        data_port: int = 1511,
        bitstream_version: tuple[int, int] = (4, 1),
    ) -> None:
        self.server_ip = server_ip
        self.local_ip = local_ip
        self.command_port = int(command_port)
        self.data_port = int(data_port)
        self.bitstream_version = bitstream_version
        self._command_socket: socket.socket | None = None
        self._data_socket: socket.socket | None = None
        self._started = False
        self._last_keepalive_s = 0.0

    def start(self) -> None:
        if self._started:
            return
        self._command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._command_socket.bind((self.local_ip, 0))

        self._data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._data_socket.bind((self.local_ip, self.data_port))

        self._send(NAT_CONNECT, _nat_connect_payload(self.bitstream_version))
        self._send(NAT_REQUEST_MODELDEF, b"")
        self._started = True

    def recv_packet(self, timeout_s: float | None = None) -> bytes | None:
        if self._command_socket is None or self._data_socket is None:
            raise RuntimeError("NatNet transport has not been started")
        self._send_keepalive_if_needed()
        ready, _writeable, _errors = select.select(
            [self._command_socket, self._data_socket],
            [],
            [],
            timeout_s,
        )
        if not ready:
            return None
        packet, _address = ready[0].recvfrom(65535)
        return packet

    def close(self) -> None:
        for sock in (self._command_socket, self._data_socket):
            if sock is not None:
                sock.close()
        self._command_socket = None
        self._data_socket = None
        self._started = False

    def _send_keepalive_if_needed(self) -> None:
        now_s = time.monotonic()
        if now_s - self._last_keepalive_s >= 1.0:
            self._send(NAT_KEEPALIVE, b"")
            self._last_keepalive_s = now_s

    def _send(self, message_id: int, payload: bytes) -> None:
        if self._command_socket is None:
            raise RuntimeError("NatNet command socket is not open")
        packet = struct.pack("<HH", message_id, len(payload)) + payload
        self._command_socket.sendto(packet, (self.server_ip, self.command_port))


class _PacketReader:
    def __init__(self, payload: bytes) -> None:
        self._payload = payload
        self._offset = 0

    def read_uint32(self) -> int:
        return int(self._unpack("<I", 4))

    def read_int16(self) -> int:
        return int(self._unpack("<h", 2))

    def read_float32(self) -> float:
        return float(self._unpack("<f", 4))

    def read_float64(self) -> float:
        return float(self._unpack("<d", 8))

    def read_float32_list(self, count: int) -> list[float]:
        values = struct.unpack_from("<" + "f" * count, self._payload, self._offset)
        self._offset += 4 * count
        return [float(value) for value in values]

    def read_c_string(self) -> str:
        end = self._payload.find(b"\0", self._offset)
        if end < 0:
            raise ValueError("NatNet string is not null terminated")
        raw = self._payload[self._offset : end]
        self._offset = end + 1
        return raw.decode("utf-8", errors="replace")

    def read_bytes(self, byte_count: int) -> bytes:
        if byte_count < 0:
            raise ValueError("NatNet byte count cannot be negative")
        if self._offset + byte_count > len(self._payload):
            raise ValueError("NatNet packet is truncated")
        raw = self._payload[self._offset : self._offset + byte_count]
        self._offset += byte_count
        return raw

    def skip(self, byte_count: int) -> None:
        if self._offset + byte_count > len(self._payload):
            raise ValueError("NatNet packet is truncated")
        self._offset += byte_count

    def _unpack(self, fmt: str, byte_count: int) -> int | float:
        if self._offset + byte_count > len(self._payload):
            raise ValueError("NatNet packet is truncated")
        value = struct.unpack_from(fmt, self._payload, self._offset)[0]
        self._offset += byte_count
        return value


def _nat_connect_payload(bitstream_version: tuple[int, int]) -> bytes:
    payload = bytearray(270)
    payload[0:4] = b"Ping"
    payload[264] = 0
    payload[265] = int(bitstream_version[0])
    payload[266] = int(bitstream_version[1])
    payload[267] = 0
    payload[268] = 0
    return bytes(payload)
