import io
import struct
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from mocap.hope_optitrack_adapter.config import load_config
from mocap.hope_optitrack_adapter.coordinate_adapter import CoordinateAdapter, RobotStaticTransform
from mocap.hope_optitrack_adapter.diagnostics import DiagnosticsLogger
from mocap.hope_optitrack_adapter.hope_udp_json_receiver import HopeUdpBallReceiver
from mocap.hope_optitrack_adapter.main import HopeOptitrackAdapter
from mocap.hope_optitrack_adapter.natnet_receiver import NatNetPacketParser, NatNetRigidBodyBallReceiver
from mocap.hope_optitrack_adapter.ros_receiver import extract_named_pose, extract_tf_pose
from mocap.hope_optitrack_adapter.schema_adapter import ball_to_zmq, robot_to_zmq
from mocap.hope_optitrack_adapter.types import BallState, RobotPose
from mocap.hope_optitrack_adapter.validation import BallFrameValidator


class HopeOptitrackAdapterTest(unittest.TestCase):
    def assertSequenceAlmostEqual(self, actual, expected, places=7):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=places)

    def test_default_coordinate_adapter_maps_hope_table_frame_to_planner_frame(self):
        adapter = CoordinateAdapter.default_hope_to_planner()

        self.assertSequenceAlmostEqual(
            adapter.transform_position([1.37, -0.7625, 0.10]),
            [0.0, 0.0, 0.12],
        )
        self.assertEqual(adapter.transform_vector([-3.0, 0.1, 0.2]), [-3.0, 0.1, 0.2])
        self.assertEqual(adapter.transform_quaternion_xyzw([0.0, 0.0, 0.0, 1.0]), [0.0, 0.0, 0.0, 1.0])

    def test_schema_adapter_outputs_current_predict_node_zmq_schema_in_mm(self):
        ball = BallState(
            timestamp_s=12.34,
            position_m=[0.0, 0.0, 0.12],
            velocity_m_s=[-3.0, 0.1, 0.2],
            frame_id=42,
        )
        robot = RobotPose(
            timestamp_s=12.30,
            position_m=[-1.8, 0.0, 0.5],
            quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
            name="P1",
        )

        self.assertEqual(
            ball_to_zmq(ball),
            {
                "type": "ball",
                "t": 12.34,
                "pos": [0.0, 0.0, 120.0],
                "vel": [-3000.0, 100.0, 200.0],
                "valid": True,
            },
        )
        self.assertEqual(
            robot_to_zmq(robot),
            {
                "type": "robot",
                "t": 12.30,
                "pos": [-1800.0, 0.0, 500.0],
                "quat": [0.0, 0.0, 0.0, 1.0],
                "valid": True,
            },
        )

    def test_robot_static_transform_maps_mocap_rigid_body_to_base_link(self):
        transform = RobotStaticTransform(
            translation_m=[0.0, 0.0, -0.03],
            quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
        )
        p1_mocap = RobotPose(
            timestamp_s=4.0,
            position_m=[1.0, -0.2, 0.4],
            quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
            name="P1",
        )

        base_link = transform.apply(p1_mocap)

        self.assertEqual(base_link.name, "P1")
        self.assertSequenceAlmostEqual(base_link.position_m, [1.0, -0.2, 0.37])
        self.assertSequenceAlmostEqual(base_link.quaternion_xyzw, [0.0, 0.0, 0.0, 1.0])

    def test_robot_static_transform_translation_is_applied_in_mocap_body_frame(self):
        transform = RobotStaticTransform(
            translation_m=[1.0, 0.0, 0.0],
            quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
        )
        yaw_90_xyzw = [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]
        p1_mocap = RobotPose(
            timestamp_s=4.0,
            position_m=[0.0, 0.0, 0.0],
            quaternion_xyzw=yaw_90_xyzw,
            name="P1",
        )

        base_link = transform.apply(p1_mocap)

        self.assertSequenceAlmostEqual(base_link.position_m, [0.0, 1.0, 0.0])
        self.assertSequenceAlmostEqual(base_link.quaternion_xyzw, yaw_90_xyzw)

    def test_ball_validator_accepts_valid_udp_json_and_rejects_source_changes(self):
        validator = BallFrameValidator(max_dt_s=0.05)
        first = {
            "frame": 100,
            "timestamp": 1.0,
            "dt": 0.004,
            "valid": True,
            "trajectory_break": False,
            "selected": 0,
            "source": "other",
            "position": [1.0, -0.5, 0.3],
            "velocity": [-2.0, 0.1, 0.2],
        }
        second = dict(first, frame=101, timestamp=1.004, position=[0.99, -0.5, 0.3])
        source_changed = dict(first, frame=102, timestamp=1.008, source="labeled")

        accepted_first = validator.accept(first)
        accepted_second = validator.accept(second)
        rejected = validator.accept(source_changed)

        self.assertIsNotNone(accepted_first)
        self.assertIsNotNone(accepted_second)
        self.assertEqual(accepted_first.frame_id, 100)
        self.assertEqual(accepted_first.source, "other")
        self.assertIsNone(rejected)
        self.assertEqual(validator.last_drop_reason, "source_changed")

    def test_ball_validator_reacquires_after_candidate_discontinuity(self):
        validator = BallFrameValidator(max_dt_s=0.05)
        first = {
            "frame": 100,
            "timestamp": 1.0,
            "dt": 0.004,
            "valid": True,
            "trajectory_break": False,
            "selected": 0,
            "source": "other",
            "position": [1.0, -0.5, 0.3],
            "velocity": [-2.0, 0.1, 0.2],
        }
        source_changed = dict(first, frame=101, timestamp=1.004, source="labeled")
        reacquired = dict(source_changed, frame=102, timestamp=1.008, position=[0.99, -0.5, 0.3])

        self.assertIsNotNone(validator.accept(first))
        self.assertIsNone(validator.accept(source_changed))
        self.assertEqual(validator.last_drop_reason, "source_changed")
        self.assertIsNotNone(validator.accept(reacquired))

    def test_ball_validator_rejects_invalid_break_and_bad_dt(self):
        validator = BallFrameValidator(min_dt_s=0.001, max_dt_s=0.05)
        valid_frame = {
            "frame": 10,
            "timestamp": 1.0,
            "dt": 0.004,
            "valid": True,
            "trajectory_break": False,
            "selected": 0,
            "source": "other",
            "position": [1.0, -0.5, 0.3],
            "velocity": [-2.0, 0.1, 0.2],
        }

        self.assertIsNone(validator.accept(dict(valid_frame, valid=False)))
        self.assertEqual(validator.last_drop_reason, "invalid")
        self.assertIsNone(validator.accept(dict(valid_frame, trajectory_break=True)))
        self.assertEqual(validator.last_drop_reason, "trajectory_break")
        self.assertIsNone(validator.accept(dict(valid_frame, dt=0.0001)))
        self.assertEqual(validator.last_drop_reason, "dt_too_small")

    def test_extract_named_pose_selects_robot_by_name_from_named_pose_array(self):
        msg = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=3, nanosec=250_000_000)),
            poses=[
                SimpleNamespace(name="P2", pose=_pose([2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])),
                SimpleNamespace(name="P1", pose=_pose([1.0, -0.2, 0.4], [0.1, 0.2, 0.3, 0.9])),
            ],
        )

        robot = extract_named_pose(msg, "P1")

        self.assertEqual(robot.name, "P1")
        self.assertEqual(robot.timestamp_s, 3.25)
        self.assertEqual(robot.position_m, [1.0, -0.2, 0.4])
        self.assertEqual(robot.quaternion_xyzw, [0.1, 0.2, 0.3, 0.9])

    def test_extract_tf_pose_selects_robot_by_child_frame_id(self):
        msg = SimpleNamespace(
            transforms=[
                _transform("P2_mocap", 4.0, [2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
                _transform("P1_mocap", 4.5, [1.0, -0.2, 0.4], [0.1, 0.2, 0.3, 0.9]),
            ],
        )

        robot = extract_tf_pose(msg, "P1_mocap", name="P1")

        self.assertEqual(robot.name, "P1")
        self.assertEqual(robot.timestamp_s, 4.5)
        self.assertEqual(robot.position_m, [1.0, -0.2, 0.4])
        self.assertEqual(robot.quaternion_xyzw, [0.1, 0.2, 0.3, 0.9])

    def test_default_yaml_config_loads_without_external_yaml_dependency(self):
        config = load_config("mocap/hope_optitrack_adapter/config.yaml")

        self.assertEqual(config.ball_source, "natnet_rigid_body")
        self.assertEqual(config.natnet_server_ip, "192.168.50.1")
        self.assertEqual(config.natnet_local_ip, "192.168.50.2")
        self.assertEqual(config.natnet_command_port, 1510)
        self.assertEqual(config.natnet_data_port, 1511)
        self.assertEqual(config.natnet_connection_type, "unicast")
        self.assertEqual(config.ball_rigid_body_name, "ball")
        self.assertEqual(config.table_rigid_body_name, "table")
        self.assertEqual(config.robot_topic, "/motion_capture_tracking/poses")
        self.assertEqual(config.robot_rigid_body_name, "P1")
        self.assertEqual(config.publish_zmq, "tcp://*:5556")
        self.assertSequenceAlmostEqual(
            config.coordinate_adapter.t_planner_from_source_m,
            [-1.37, 0.7625, 0.02],
        )
        self.assertSequenceAlmostEqual(config.robot_static_transform.translation_m, [0.0, 0.0, 0.0])
        self.assertSequenceAlmostEqual(config.robot_static_transform.quaternion_xyzw, [0.0, 0.0, 0.0, 1.0])
        self.assertEqual(config.diagnostics_print_interval_s, 1.0)

    def test_default_yaml_config_does_not_advertise_unused_tf_fallback(self):
        config_text = Path("mocap/hope_optitrack_adapter/config.yaml").read_text(encoding="utf-8")

        self.assertNotIn("tf_child_frame_id", config_text)
        self.assertNotIn("38999", config_text)
        self.assertNotIn("hope_natnet_udp_json", config_text)


class DiagnosticsLoggerTest(unittest.TestCase):
    def test_prints_per_second_summary_even_when_jsonl_is_disabled(self):
        clock = _FakeClock(10.0)
        output = io.StringIO()
        logger = DiagnosticsLogger(
            enabled=False,
            print_interval_s=1.0,
            monotonic_clock=clock,
            output=output,
        )
        ball = BallState(
            timestamp_s=12.34,
            position_m=[0.0, 0.0, 0.12],
            velocity_m_s=[-3.0, 0.1, 0.2],
            frame_id=42,
        )

        logger.record("publish", ball=ball, received_monotonic_s=9.998)
        logger.record("ball_drop", extra={"reason": "invalid"}, received_monotonic_s=9.999)
        clock.advance(1.0)
        logger.tick()

        self.assertEqual(
            output.getvalue(),
            "mocap fps=2.0 latency_ms=1.5 valid_rate=50.0% drop=1 drop_reason=invalid:1\n",
        )

    def test_prints_source_status_when_no_ball_frames_arrive(self):
        clock = _FakeClock(10.0)
        output = io.StringIO()
        logger = DiagnosticsLogger(
            enabled=False,
            print_interval_s=1.0,
            monotonic_clock=clock,
            output=output,
        )

        clock.advance(1.0)
        logger.tick(source_status="source=natnet packets=0 last_msg=n/a")

        self.assertEqual(
            output.getvalue(),
            "mocap fps=0.0 latency_ms=n/a valid_rate=n/a drop=0 source=natnet packets=0 last_msg=n/a\n",
        )


class HopeOptitrackRuntimeTest(unittest.TestCase):
    @patch("mocap.hope_optitrack_adapter.main.HopeUdpBallReceiver")
    @patch("mocap.hope_optitrack_adapter.main.NatNetRigidBodyBallReceiver")
    @patch("mocap.hope_optitrack_adapter.main.ZmqPublisher")
    def test_default_runtime_uses_direct_natnet_receiver(self, publisher_class, natnet_class, udp_class):
        adapter = HopeOptitrackAdapter(config_path="mocap/hope_optitrack_adapter/config.yaml", enable_ros=False)
        adapter.close()

        natnet_class.assert_called_once()
        udp_class.assert_not_called()

    def test_robot_pose_is_published_when_no_ball_frame_arrives(self):
        robot = RobotPose(
            timestamp_s=3.25,
            position_m=[1.37, -0.7625, 0.40],
            quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
            name="P1",
        )
        robot_receiver = _FakeRobotReceiver(robot)
        publisher = _FakePublisher()
        diagnostics = _FakeDiagnostics()
        adapter = object.__new__(HopeOptitrackAdapter)
        adapter.robot_receiver = robot_receiver
        adapter.ball_receiver = _FakeBallReceiver()
        adapter.publisher = publisher
        adapter.diagnostics = diagnostics
        adapter.config = SimpleNamespace(robot_static_transform=RobotStaticTransform.identity())
        adapter.coordinate_adapter = CoordinateAdapter.default_hope_to_planner()
        adapter._last_published_robot_timestamp_s = None

        adapter.process_once()
        adapter.process_once()

        self.assertEqual(robot_receiver.spin_timeouts, [0.0, 0.0])
        self.assertEqual(len(publisher.messages), 1)
        message = publisher.messages[0]
        self.assertEqual(message["type"], "robot")
        self.assertEqual(message["t"], 3.25)
        self.assertEqual(message["quat"], [0.0, 0.0, 0.0, 1.0])
        self.assertTrue(message["valid"])
        for actual, expected in zip(message["pos"], [0.0, 0.0, 420.0]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(diagnostics.tick_calls, 2)


class HopeUdpBallReceiverTest(unittest.TestCase):
    @patch("mocap.hope_optitrack_adapter.hope_udp_json_receiver.socket.socket")
    def test_configures_five_millisecond_idle_poll(self, socket_factory):
        socket_instance = MagicMock()
        socket_factory.return_value = socket_instance

        HopeUdpBallReceiver()

        socket_instance.settimeout.assert_called_once_with(0.005)


class NatNetPacketParserTest(unittest.TestCase):
    def test_model_definition_maps_rigid_body_names_to_ids(self):
        parser = NatNetPacketParser(bitstream_version=(4, 1))

        parser.parse_packet(_natnet_modeldef_packet({"table": 1, "ball": 2}))

        self.assertEqual(parser.rigid_body_names_by_id, {1: "table", 2: "ball"})
        self.assertEqual(parser.rigid_body_ids_by_name, {"table": 1, "ball": 2})

    def test_model_definition_accepts_sized_dataset_blocks_and_skips_unneeded_types(self):
        parser = NatNetPacketParser(bitstream_version=(4, 1))

        parser.parse_packet(
            _natnet_modeldef_packet_with_sized_blocks(
                {"table": 1, "ball": 2},
                unsupported_blocks=[(6, b"asset data this adapter does not consume")],
            )
        )

        self.assertEqual(parser.rigid_body_names_by_id, {1: "table", 2: "ball"})
        self.assertEqual(parser.rigid_body_ids_by_name, {"table": 1, "ball": 2})

    def test_frame_packet_decodes_named_rigid_bodies_and_timestamp(self):
        parser = NatNetPacketParser(bitstream_version=(4, 1))
        parser.parse_packet(_natnet_modeldef_packet({"table": 1, "ball": 2}))

        frame = parser.parse_packet(
            _natnet_frame_packet(
                frame_number=25,
                timestamp_s=8.5,
                rigid_bodies={
                    1: ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], True),
                    2: ([1.0, -0.5, 0.25], [0.0, 0.0, 0.0, 1.0], True),
                },
            )
        )

        self.assertEqual(frame.frame_id, 25)
        self.assertEqual(frame.timestamp_s, 8.5)
        self.assertEqual(frame.rigid_bodies["ball"].position_m, [1.0, -0.5, 0.25])
        self.assertTrue(frame.rigid_bodies["ball"].tracking_valid)


class NatNetRigidBodyBallReceiverTest(unittest.TestCase):
    def test_reports_natnet_status_when_no_packets_arrive(self):
        receiver = NatNetRigidBodyBallReceiver(
            server_ip="192.168.50.1",
            local_ip="192.168.50.2",
            ball_name="ball",
            table_name="table",
            transport=_FakeNatNetTransport([]),
        )

        self.assertIsNone(receiver.recv_frame())

        status = receiver.diagnostic_status()
        self.assertIn("source=natnet", status)
        self.assertIn("packets=0", status)
        self.assertIn("last_msg=n/a", status)
        self.assertIn("rb=[]", status)
        receiver.close()

    def test_ignores_unparseable_natnet_packets_without_crashing(self):
        receiver = NatNetRigidBodyBallReceiver(
            server_ip="192.168.50.1",
            local_ip="192.168.50.2",
            ball_name="ball",
            table_name="table",
            transport=_FakeNatNetTransport(
                [
                    _natnet_unsupported_modeldef_packet(),
                    _natnet_modeldef_packet({"table": 1, "ball": 2}),
                    _natnet_frame_packet(
                        frame_number=12,
                        timestamp_s=1.02,
                        rigid_bodies={2: ([0.96, -0.48, 0.32], [0.0, 0.0, 0.0, 1.0], True)},
                    ),
                ]
            ),
        )

        frame = receiver.recv_frame()

        self.assertEqual(frame["frame"], 12)
        self.assertIn("unsupported NatNet model definition dataset type", receiver.last_error)
        receiver.close()

    def test_converts_direct_natnet_rigid_body_frames_to_validator_shape(self):
        receiver = NatNetRigidBodyBallReceiver(
            server_ip="192.168.50.1",
            local_ip="192.168.50.2",
            ball_name="ball",
            table_name="table",
            transport=_FakeNatNetTransport(
                [
                    _natnet_modeldef_packet({"table": 1, "ball": 2}),
                    _natnet_frame_packet(
                        frame_number=10,
                        timestamp_s=1.0,
                        rigid_bodies={2: ([1.00, -0.50, 0.30], [0.0, 0.0, 0.0, 1.0], True)},
                    ),
                    _natnet_frame_packet(
                        frame_number=11,
                        timestamp_s=1.01,
                        rigid_bodies={2: ([0.97, -0.49, 0.31], [0.0, 0.0, 0.0, 1.0], True)},
                    ),
                ]
            ),
        )

        first = receiver.recv_frame()
        second = receiver.recv_frame()

        self.assertEqual(first["source"], "natnet_rigid_body")
        self.assertEqual(first["selected"], 2)
        self.assertEqual(first["frame"], 10)
        self.assertEqual(first["velocity"], [0.0, 0.0, 0.0])
        self.assertEqual(second["frame"], 11)
        for actual, expected in zip(second["position"], [0.97, -0.49, 0.31]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(second["velocity"], [-3.0, 1.0, 1.0]):
            self.assertAlmostEqual(actual, expected, places=5)
        receiver.close()


class _FakeClock:
    def __init__(self, value: float) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


class _FakeRobotReceiver:
    def __init__(self, robot: RobotPose) -> None:
        self._robot = robot
        self.spin_timeouts: list[float] = []

    def spin_once(self, timeout_sec: float) -> None:
        self.spin_timeouts.append(timeout_sec)

    def latest_robot_pose(self) -> RobotPose:
        return self._robot


class _FakeBallReceiver:
    last_received_monotonic_s = 10.0

    def recv_frame(self):
        return None


class _FakePublisher:
    def __init__(self) -> None:
        self.messages: list[dict] = []

    def send(self, message: dict) -> None:
        self.messages.append(message)


class _FakeDiagnostics:
    def __init__(self) -> None:
        self.tick_calls = 0

    def tick(self, *_args, **_kwargs) -> None:
        self.tick_calls += 1


class _FakeNatNetTransport:
    def __init__(self, packets: list[bytes]) -> None:
        self.packets = list(packets)
        self.started = False
        self.closed = False

    def start(self) -> None:
        self.started = True

    def recv_packet(self, timeout_s: float | None = None) -> bytes | None:
        if not self.packets:
            return None
        return self.packets.pop(0)

    def close(self) -> None:
        self.closed = True


def _pose(position, quaternion):
    return SimpleNamespace(
        position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
        orientation=SimpleNamespace(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        ),
    )


def _transform(child_frame_id, timestamp_s, translation, quaternion):
    sec = int(timestamp_s)
    nanosec = int(round((timestamp_s - sec) * 1_000_000_000))
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        child_frame_id=child_frame_id,
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=translation[0], y=translation[1], z=translation[2]),
            rotation=SimpleNamespace(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3],
            ),
        ),
    )


def _natnet_packet(message_id: int, payload: bytes) -> bytes:
    return struct.pack("<HH", message_id, len(payload)) + payload


def _natnet_modeldef_packet(names_by_name: dict[str, int]) -> bytes:
    payload = struct.pack("<I", len(names_by_name))
    for name, rigid_body_id in names_by_name.items():
        payload += struct.pack("<I", 1)
        payload += name.encode("utf-8") + b"\0"
        payload += struct.pack("<ii", rigid_body_id, 0)
        payload += struct.pack("<fff", 0.0, 0.0, 0.0)
        payload += struct.pack("<I", 0)
    return _natnet_packet(5, payload)


def _natnet_modeldef_packet_with_sized_blocks(
    names_by_name: dict[str, int],
    unsupported_blocks: list[tuple[int, bytes]] | None = None,
) -> bytes:
    blocks: list[tuple[int, bytes]] = []
    for name, rigid_body_id in names_by_name.items():
        block = name.encode("utf-8") + b"\0"
        block += struct.pack("<ii", rigid_body_id, 0)
        block += struct.pack("<fff", 0.0, 0.0, 0.0)
        block += struct.pack("<I", 0)
        blocks.append((1, block))
    blocks.extend(unsupported_blocks or [])

    payload = struct.pack("<I", len(blocks))
    for dataset_type, block in blocks:
        payload += struct.pack("<II", dataset_type, len(block))
        payload += block
    return _natnet_packet(5, payload)


def _natnet_unsupported_modeldef_packet() -> bytes:
    return _natnet_packet(5, struct.pack("<II", 1, 99))


def _natnet_frame_packet(frame_number: int, timestamp_s: float, rigid_bodies: dict[int, tuple]) -> bytes:
    payload = struct.pack("<I", frame_number)
    payload += struct.pack("<II", 0, 0)  # marker sets + NatNet 4.1 byte count
    payload += struct.pack("<II", 0, 0)  # legacy markers + byte count
    payload += struct.pack("<II", len(rigid_bodies), 0)
    for rigid_body_id, (position, quaternion, valid) in rigid_bodies.items():
        payload += struct.pack("<I", rigid_body_id)
        payload += struct.pack("<fff", *position)
        payload += struct.pack("<ffff", *quaternion)
        payload += struct.pack("<f", 0.0)
        payload += struct.pack("<h", 1 if valid else 0)
    payload += struct.pack("<II", 0, 0)  # skeletons + byte count
    payload += struct.pack("<II", 0, 0)  # NatNet 4.1 assets + byte count
    payload += struct.pack("<II", 0, 0)  # labeled markers + byte count
    payload += struct.pack("<II", 0, 0)  # force plates + byte count
    payload += struct.pack("<II", 0, 0)  # devices + byte count
    payload += struct.pack("<II", 0, 0)  # timecode
    payload += struct.pack("<d", timestamp_s)
    payload += struct.pack("<QQQ", 0, 0, 0)
    payload += struct.pack("<II", int(timestamp_s), 0)
    payload += struct.pack("<h", 0)
    return _natnet_packet(7, payload)


if __name__ == "__main__":
    unittest.main()
